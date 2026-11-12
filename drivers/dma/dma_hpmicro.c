/*
 * Copyright (c) 2024 HPMicro
 * Copyright (c) 2026 Cody Gu <gujiaqi@iscas.ac.cn>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hpmicro_hpm_dma

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include <hpm_clock_drv.h>
#include <hpm_common.h>
#include <hpm_dmamux_drv.h>
#include <hpm_dmav2_drv.h>

#define HPM_DMA_NO_REQUEST 0xFFU

struct hpm_dma_channel_data {
	struct k_spinlock lock;
	dma_callback_t callback;
	void *user_data;
	uint32_t direction;
	uint32_t block_size;
	uint32_t data_size;
	uint32_t interrupt_mask;
	bool busy;
	bool cyclic;
	bool error_callback_dis;
};

struct hpm_dma_config {
	DMAV2_Type *base;
	DMAMUX_Type *dmamux;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
	clock_name_t clock_name;
	clk_src_t clock_src;
	uint32_t clock_div;
	uint32_t max_requests;
	void (*irq_config_func)(const struct device *dev);
};

struct hpm_dma_data {
	struct dma_context ctx;
	struct hpm_dma_channel_data *channels;
};

static bool hpm_dma_channel_is_valid(const struct device *dev, uint32_t channel)
{
	const struct hpm_dma_data *data = dev->data;

	return channel < (uint32_t)data->ctx.dma_channels;
}

static uint8_t hpm_dma_width(uint32_t size)
{
	switch (size) {
	case 1U:
		return DMA_TRANSFER_WIDTH_BYTE;
	case 2U:
		return DMA_TRANSFER_WIDTH_HALF_WORD;
	case 4U:
		return DMA_TRANSFER_WIDTH_WORD;
	default:
		return HPM_DMA_NO_REQUEST;
	}
}

static uint32_t hpm_dma_size_to_shift(uint32_t size)
{
	switch (size) {
	case 1U:
		return 0U;
	case 2U:
		return 1U;
	case 4U:
		return 2U;
	default:
		return UINT32_MAX;
	}
}

static bool hpm_dma_size_is_valid(size_t size, uint32_t data_size)
{
	uint32_t shift = hpm_dma_size_to_shift(data_size);
	uint32_t max_count =
		DMAV2_CHCTRL_TRANSIZE_TRANSIZE_MASK >> DMAV2_CHCTRL_TRANSIZE_TRANSIZE_SHIFT;

	if ((shift == UINT32_MAX) || (size == 0U) || !IS_ALIGNED(size, data_size)) {
		return false;
	}

	return (size >> shift) <= max_count;
}

static uint8_t hpm_dma_addr_adjustment(uint16_t adjustment)
{
	switch (adjustment) {
	case DMA_ADDR_ADJ_INCREMENT:
		return DMA_ADDRESS_CONTROL_INCREMENT;
	case DMA_ADDR_ADJ_DECREMENT:
		return DMA_ADDRESS_CONTROL_DECREMENT;
	case DMA_ADDR_ADJ_NO_CHANGE:
		return DMA_ADDRESS_CONTROL_FIXED;
	default:
		return HPM_DMA_NO_REQUEST;
	}
}

static uint8_t hpm_dma_burst_size(uint32_t burst_length, uint32_t data_size)
{
	uint32_t transfers;

	if ((burst_length == 0U) || (data_size == 0U) || ((burst_length % data_size) != 0U)) {
		return HPM_DMA_NO_REQUEST;
	}

	transfers = burst_length / data_size;
	if ((transfers & (transfers - 1U)) != 0U) {
		return HPM_DMA_NO_REQUEST;
	}

	if (transfers > 1024U) {
		return HPM_DMA_NO_REQUEST;
	}

	switch (transfers) {
	case 1U:
		return DMA_NUM_TRANSFER_PER_BURST_1T;
	case 2U:
		return DMA_NUM_TRANSFER_PER_BURST_2T;
	case 4U:
		return DMA_NUM_TRANSFER_PER_BURST_4T;
	case 8U:
		return DMA_NUM_TRANSFER_PER_BURST_8T;
	case 16U:
		return DMA_NUM_TRANSFER_PER_BURST_16T;
	case 32U:
		return DMA_NUM_TRANSFER_PER_BURST_32T;
	case 64U:
		return DMA_NUM_TRANSFER_PER_BURST_64T;
	case 128U:
		return DMA_NUM_TRANSFER_PER_BURST_128T;
	case 256U:
		return DMA_NUM_TRANSFER_PER_BURST_256T;
	case 512U:
		return DMA_NUM_TRANSFER_PER_BURST_512T;
	case 1024U:
		return DMA_NUM_TRANSFER_PER_BURST_1024T;
	default:
		return HPM_DMA_NO_REQUEST;
	}
}

static void hpm_dma_set_request(const struct device *dev, uint32_t channel, uint32_t request)
{
	const struct hpm_dma_config *config = dev->config;

	config->base->CHCTRL[channel].CHANREQCTRL =
		DMAV2_CHCTRL_CHANREQCTRL_SRCREQSEL_SET(channel) |
		DMAV2_CHCTRL_CHANREQCTRL_DSTREQSEL_SET(channel);

	dmamux_config(config->dmamux, channel, request == HPM_DMA_NO_REQUEST ? 0U : request,
		      request != HPM_DMA_NO_REQUEST);
}

static int hpm_dma_config(const struct device *dev, uint32_t channel, struct dma_config *dma_cfg)
{
	const struct hpm_dma_config *config = dev->config;
	struct hpm_dma_data *data = dev->data;
	struct hpm_dma_channel_data *channel_data;
	const struct dma_block_config *block;
	dma_channel_config_t hal_cfg;
	uint8_t src_width;
	uint8_t dst_width;
	uint8_t src_adj;
	uint8_t dst_adj;
	uint8_t burst_size;
	uint8_t src_mode = DMA_HANDSHAKE_MODE_NORMAL;
	uint8_t dst_mode = DMA_HANDSHAKE_MODE_NORMAL;
	hpm_stat_t status;
	k_spinlock_key_t key;
	uint32_t request = HPM_DMA_NO_REQUEST;

	if (!hpm_dma_channel_is_valid(dev, channel)) {
		return -EINVAL;
	}

	if ((dma_cfg == NULL) || (dma_cfg->head_block == NULL)) {
		return -EINVAL;
	}

	if (dma_cfg->block_count != 1U) {
		return -ENOTSUP;
	}

	if ((dma_cfg->source_data_size == 0U) || (dma_cfg->source_data_size > 4U) ||
	    (dma_cfg->dest_data_size == 0U) || (dma_cfg->dest_data_size > 4U) ||
	    (dma_cfg->channel_priority > 1U)) {
		return -EINVAL;
	}

	if (dma_cfg->source_data_size != dma_cfg->dest_data_size) {
		return -ENOTSUP;
	}

	if ((dma_cfg->source_burst_length != 0U) && (dma_cfg->dest_burst_length != 0U) &&
	    (dma_cfg->source_burst_length != dma_cfg->dest_burst_length)) {
		return -ENOTSUP;
	}

	if ((dma_cfg->source_chaining_en) || (dma_cfg->dest_chaining_en) ||
	    (dma_cfg->linked_channel != 0U)) {
		return -ENOTSUP;
	}

	src_width = hpm_dma_width(dma_cfg->source_data_size);
	dst_width = hpm_dma_width(dma_cfg->dest_data_size);
	if ((src_width == HPM_DMA_NO_REQUEST) || (dst_width == HPM_DMA_NO_REQUEST)) {
		return -EINVAL;
	}

	block = dma_cfg->head_block;
	if ((block->next_block != NULL) || block->source_gather_en || block->dest_scatter_en ||
	    block->source_reload_en || block->dest_reload_en || (block->fifo_mode_control != 0U) ||
	    block->flow_control_mode) {
		return -ENOTSUP;
	}

	if (!hpm_dma_size_is_valid(block->block_size, dma_cfg->source_data_size) ||
	    !IS_ALIGNED(block->source_address, dma_cfg->source_data_size) ||
	    !IS_ALIGNED(block->dest_address, dma_cfg->source_data_size)) {
		return -EINVAL;
	}

	src_adj = hpm_dma_addr_adjustment(block->source_addr_adj);
	dst_adj = hpm_dma_addr_adjustment(block->dest_addr_adj);
	if ((src_adj == HPM_DMA_NO_REQUEST) || (dst_adj == HPM_DMA_NO_REQUEST)) {
		return -EINVAL;
	}

	burst_size =
		hpm_dma_burst_size(dma_cfg->source_burst_length != 0U ? dma_cfg->source_burst_length
								      : dma_cfg->source_data_size,
				   dma_cfg->source_data_size);
	if (burst_size == HPM_DMA_NO_REQUEST) {
		return -ENOTSUP;
	}

	switch (dma_cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
		break;
	case MEMORY_TO_PERIPHERAL:
		dst_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
		request = dma_cfg->dma_slot;
		break;
	case PERIPHERAL_TO_MEMORY:
		src_mode = DMA_HANDSHAKE_MODE_HANDSHAKE;
		request = dma_cfg->dma_slot;
		break;
	default:
		return -ENOTSUP;
	}

	if ((dma_cfg->channel_direction != MEMORY_TO_MEMORY) &&
	    (dma_cfg->dma_slot >= config->max_requests)) {
		return -EINVAL;
	}

	if ((dma_cfg->channel_direction != MEMORY_TO_MEMORY) &&
	    (dma_cfg->source_burst_length != 0U) &&
	    (dma_cfg->source_burst_length != dma_cfg->source_data_size)) {
		return -ENOTSUP;
	}

	channel_data = &data->channels[channel];
	key = k_spin_lock(&channel_data->lock);
	if (channel_data->busy) {
		k_spin_unlock(&channel_data->lock, key);
		return -EBUSY;
	}

	hal_cfg = (dma_channel_config_t){0};
	dma_default_channel_config(config->base, &hal_cfg);
	hal_cfg.src_addr = block->source_address;
	hal_cfg.dst_addr = block->dest_address;
	hal_cfg.size_in_byte = block->block_size;
	hal_cfg.src_width = src_width;
	hal_cfg.dst_width = dst_width;
	hal_cfg.src_addr_ctrl = src_adj;
	hal_cfg.dst_addr_ctrl = dst_adj;
	hal_cfg.src_mode = src_mode;
	hal_cfg.dst_mode = dst_mode;
	hal_cfg.src_burst_size = burst_size;
	hal_cfg.priority = dma_cfg->channel_priority;
	hal_cfg.en_infiniteloop = dma_cfg->cyclic;
	hal_cfg.interrupt_mask =
		dma_cfg->half_complete_callback_en ? 0U : DMA_INTERRUPT_MASK_HALF_TC;
	/* Error interrupts maintain channel state even when error callbacks are disabled. */

	status = dma_setup_channel(config->base, channel, &hal_cfg, false);
	if (status != status_success) {
		k_spin_unlock(&channel_data->lock, key);
		return status == status_dma_alignment_error ? -EINVAL : -EIO;
	}

	hpm_dma_set_request(dev, channel, request);

	channel_data->callback = dma_cfg->dma_callback;
	channel_data->user_data = dma_cfg->user_data;
	channel_data->direction = dma_cfg->channel_direction;
	channel_data->block_size = block->block_size;
	channel_data->data_size = dma_cfg->source_data_size;
	channel_data->cyclic = dma_cfg->cyclic;
	channel_data->error_callback_dis = dma_cfg->error_callback_dis;
	channel_data->interrupt_mask = hal_cfg.interrupt_mask;
	channel_data->busy = false;
	k_spin_unlock(&channel_data->lock, key);

	return 0;
}

static int hpm_dma_reload(const struct device *dev, uint32_t channel, uint32_t src, uint32_t dst,
			  size_t size)
{
	const struct hpm_dma_config *config = dev->config;
	struct hpm_dma_data *data = dev->data;
	struct hpm_dma_channel_data *channel_data;
	uint32_t shift;
	k_spinlock_key_t key;

	if (!hpm_dma_channel_is_valid(dev, channel)) {
		return -EINVAL;
	}

	channel_data = &data->channels[channel];
	key = k_spin_lock(&channel_data->lock);
	if (channel_data->busy) {
		k_spin_unlock(&channel_data->lock, key);
		return -EBUSY;
	}

	if (!hpm_dma_size_is_valid(size, channel_data->data_size) ||
	    !IS_ALIGNED(src, channel_data->data_size) ||
	    !IS_ALIGNED(dst, channel_data->data_size)) {
		k_spin_unlock(&channel_data->lock, key);
		return -EINVAL;
	}

	shift = hpm_dma_size_to_shift(channel_data->data_size);
	dma_set_source_address(config->base, channel, src);
	dma_set_destination_address(config->base, channel, dst);
	dma_set_transfer_size(config->base, channel, size >> shift);
	channel_data->block_size = size;
	k_spin_unlock(&channel_data->lock, key);

	return 0;
}

static int hpm_dma_start(const struct device *dev, uint32_t channel)
{
	const struct hpm_dma_config *config = dev->config;
	struct hpm_dma_data *data = dev->data;
	struct hpm_dma_channel_data *channel_data;
	k_spinlock_key_t key;

	if (!hpm_dma_channel_is_valid(dev, channel)) {
		return -EINVAL;
	}

	channel_data = &data->channels[channel];
	key = k_spin_lock(&channel_data->lock);
	if (channel_data->busy) {
		k_spin_unlock(&channel_data->lock, key);
		return 0;
	}

	dma_clear_transfer_status(config->base, channel);
	channel_data->busy = true;
	/* A transfer can complete or fault before CHEN is read back. */
	config->base->CHCTRL[channel].CTRL |= DMAV2_CHCTRL_CTRL_ENABLE_MASK;
	k_spin_unlock(&channel_data->lock, key);

	return 0;
}

static int hpm_dma_stop(const struct device *dev, uint32_t channel)
{
	const struct hpm_dma_config *config = dev->config;
	struct hpm_dma_data *data = dev->data;
	struct hpm_dma_channel_data *channel_data;
	k_spinlock_key_t key;

	if (!hpm_dma_channel_is_valid(dev, channel)) {
		return -EINVAL;
	}

	channel_data = &data->channels[channel];
	key = k_spin_lock(&channel_data->lock);
	dma_disable_channel(config->base, channel);
	dma_clear_transfer_status(config->base, channel);
	channel_data->busy = false;
	k_spin_unlock(&channel_data->lock, key);

	return 0;
}

static int hpm_dma_get_status(const struct device *dev, uint32_t channel, struct dma_status *status)
{
	const struct hpm_dma_config *config = dev->config;
	struct hpm_dma_data *data = dev->data;
	struct hpm_dma_channel_data *channel_data;
	k_spinlock_key_t key;
	uint32_t remaining;

	if ((status == NULL) || !hpm_dma_channel_is_valid(dev, channel)) {
		return -EINVAL;
	}

	channel_data = &data->channels[channel];
	key = k_spin_lock(&channel_data->lock);
	remaining = dma_get_remaining_transfer_size(config->base, channel);
	status->busy = channel_data->busy;
	status->dir = channel_data->direction;
	status->pending_length = remaining * channel_data->data_size;
	status->write_position = 0U;
	status->read_position = 0U;
	status->free = 0U;
	status->total_copied = channel_data->block_size >= status->pending_length
				       ? channel_data->block_size - status->pending_length
				       : 0U;
	k_spin_unlock(&channel_data->lock, key);

	return 0;
}

static int hpm_dma_get_attribute(const struct device *dev, uint32_t type, uint32_t *value)
{
	ARG_UNUSED(dev);

	if (value == NULL) {
		return -EINVAL;
	}

	switch (type) {
	case DMA_ATTR_BUFFER_ADDRESS_ALIGNMENT:
	case DMA_ATTR_BUFFER_SIZE_ALIGNMENT:
	case DMA_ATTR_COPY_ALIGNMENT:
		*value = 1U;
		return 0;
	case DMA_ATTR_MAX_BLOCK_COUNT:
		*value = 1U;
		return 0;
	default:
		return -EINVAL;
	}
}

static bool hpm_dma_chan_filter(const struct device *dev, int channel, void *filter_param)
{
	const struct hpm_dma_data *data = dev->data;
	struct hpm_dma_channel_data *channel_data;
	k_spinlock_key_t key;
	bool available;

	if (!hpm_dma_channel_is_valid(dev, (uint32_t)channel)) {
		return false;
	}

	channel_data = &data->channels[channel];
	key = k_spin_lock(&channel_data->lock);
	available = !channel_data->busy && ((filter_param == NULL) ||
					    (*(const uint32_t *)filter_param == (uint32_t)channel));
	k_spin_unlock(&channel_data->lock, key);

	return available;
}

static void hpm_dma_chan_release(const struct device *dev, uint32_t channel)
{
	const struct hpm_dma_config *config = dev->config;
	struct hpm_dma_data *data = dev->data;
	struct hpm_dma_channel_data *channel_data = &data->channels[channel];
	k_spinlock_key_t key;

	key = k_spin_lock(&channel_data->lock);
	dma_disable_channel(config->base, channel);
	dma_clear_transfer_status(config->base, channel);
	dmamux_config(config->dmamux, channel, 0U, false);
	channel_data->callback = NULL;
	channel_data->user_data = NULL;
	channel_data->busy = false;
	k_spin_unlock(&channel_data->lock, key);
}

static void hpm_dma_isr(const struct device *dev)
{
	const struct hpm_dma_config *config = dev->config;
	struct hpm_dma_data *data = dev->data;
	uint32_t channel_count = (uint32_t)data->ctx.dma_channels;

	for (uint32_t channel = 0U; channel < channel_count; channel++) {
		struct hpm_dma_channel_data *channel_data = &data->channels[channel];
		k_spinlock_key_t key;
		uint32_t status;
		dma_callback_t callback;
		void *user_data;
		uint32_t interrupt_mask;
		bool error_callback_dis;
		int callback_status = DMA_STATUS_COMPLETE;

		key = k_spin_lock(&channel_data->lock);
		if (!channel_data->busy) {
			k_spin_unlock(&channel_data->lock, key);
			continue;
		}

		status = dma_check_transfer_status(config->base, channel);
		if (status == DMA_CHANNEL_STATUS_ONGOING) {
			k_spin_unlock(&channel_data->lock, key);
			continue;
		}

		interrupt_mask = channel_data->interrupt_mask;
		if ((status & DMA_CHANNEL_STATUS_ERROR) != 0U) {
			callback_status = -EIO;
		} else if ((status & DMA_CHANNEL_STATUS_ABORT) != 0U) {
			callback_status = -ECANCELED;
		} else if (((status & DMA_CHANNEL_STATUS_HALF_TC) != 0U) &&
			   ((status & DMA_CHANNEL_STATUS_TC) == 0U)) {
			callback_status = DMA_STATUS_HALF_COMPLETE;
		}

		if ((status & (DMA_CHANNEL_STATUS_ERROR | DMA_CHANNEL_STATUS_ABORT)) != 0U) {
			dma_disable_channel(config->base, channel);
			channel_data->busy = false;
		} else if (((status & DMA_CHANNEL_STATUS_TC) != 0U) && !channel_data->cyclic) {
			channel_data->busy = false;
		}

		callback = channel_data->callback;
		user_data = channel_data->user_data;
		error_callback_dis = channel_data->error_callback_dis;
		k_spin_unlock(&channel_data->lock, key);

		if ((callback != NULL) && (((callback_status >= 0) &&
					    !((interrupt_mask & DMA_INTERRUPT_MASK_HALF_TC) != 0U &&
					      callback_status == DMA_STATUS_HALF_COMPLETE)) ||
					   ((callback_status < 0) && !error_callback_dis))) {
			callback(dev, user_data, channel, callback_status);
		}
	}
}

static int hpm_dma_init(const struct device *dev)
{
	const struct hpm_dma_config *config = dev->config;
	struct hpm_dma_data *data = dev->data;
	int ret;

	if (!device_is_ready(config->clock_dev)) {
		return -ENODEV;
	}

	if (GET_CLK_SRC_GROUP_FROM_NAME(config->clock_name) != CLK_SRC_GROUP_AHB) {
		ret = clock_set_source_divider(config->clock_name, config->clock_src,
					       config->clock_div);
		if (ret != status_success) {
			return -EINVAL;
		}
	}

	ret = clock_control_on(config->clock_dev, config->clock_subsys);
	if (ret < 0) {
		return ret;
	}

	for (uint32_t channel = 0U; channel < (uint32_t)data->ctx.dma_channels; channel++) {
		dma_disable_channel(config->base, channel);
		dma_clear_transfer_status(config->base, channel);
	}

	config->irq_config_func(dev);

	return 0;
}

static DEVICE_API(dma, hpm_dma_api) = {
	.config = hpm_dma_config,
	.reload = hpm_dma_reload,
	.start = hpm_dma_start,
	.stop = hpm_dma_stop,
	.get_status = hpm_dma_get_status,
	.get_attribute = hpm_dma_get_attribute,
	.chan_filter = hpm_dma_chan_filter,
	.chan_release = hpm_dma_chan_release,
};

#define HPM_DMA_IRQ_CONFIG(n)                                                                      \
	static void hpm_dma_irq_config_##n(const struct device *dev)                               \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), hpm_dma_isr,                \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define HPM_DMA_INIT(n)                                                                            \
	HPM_DMA_IRQ_CONFIG(n)                                                                      \
                                                                                                   \
	static struct hpm_dma_channel_data hpm_dma_channels_##n[DT_INST_PROP(n, dma_channels)];    \
	ATOMIC_DEFINE(hpm_dma_atomic_##n, DT_INST_PROP(n, dma_channels));                          \
                                                                                                   \
	static struct hpm_dma_data hpm_dma_data_##n = {                                            \
		.ctx =                                                                             \
			{                                                                          \
				.magic = DMA_MAGIC,                                                \
				.dma_channels = DT_INST_PROP(n, dma_channels),                     \
				.atomic = hpm_dma_atomic_##n,                                      \
			},                                                                         \
		.channels = hpm_dma_channels_##n,                                                  \
	};                                                                                         \
                                                                                                   \
	static const struct hpm_dma_config hpm_dma_config_##n = {                                  \
		.base = (DMAV2_Type *)DT_INST_REG_ADDR_BY_IDX(n, 0),                               \
		.dmamux = (DMAMUX_Type *)DT_INST_REG_ADDR_BY_IDX(n, 1),                            \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_subsys = (clock_control_subsys_t)DT_INST_CLOCKS_CELL(n, name),              \
		.clock_name = (clock_name_t)DT_INST_CLOCKS_CELL(n, name),                          \
		.clock_src = (clk_src_t)DT_INST_CLOCKS_CELL(n, source),                            \
		.clock_div = DT_INST_CLOCKS_CELL(n, divider),                                      \
		.max_requests = DT_INST_PROP(n, dma_requests),                                     \
		.irq_config_func = hpm_dma_irq_config_##n,                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, hpm_dma_init, NULL, &hpm_dma_data_##n, &hpm_dma_config_##n,       \
			      PRE_KERNEL_1, CONFIG_DMA_INIT_PRIORITY, &hpm_dma_api);

DT_INST_FOREACH_STATUS_OKAY(HPM_DMA_INIT)
