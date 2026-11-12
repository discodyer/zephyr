/*
 * Copyright (c) 2022 HPMicro
 * Copyright (c) 2026 Cody Gu <gujiaqi@iscas.ac.cn>
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT hpmicro_hpm_uart

#include <errno.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>

#include <hpm_clock_drv.h>
#include <hpm_soc.h>
#include <hpm_uart_drv.h>

/* The UART interrupt ID is 1 when no enabled interrupt is pending. */
#define HPM_UART_IRQ_ID_NONE 1U

struct hpm_uart_dev_config {
	UART_Type *base;
	const struct device *clock_dev;
	clock_name_t clock_name;
	clk_src_t clock_source;
	uint32_t clock_divider;
	uint32_t baud_rate;
	uint8_t parity;
	uint8_t stop_bits;
	uint8_t data_bits;
	uint8_t flow_ctrl;
	const struct pinctrl_dev_config *pincfg;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_config_func_t irq_config_func;
#endif
};

struct hpm_uart_dev_data {
	struct uart_config uart_config;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *callback_data;
	uint32_t iir;
#endif
};

static int hpm_uart_configure(const struct device *dev, const struct uart_config *cfg)
{
	const struct hpm_uart_dev_config *config = dev->config;
	struct hpm_uart_dev_data *data = dev->data;
	uart_config_t uart_config;
	parity_setting_t parity;
	num_of_stop_bits_t stop_bits;
	word_length_t word_length;
	uint32_t clock_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uint32_t irq_mask;
#endif
	hpm_stat_t status;
	int ret;

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		parity = parity_none;
		break;
	case UART_CFG_PARITY_ODD:
		parity = parity_odd;
		break;
	case UART_CFG_PARITY_EVEN:
		parity = parity_even;
		break;
	case UART_CFG_PARITY_MARK:
		parity = parity_always_1;
		break;
	case UART_CFG_PARITY_SPACE:
		parity = parity_always_0;
		break;
	default:
		return -ENOTSUP;
	}

	switch (cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		word_length = word_length_5_bits;
		break;
	case UART_CFG_DATA_BITS_6:
		word_length = word_length_6_bits;
		break;
	case UART_CFG_DATA_BITS_7:
		word_length = word_length_7_bits;
		break;
	case UART_CFG_DATA_BITS_8:
		word_length = word_length_8_bits;
		break;
	default:
		return -ENOTSUP;
	}

	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		stop_bits = stop_bits_1;
		break;
	case UART_CFG_STOP_BITS_1_5:
		stop_bits = stop_bits_1_5;
		break;
	case UART_CFG_STOP_BITS_2:
		stop_bits = stop_bits_2;
		break;
	default:
		return -ENOTSUP;
	}

	if (cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
		return -ENOTSUP;
	}

	ret = clock_control_get_rate(config->clock_dev,
				     (clock_control_subsys_t)(uintptr_t)config->clock_name,
				     &clock_rate);
	if (ret < 0) {
		return ret;
	}

	uart_default_config(config->base, &uart_config);
	uart_config.src_freq_in_hz = clock_rate;
	uart_config.baudrate = cfg->baudrate;
	uart_config.parity = parity;
	uart_config.word_length = word_length;
	uart_config.num_of_stop_bits = stop_bits;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	/*
	 * uart_init() disables all interrupts. Preserve the caller's
	 * interrupt enables across runtime reconfiguration.
	 */
	irq_mask = uart_get_enabled_irq(config->base);
#endif

	status = uart_init(config->base, &uart_config);
	if (status != status_success) {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
		uart_enable_irq(config->base, irq_mask);
#endif
		return -EINVAL;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_enable_irq(config->base, irq_mask);
#endif

	data->uart_config = *cfg;

	return 0;
}

static int hpm_uart_config_get(const struct device *dev, struct uart_config *cfg)
{
	const struct hpm_uart_dev_data *data = dev->data;

	*cfg = data->uart_config;

	return 0;
}

static int hpm_uart_poll_in(const struct device *dev, unsigned char *c)
{
	const struct hpm_uart_dev_config *config = dev->config;

	if (!uart_check_status(config->base, uart_stat_data_ready)) {
		return -1;
	}

	*c = (unsigned char)(config->base->RBR & UART_RBR_RBR_MASK);

	return 0;
}

static void hpm_uart_poll_out(const struct device *dev, unsigned char c)
{
	const struct hpm_uart_dev_config *config = dev->config;

	while (!uart_check_status(config->base, uart_stat_tx_slot_avail)) {
	}

	config->base->THR = UART_THR_THR_SET(c);
}

static int hpm_uart_err_check(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;
	uint32_t status = uart_get_status(config->base);
	int err = 0;

	if ((status & UART_LSR_OE_MASK) != 0U) {
		err |= UART_ERROR_OVERRUN;
	}

	if ((status & UART_LSR_PE_MASK) != 0U) {
		err |= UART_ERROR_PARITY;
	}

	if ((status & UART_LSR_FE_MASK) != 0U) {
		err |= UART_ERROR_FRAMING;
	}

	if ((status & UART_LSR_LBREAK_MASK) != 0U) {
		err |= UART_BREAK;
	}

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static uint32_t hpm_uart_read_iir(const struct hpm_uart_dev_config *config)
{
#if defined(HPM_IP_FEATURE_UART_IIR2) && (HPM_IP_FEATURE_UART_IIR2 == 1)
	return config->base->IIR2 & UART_IIR2_INTRID_MASK;
#else
	return config->base->IIR & UART_IIR_INTRID_MASK;
#endif
}

static int hpm_uart_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	const struct hpm_uart_dev_config *config = dev->config;
	int count = 0;

	while ((count < size) && uart_check_status(config->base, uart_stat_tx_slot_avail)) {
		uart_write_byte(config->base, tx_data[count]);
		count++;
	}

	return count;
}

static int hpm_uart_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct hpm_uart_dev_config *config = dev->config;
	int count = 0;

	while ((count < size) && uart_check_status(config->base, uart_stat_data_ready)) {
		rx_data[count] = uart_read_byte(config->base);
		count++;
	}

	return count;
}

static void hpm_uart_irq_tx_enable(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;

	uart_enable_irq(config->base, uart_intr_tx_slot_avail);
}

static void hpm_uart_irq_tx_disable(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;

	uart_disable_irq(config->base, uart_intr_tx_slot_avail);
}

static int hpm_uart_irq_tx_ready(const struct device *dev)
{
	const struct hpm_uart_dev_data *data = dev->data;

	return (data->iir == uart_intr_id_tx_slot_avail) ? 1 : 0;
}

static void hpm_uart_irq_rx_enable(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;

	uart_enable_irq(config->base, uart_intr_rx_data_avail_or_timeout);
}

static void hpm_uart_irq_rx_disable(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;

	uart_disable_irq(config->base, uart_intr_rx_data_avail_or_timeout);
}

static int hpm_uart_irq_tx_complete(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;

	return uart_check_status(config->base, uart_stat_transmitter_empty) ? 1 : 0;
}

static int hpm_uart_irq_rx_ready(const struct device *dev)
{
	const struct hpm_uart_dev_data *data = dev->data;

	if (data->iir == uart_intr_id_rx_data_avail) {
		return 1;
	}

	if (data->iir == uart_intr_id_rx_timeout) {
		return 1;
	}

	return 0;
}

static void hpm_uart_irq_err_enable(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;

	uart_enable_irq(config->base, uart_intr_rx_line_stat);
}

static void hpm_uart_irq_err_disable(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;

	uart_disable_irq(config->base, uart_intr_rx_line_stat);
}

static int hpm_uart_irq_is_pending(const struct device *dev)
{
	const struct hpm_uart_dev_data *data = dev->data;

	return (data->iir != HPM_UART_IRQ_ID_NONE) ? 1 : 0;
}

static void hpm_uart_irq_update(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;
	struct hpm_uart_dev_data *data = dev->data;

	data->iir = hpm_uart_read_iir(config);
}

static void hpm_uart_irq_callback_set(const struct device *dev,
				      uart_irq_callback_user_data_t callback, void *user_data)
{
	struct hpm_uart_dev_data *data = dev->data;

	data->callback = callback;
	data->callback_data = user_data;
}

static void hpm_uart_isr(const struct device *dev)
{
	struct hpm_uart_dev_data *data = dev->data;

	if (data->callback != NULL) {
		data->callback(dev, data->callback_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int hpm_uart_init(const struct device *dev)
{
	const struct hpm_uart_dev_config *config = dev->config;
	struct hpm_uart_dev_data *data = dev->data;
	hpm_stat_t status;
	int ret;

	ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	status = clock_set_source_divider(config->clock_name, config->clock_source,
					  config->clock_divider);
	if (status != status_success) {
		return -EIO;
	}

	ret = clock_control_on(config->clock_dev,
			       (clock_control_subsys_t)(uintptr_t)config->clock_name);
	if (ret < 0) {
		return ret;
	}

	data->uart_config = (struct uart_config){
		.baudrate = config->baud_rate,
		.parity = config->parity,
		.stop_bits = config->stop_bits,
		.flow_ctrl = config->flow_ctrl,
	};

	switch (config->data_bits) {
	case 5:
		data->uart_config.data_bits = UART_CFG_DATA_BITS_5;
		break;
	case 6:
		data->uart_config.data_bits = UART_CFG_DATA_BITS_6;
		break;
	case 7:
		data->uart_config.data_bits = UART_CFG_DATA_BITS_7;
		break;
	case 8:
		data->uart_config.data_bits = UART_CFG_DATA_BITS_8;
		break;
	default:
		return -ENOTSUP;
	}

	ret = hpm_uart_configure(dev, &data->uart_config);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	return 0;
}

static DEVICE_API(uart, hpm_uart_driver_api) = {
	.poll_in = hpm_uart_poll_in,
	.poll_out = hpm_uart_poll_out,
	.err_check = hpm_uart_err_check,
	.configure = hpm_uart_configure,
	.config_get = hpm_uart_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = hpm_uart_fifo_fill,
	.fifo_read = hpm_uart_fifo_read,
	.irq_tx_enable = hpm_uart_irq_tx_enable,
	.irq_tx_disable = hpm_uart_irq_tx_disable,
	.irq_tx_ready = hpm_uart_irq_tx_ready,
	.irq_rx_enable = hpm_uart_irq_rx_enable,
	.irq_rx_disable = hpm_uart_irq_rx_disable,
	.irq_tx_complete = hpm_uart_irq_tx_complete,
	.irq_rx_ready = hpm_uart_irq_rx_ready,
	.irq_err_enable = hpm_uart_irq_err_enable,
	.irq_err_disable = hpm_uart_irq_err_disable,
	.irq_is_pending = hpm_uart_irq_is_pending,
	.irq_update = hpm_uart_irq_update,
	.irq_callback_set = hpm_uart_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define HPM_UART_IRQ_CONFIG(n)                                                                     \
	static void hpm_uart_irq_config_##n(const struct device *dev)                              \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), hpm_uart_isr,               \
			    DEVICE_DT_INST_GET(n), 0);                                             \
		irq_enable(DT_INST_IRQN(n));                                                       \
	}

#define HPM_UART_IRQ_CONFIG_FIELD(n) .irq_config_func = hpm_uart_irq_config_##n,
#else
#define HPM_UART_IRQ_CONFIG(n)
#define HPM_UART_IRQ_CONFIG_FIELD(n)
#endif

#define HPM_UART_INIT(n)                                                                           \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	HPM_UART_IRQ_CONFIG(n)                                                                     \
	static struct hpm_uart_dev_data hpm_uart_data_##n;                                         \
	static const struct hpm_uart_dev_config hpm_uart_config_##n = {                            \
		.base = (UART_Type *)DT_INST_REG_ADDR(n),                                          \
		.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),                                \
		.clock_name = (clock_name_t)DT_INST_CLOCKS_CELL(n, name),                          \
		.clock_source = (clk_src_t)DT_INST_CLOCKS_CELL(n, source),                         \
		.clock_divider = DT_INST_CLOCKS_CELL(n, divider),                                  \
		.baud_rate = DT_INST_PROP_OR(n, current_speed, 115200),                            \
		.parity = DT_INST_ENUM_IDX_OR(n, parity, UART_CFG_PARITY_NONE),                    \
		.stop_bits = DT_INST_ENUM_IDX_OR(n, stop_bits, UART_CFG_STOP_BITS_1),              \
		.data_bits = DT_INST_PROP_OR(n, data_bits, 8),                                     \
		.flow_ctrl = DT_INST_PROP(n, hw_flow_control) ? UART_CFG_FLOW_CTRL_RTS_CTS         \
							      : UART_CFG_FLOW_CTRL_NONE,           \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
		HPM_UART_IRQ_CONFIG_FIELD(n)};                                                     \
	DEVICE_DT_INST_DEFINE(n, hpm_uart_init, NULL, &hpm_uart_data_##n, &hpm_uart_config_##n,    \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, &hpm_uart_driver_api);

DT_INST_FOREACH_STATUS_OKAY(HPM_UART_INIT)
