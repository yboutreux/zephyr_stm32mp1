/*
 * Copyright (c) 2016 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for UART on ARM CMSDK APB UART.
 *
 * UART has two wires for RX and TX, and does not provide CTS or RTS.
 */

#include <kernel.h>
#include <arch/cpu.h>
#include <clock_control/arm_clock_control.h>
#include <misc/__assert.h>
#include <board.h>
#include <init.h>
#include <uart.h>
#include <linker/sections.h>

/* UART registers struct */
struct uart_cmsdk_apb {
	/* offset: 0x000 (r/w) data register    */
	volatile u32_t  data;
	/* offset: 0x004 (r/w) status register  */
	volatile u32_t  state;
	/* offset: 0x008 (r/w) control register */
	volatile u32_t  ctrl;
	union {
		/* offset: 0x00c (r/ ) interrupt status register */
		volatile u32_t  intstatus;
		/* offset: 0x00c ( /w) interrupt clear register  */
		volatile u32_t  intclear;
	};
	/* offset: 0x010 (r/w) baudrate divider register */
	volatile u32_t  bauddiv;
};

/* UART Bits */
/* CTRL Register */
#define UART_TX_EN	(1 << 0)
#define UART_RX_EN	(1 << 1)

/* STATE Register */
#define UART_TX_BF	(1 << 0)
#define UART_RX_BF	(1 << 1)

/* Device data structure */
struct uart_cmsdk_apb_dev_data {
	u32_t baud_rate;	/* Baud rate */
};

/* convenience defines */
#define DEV_CFG(dev) \
	((const struct uart_device_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct uart_cmsdk_apb_dev_data * const)(dev)->driver_data)
#define UART_STRUCT(dev) \
	((volatile struct uart_cmsdk_apb *)(DEV_CFG(dev))->base)

static const struct uart_driver_api uart_cmsdk_apb_driver_api;

/**
 * @brief Set the baud rate
 *
 * This routine set the given baud rate for the UART.
 *
 * @param dev UART device struct
 *
 * @return N/A
 */
static void baudrate_set(struct device *dev)
{
	volatile struct uart_cmsdk_apb *uart = UART_STRUCT(dev);
	const struct uart_device_config * const dev_cfg = DEV_CFG(dev);
	struct uart_cmsdk_apb_dev_data *const dev_data = DEV_DATA(dev);
	/*
	 * If baudrate and/or sys_clk_freq are 0 the configuration remains
	 * unchanged. It can be useful in case that Zephyr it is run via
	 * a bootloader that brings up the serial and sets the baudrate.
	 */
	if ((dev_data->baud_rate != 0) && (dev_cfg->sys_clk_freq != 0)) {
		/* calculate baud rate divisor */
		uart->bauddiv = (dev_cfg->sys_clk_freq / dev_data->baud_rate);
	}
}

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0
 */
static int uart_cmsdk_apb_init(struct device *dev)
{
	volatile struct uart_cmsdk_apb *uart = UART_STRUCT(dev);

#ifdef CONFIG_CLOCK_CONTROL
	/* Enable clock for subsystem */
	struct device *clk =
		device_get_binding(CONFIG_ARM_CLOCK_CONTROL_DEV_NAME);

	struct uart_cmsdk_apb_dev_data * const data = DEV_DATA(dev);
#endif /* CONFIG_CLOCK_CONTROL */

	/* Set baud rate */
	baudrate_set(dev);

	/* Enable receiver and transmitter */
	uart->ctrl = UART_RX_EN | UART_TX_EN;

	return 0;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */

static int uart_cmsdk_apb_poll_in(struct device *dev, unsigned char *c)
{
	volatile struct uart_cmsdk_apb *uart = UART_STRUCT(dev);

	/* If the receiver is not ready returns -1 */
	if (!(uart->state & UART_RX_BF)) {
		return -1;
	}

	/* got a character */
	*c = (unsigned char)uart->data;

	return 0;
}

/**
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * @param dev UART device struct
 * @param c Character to send
 *
 * @return Sent character
 */
static unsigned char uart_cmsdk_apb_poll_out(struct device *dev,
					     unsigned char c)
{
	volatile struct uart_cmsdk_apb *uart = UART_STRUCT(dev);

	/* Wait for transmitter to be ready */
	while (uart->state & UART_TX_BF) {
		; /* Wait */
	}

	/* Send a character */
	uart->data = (u32_t)c;
	return c;
}

static const struct uart_driver_api uart_cmsdk_apb_driver_api = {
	.poll_in = uart_cmsdk_apb_poll_in,
	.poll_out = uart_cmsdk_apb_poll_out,
};

#ifdef CONFIG_UART_CMSDK_APB_PORT0

static const struct uart_device_config uart_cmsdk_apb_dev_cfg_0 = {
	.base = (u8_t *)CMSDK_APB_UART0,
	.sys_clk_freq = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
};

static struct uart_cmsdk_apb_dev_data uart_cmsdk_apb_dev_data_0 = {
	.baud_rate = CONFIG_UART_CMSDK_APB_PORT0_BAUD_RATE,
};

DEVICE_AND_API_INIT(uart_cmsdk_apb_0,
		    CONFIG_UART_CMSDK_APB_PORT0_NAME,
		    &uart_cmsdk_apb_init,
		    &uart_cmsdk_apb_dev_data_0,
		    &uart_cmsdk_apb_dev_cfg_0, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_cmsdk_apb_driver_api);
#endif /* CONFIG_UART_CMSDK_APB_PORT0 */

#ifdef CONFIG_UART_CMSDK_APB_PORT1
static const struct uart_device_config uart_cmsdk_apb_dev_cfg_1 = {
	.base = (u8_t *)CMSDK_APB_UART1,
	.sys_clk_freq = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC,
};

static struct uart_cmsdk_apb_dev_data uart_cmsdk_apb_dev_data_1 = {
	.baud_rate = CONFIG_UART_CMSDK_APB_PORT1_BAUD_RATE,
};

DEVICE_AND_API_INIT(uart_cmsdk_apb_1,
		    CONFIG_UART_CMSDK_APB_PORT1_NAME,
		    &uart_cmsdk_apb_init,
		    &uart_cmsdk_apb_dev_data_1,
		    &uart_cmsdk_apb_dev_cfg_1, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_cmsdk_apb_driver_api);
#endif /* CONFIG_UART_CMSDK_APB_PORT1 */
