/* syskernel.c */

/*
 * Copyright (c) 1997-2010, 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <tc_util.h>

#include "syskernel.h"

#include <string.h>
#include <fsl_rtc.h>

K_THREAD_STACK_DEFINE(thread_stack1, STACK_SIZE);
K_THREAD_STACK_DEFINE(thread_stack2, STACK_SIZE);
struct k_thread thread_data1;
struct k_thread thread_data2;

char Msg[256];

FILE *output_file;

const char sz_success[] = "SUCCESSFUL";
const char sz_partial[] = "PARTIAL";
const char sz_fail[] = "FAILED";

/* time necessary to read the time */
u32_t tm_off;

/**
 *
 * @brief Get the time ticks before test starts
 *
 * Routine does necessary preparations for the test to start
 *
 * @return N/A
 */
void begin_test(void)
{
	/*
	 * Invoke bench_test_start in order to be able to use
	 * tCheck static variable.
	 */
	bench_test_start();
}

/**
 *
 * @brief Checks number of tests and calculate average time
 *
 * @return 1 if success and 0 on failure
 *
 * @param i   Number of tests.
 * @param t   Time in ticks for the whole test.
 */
int check_result(int i, u32_t t)
{
	/*
	 * bench_test_end checks tCheck static variable.
	 * bench_test_start modifies it
	 */
	if (bench_test_end() != 0) {
		fprintf(output_file, sz_case_result_fmt, sz_fail);
		fprintf(output_file, sz_case_details_fmt,
				"timer tick happened. Results are inaccurate");
		fprintf(output_file, sz_case_end_fmt);
		return 0;
	}
	if (i != NUMBER_OF_LOOPS) {
		fprintf(output_file, sz_case_result_fmt, sz_fail);
		fprintf(output_file, sz_case_details_fmt, "loop counter = ");
		fprintf(output_file, "%i !!!", i);
		fprintf(output_file, sz_case_end_fmt);
		return 0;
	}
	fprintf(output_file, sz_case_result_fmt, sz_success);
	fprintf(output_file, sz_case_details_fmt,
			"Average time for 1 iteration: ");
	fprintf(output_file, sz_case_timing_fmt,
			SYS_CLOCK_HW_CYCLES_TO_NS_AVG(t, NUMBER_OF_LOOPS));

	fprintf(output_file, sz_case_end_fmt);
	return 1;
}


/**
 *
 * @brief Check for a key press
 *
 * @return 1 when a keyboard key is pressed, or 0 if no keyboard support
 */
int kbhit(void)
{
	return 0;
}


/**
 *
 * @brief Prepares the test output
 *
 * @param continuously   Run test till the user presses the key.
 *
 * @return N/A
 */

void init_output(int *continuously)
{
	ARG_UNUSED(continuously);

	/*
	 * send all printf and fprintf to console
	 */
	output_file = stdout;
}


/**
 *
 * @brief Close output for the test
 *
 * @return N/A
 */
void output_close(void)
{
}

/**
 *
 * @brief Perform all selected benchmarks
 *
 * @return N/A
 */
void main(void)
{
	int	    continuously = 0;
	int	    test_result;

	rtc_config_t rtc_cfg;

	RTC_GetDefaultConfig(&rtc_cfg);

	RTC_Init(RTC, &rtc_cfg);
	RTC->CR |= RTC_CR_OSCE_MASK;
	printk("RTC->CR %x TSR %x SR %x\n", RTC->CR, RTC->TSR, RTC->SR);
	RTC_StartTimer(RTC);

#if 0

	printk("SIM->SCGC6 %x\n", SIM->SCGC6);
	SIM->SCGC6 |= 0x20000000;
	printk("SIM->SCGC6 %x\n", SIM->SCGC6);

	printk("RTC->CR %x\n", RTC->CR);

	RTC_Reset(RTC);
	RTC_StartTimer(RTC);
#endif
	printk("RTC->CR %x TSR %x SR %x\n", RTC->CR, RTC->TSR, RTC->SR);

	init_output(&continuously);
	bench_test_init();

	do {
		fprintf(output_file, sz_module_title_fmt,
			"kernel API test");
		fprintf(output_file, sz_kernel_ver_fmt,
			sys_kernel_version_get());
		fprintf(output_file,
			"\n\nEach test below is repeated %d times;\n"
			"average time for one iteration is displayed.",
			NUMBER_OF_LOOPS);

		test_result = 0;

		test_result += sema_test();
		test_result += lifo_test();
		test_result += fifo_test();
		test_result += stack_test();

		if (test_result) {
			/* sema/lifo/fifo/stack account for 12 tests in total */
			if (test_result == 12) {
				fprintf(output_file, sz_module_result_fmt,
					sz_success);
			} else {
				fprintf(output_file, sz_module_result_fmt,
					sz_partial);
			}
		} else {
			fprintf(output_file, sz_module_result_fmt, sz_fail);
		}
		TC_PRINT_RUNID;

	} while (continuously && !kbhit());

	output_close();

	printk("RTC->CR %x TSR %x SR %x\n", RTC->CR, RTC->TSR, RTC->SR);

	for (int i = 0; i < 10; i++) {
		printf("counting %d sec\n", i);
		k_sleep(5*MSEC_PER_SEC);
	}
	printk("RTC->CR %x TSR %x SR %x\n", RTC->CR, RTC->TSR, RTC->SR);


	for (int i = 0; i < 10; i++) {
		printf("counting %d sec\n", i);
		k_sleep(5*MSEC_PER_SEC);
	}
	printk("RTC->CR %x TSR %x SR %x\n", RTC->CR, RTC->TSR, RTC->SR);

	for (int i = 0; i < 60; i++) {
		k_sleep(MSEC_PER_SEC);
		printf("counting %d sec\n", i);
	}
	printk("RTC->CR %x TSR %x SR %x\n", RTC->CR, RTC->TSR, RTC->SR);
}
