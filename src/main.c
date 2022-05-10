/**
 * \file
 *
 * \brief  This file contains the SAMD QTouch library sample user application.
 *
 * Copyright (c) 2014-2015 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * \page License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 */

/**
 * \mainpage SAMD20/D21 QTouch Example
 *
 * \section Purpose
 *
 * This example demonstrates how to use QTouch library on SAMD20/D21 devices.
 *
 * \section Requirements
 *
 * This example can be used on SAMD20/D21 xplained pro and .
 *
 * \section Description
 *
 * The program configures the necessary modules for using QTouch library. After
 * it started, users can turn on and off the LED by touching the button and slider,
 * change the color of RGB LED by touching wheel on QT1 Xplained Pro board.
 */

#include <asf.h>

// --------------------------------------------------------
// Macros
// --------------------------------------------------------

/**
 * \def GET_SENSOR_STATE(SENSOR_NUMBER)
 * \brief To get the sensor state that it is in detect or not
 * \param SENSOR_NUMBER for which the state to be detected
 * \return Returns either 0 or 1
 * If the bit value is 0, it is not in detect
 * If the bit value is 1, it is in detect
 * Alternatively, the individual sensor state can be directly accessed using
 * p_qm_measure_data->p_sensor_states[(SENSOR_NUMBER/8)] variable.
 */
 #define GET_SELFCAP_SENSOR_STATE(SENSOR_NUMBER) p_selfcap_measure_data->p_sensor_states[(SENSOR_NUMBER/8)] & (1 << (SENSOR_NUMBER % 8))


// --------------------------------------------------------
// Definitions
// --------------------------------------------------------
#if SAMD20
#define LED_8_PIN  PIN_PA22
#define LED_9_PIN  PIN_PA23
#elif SAMD21
#define LED_8_PIN  PIN_PB12
#define LED_9_PIN  PIN_PB13
#endif

#define TIME_PERIOD_1MSEC 33u

// --------------------------------------------------------
// Variables
// --------------------------------------------------------
volatile uint8_t PWM_Count;
volatile uint16_t touch_time_counter = 0u;
struct rtc_module rtc_instance;

// --------------------------------------------------------
// function  prototypes
// --------------------------------------------------------
void configure_port_pins(void);
void timer_init( void );
void rtc_overflow_callback(void);
void configure_rtc_callbacks(void);
void configure_rtc_count(void);


// --------------------------------------------------------
// function  definitions
// --------------------------------------------------------

//Configure the RTC timer overflow callback
void rtc_overflow_callback(void)
{
	/* Do something on RTC overflow here */
	if(touch_time_counter == touch_time.measurement_period_ms)
	{
		touch_time.time_to_measure_touch = 1;
		touch_time.current_time_ms = touch_time.current_time_ms +
		touch_time.measurement_period_ms;
		touch_time_counter = 0u;
	}
	else
	{
		touch_time_counter++;
	}

	// Update PWM counter
	if(PWM_Count < 20)
	{
		PWM_Count++;
	}
	else
	{
		PWM_Count = 0;
	}

}
// Configure the RTC timer callback
void configure_rtc_callbacks(void)
{
	rtc_count_register_callback(&rtc_instance, rtc_overflow_callback, RTC_COUNT_CALLBACK_OVERFLOW);
	rtc_count_enable_callback(&rtc_instance,RTC_COUNT_CALLBACK_OVERFLOW);
}

//Configure the RTC timer count after which interrupts comes
void configure_rtc_count(void)
{
	struct rtc_count_config config_rtc_count;
	rtc_count_get_config_defaults(&config_rtc_count);

	config_rtc_count.prescaler           = RTC_COUNT_PRESCALER_DIV_1;
	config_rtc_count.mode                = RTC_COUNT_MODE_16BIT;
	config_rtc_count.continuously_update = true;
	
	rtc_count_init(&rtc_instance,RTC,&config_rtc_count);
	rtc_count_enable(&rtc_instance);
}

//Initialize timer
void timer_init(void)
{
	configure_rtc_count();
	configure_rtc_callbacks();
	rtc_count_set_period(&rtc_instance,TIME_PERIOD_1MSEC);
}

//LED Port Pin Configuration
void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	//port_pin_set_config(LED_8_PIN, &config_port_pin);
	port_pin_set_config(LED0_PIN, &config_port_pin);
}


/***********************************************************************
 * MAIN PROGRAM STARTS HERE
 ***********************************************************************/
int main(void)
{
	uint8_t button1_state, temp=0;
	static int16_t delta_ch0=0;

	// hardware init
	system_init();
	system_interrupt_enable_global();
	delay_init();
	timer_init();

	// Initialize QTouch library and configure touch sensors.
	touch_sensors_init();

	// init ports
	configure_port_pins();
	//port_pin_set_output_level(LED_8_PIN, 1);
	port_pin_set_output_level(LED0_PIN, 1);

	/* Configure System Sleep mode to STANDBY. */
	//system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);

	PWM_Count = 0;
	while (1)
	{
		delta_ch0=0;
		
		// Goto STANDBY sleep mode, unless woken by timer or PTC interrupt.
		//system_sleep();

		// Start touch sensor measurement, if touch_time.time_to_measure_touch flag is set by timer.
		touch_sensors_measure();

		// Update touch status once measurement complete flag is set.
		if ((p_selfcap_measure_data->measurement_done_touch == 1u))
		{
			p_selfcap_measure_data->measurement_done_touch = 0u;
			
			delta_ch0 = p_selfcap_measure_data->p_channel_signals[0] - p_selfcap_measure_data->p_channel_references[0];
			
			if(delta_ch0)
				temp++;
			
			
			// check touch sensor states and update LED
			button1_state = GET_SELFCAP_SENSOR_STATE(0);
			if(button1_state)
			{
				//port_pin_set_output_level(LED_8_PIN, 0);  // on
				port_pin_set_output_level(LED0_PIN, 0);
			}
			else
			{
				//port_pin_set_output_level(LED_8_PIN, 1);  // off
				port_pin_set_output_level(LED0_PIN, 1);
			}
		}

	}
}

