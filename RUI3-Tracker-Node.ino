/**
 * @file WisBlock-Sensor-Node.ino
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief RUI3 based code for easy testing of WisBlock I2C modules
 * @version 0.1
 * @date 2022-04-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "main.h"
#include "udrv_timer.h"

/** Initialization results */
bool ret;

/** LoRaWAN packet */
WisCayenne g_solution_data(255);

/** Set the device name, max length is 10 characters */
char g_dev_name[64] = "RUI3 GNSS Tracker                                              ";

/** Device settings */
s_lorawan_settings g_lorawan_settings;
s_lorawan_settings check_settings;

/** OTAA Device EUI MSB */
uint8_t node_device_eui[8] = {0}; // ac1f09fff8683172
/** OTAA Application EUI MSB */
uint8_t node_app_eui[8] = {0}; // ac1f09fff8683172
/** OTAA Application Key MSB */
uint8_t node_app_key[16] = {0}; // efadff29c77b4829acf71e1a6e76f713

/** Counter for GNSS readings */
uint16_t check_gnss_counter = 0;
/** Max number of GNSS readings before giving up */
uint16_t check_gnss_max_try = 0;

/** Flag for GNSS readings active */
bool gnss_active = false;

/**
 * @brief Callback after packet was received
 *
 * @param data Structure with the received data
 */
void receiveCallback(SERVICE_LORA_RECEIVE_T *data)
{
	MYLOG("RX-CB", "RX, port %d, DR %d, RSSI %d, SNR %d", data->Port, data->RxDatarate, data->Rssi, data->Snr);
	for (int i = 0; i < data->BufferSize; i++)
	{
		Serial.printf("%02X", data->Buffer[i]);
	}
	Serial.print("\r\n");
}

/**
 * @brief Callback after TX is finished
 *
 * @param status TX status
 */
void sendCallback(int32_t status)
{
	gnss_active = false;
	MYLOG("TX-CB", "TX status %d", status);
	digitalWrite(LED_BLUE, LOW);
}

/**
 * @brief Callback after join request cycle
 *
 * @param status Join result
 */
void joinCallback(int32_t status)
{
	// MYLOG("JOIN-CB", "Join result %d", status);
	if (status != 0)
	{
		if (!(ret = api.lorawan.join()))
		{
			MYLOG("JOIN-CB", "Join fail!");
			// if (found_sensors[OLED_ID].found_sensor)
			// {
			// 	rak1921_add_line((char *)"Join NW failed");
			// }
		}
	}
	else
	{
		MYLOG("JOIN-CB", "DR  %s", api.lorawan.dr.set(g_lorawan_settings.data_rate) ? "Success" : "Fail");
		MYLOG("JOIN-CB", "ADR  %s", api.lorawan.adr.set(g_lorawan_settings.adr_enabled ? 1 : 0) ? "Success" : "Fail");
		// MYLOG("JOIN-CB", "LoRaWan OTAA - joined! \r\n");
		digitalWrite(LED_BLUE, LOW);

		// if (found_sensors[OLED_ID].found_sensor)
		// {
		// 	rak1921_add_line((char *)"Joined NW");
		// }
	}
}

/**
 * @brief GNSS location aqcuisition
 * Called every 2.5 seconds by timer 1
 * Gives up after 1/2 of send frequency
 * or when location was aquired
 *
 */
void gnss_handler(void *)
{
	digitalWrite(LED_GREEN, HIGH);
	if (poll_gnss())
	{
		// Power down the module
		digitalWrite(WB_IO2, LOW);
		delay(100);
		// MYLOG("GNSS", "Got location");
		udrv_timer_stop(TIMER_1);
		send_packet();
	}
	else
	{
		if (check_gnss_counter >= check_gnss_max_try)
		{
			// Power down the module
			digitalWrite(WB_IO2, LOW);
			delay(100);
			// MYLOG("GNSS", "Location timeout");
			udrv_timer_stop(TIMER_1);
			if (gnss_format != HELIUM_MAPPER)
			{
				send_packet();
			}
		}
	}
	check_gnss_counter++;
	digitalWrite(LED_GREEN, LOW);
}

/**
 * @brief Arduino setup, called once after reboot/power-up
 *
 */
void setup()
{
	pinMode(LED_GREEN, OUTPUT);
	digitalWrite(LED_GREEN, HIGH);
	pinMode(LED_BLUE, OUTPUT);
	digitalWrite(LED_BLUE, HIGH);

	pinMode(WB_IO2, OUTPUT);
	digitalWrite(WB_IO2, HIGH);

	// Use RAK_CUSTOM_MODE supresses AT command and default responses from RUI3
	// Serial.begin(115200, RAK_CUSTOM_MODE);
	// Use "normal" mode to have AT commands available
	Serial.begin(115200);

#ifdef _VARIANT_RAK4630_
	time_t serial_timeout = millis();
	// On nRF52840 the USB serial is not available immediately
	while (!Serial.available())
	{
		if ((millis() - serial_timeout) < 5000)
		{
			delay(100);
			digitalWrite(LED_GREEN, !digitalRead(LED_GREEN));
		}
		else
		{
			break;
		}
	}
#else
	// For RAK3172 just wait a little bit for the USB to be ready
	delay(5000);
#endif

	// Find WisBlock I2C modules
	find_modules();

	MYLOG("SETUP", "RAKwireless %s Node", g_dev_name);
	// MYLOG("SETUP", "------------------------------------------------------");

	/************************************************************************/
	/* Experimental                                                         */
	/* LoRaWAN credentials and settings are taken from structure            */
	/* s_lorawan_settings. This is used in Arduino BSP WisBlock API and not */
	/* fully implemented here. Once custom AT commands are available this   */
	/* can be improved or removed                                           */
	/************************************************************************/
		bool creds_ok = true;
		if (api.lorawan.appeui.get(node_app_eui, 8))
		{
			if (node_app_eui[0] == 0)
			{
				if (!api.lorawan.appeui.set(g_lorawan_settings.node_app_eui, 8))
				{
					MYLOG("SETUP", "LoRaWan OTAA - set app EUI failed!");
					return;
				}
			}
		}

		if (api.lorawan.appkey.get(node_app_key, 16))
		{
			if (node_app_key[0] == 0)
			{
				if (!api.lorawan.appkey.set(g_lorawan_settings.node_app_key, 16))
				{
					MYLOG("SETUP", "LoRaWan OTAA - set application key failed!");
					return;
				}
			}
		}

		if (api.lorawan.deui.get(node_device_eui, 8))
		{
			if (node_device_eui[0] == 0)
			{
				if (!api.lorawan.deui.set(g_lorawan_settings.node_device_eui, 8))
				{
					MYLOG("SETUP", "LoRaWan OTAA - set device EUI failed! \r\n");
					return;
				}
			}
		}

/*************************************
LoRaWAN band setting:
RAK_REGION_EU433	0
RAK_REGION_CN470	1
RAK_REGION_RU864	2
RAK_REGION_IN865	3
RAK_REGION_EU868	4
RAK_REGION_US915	5
RAK_REGION_AU915	6
RAK_REGION_KR920	7
RAK_REGION_AS923	8
RAK_REGION_AS923-2	9
RAK_REGION_AS923-3	10
RAK_REGION_AS923-4	11
*************************************/

// Set region
#if RUI_DEV == 1
	MYLOG("SETUP", "Set Class A %s", api.lorawan.deviceClass.set(0) ? "Success" : "Fail");
#else
	MYLOG("SETUP", "Set Class C %s", api.lorawan.deviceClass.set(2) ? "Success" : "Fail");
#endif
	// MYLOG("SETUP", "Setting band %d", g_lorawan_settings.lora_region);
	uint8_t curr_band = (uint8_t)api.lorawan.band.get();
	// MYLOG("SETUP", "Current region %d", curr_band);
	if (curr_band == g_lorawan_settings.lora_region)
	{
		// MYLOG("SETUP", "Band is already %d", curr_band);
	}
	else
	{
		MYLOG("SETUP", "Region %s", api.lorawan.band.set(g_lorawan_settings.lora_region) ? "Success" : "Fail");
	}

	MYLOG("SETUP", "TXP %s", api.lorawan.txp.set(g_lorawan_settings.tx_power) ? "Success" : "Fail");

	// Set subband (only US915, AU195 and CN470)
	if ((g_lorawan_settings.lora_region == RAK_REGION_US915) ||
		(g_lorawan_settings.lora_region == RAK_REGION_AU915) ||
		(g_lorawan_settings.lora_region == RAK_REGION_CN470))
	{
		uint16_t maskBuff = 0x0001 << (g_lorawan_settings.subband_channels - 1);
		MYLOG("SETUP", "Channel mask %s", api.lorawan.mask.set(&maskBuff) ? "Success" : "Fail");
		// maskBuff = 0x0000;
		// api.lorawan.mask.get(&maskBuff);
		// MYLOG("SETUP", "Channel mask is set to 0x%04X", maskBuff);
	}

	// Set the network join mode
	MYLOG("SETUP", "Join mode %s", api.lorawan.njm.set(g_lorawan_settings.otaa_enabled) ? "Success" : "Fail");

	// Set packet mode (confirmed/unconfirmed)
	if (g_lorawan_settings.confirmed_msg_enabled)
	{
		MYLOG("SETUP", "Confirmed packets  %s", api.lorawan.cfm.set(0) ? "Success" : "Fail");
	}
	else
	{
		MYLOG("SETUP", "Unconfirmed packets  %s", api.lorawan.cfm.set(1) ? "Success" : "Fail");
	}

	// Start the join process
	if (!(ret = api.lorawan.join()))
	{
		MYLOG("SETUP", "LoRaWan OTAA - join fail! \r\n");
		return;
	}
	digitalWrite(LED_GREEN, LOW);

	// Setup the callbacks for joined and send finished
	api.lorawan.registerRecvCallback(receiveCallback);
	api.lorawan.registerSendCallback(sendCallback);
	api.lorawan.registerJoinCallback(joinCallback);

	// Get saved sending frequency from flash
	get_at_setting(SEND_FREQ_OFFSET);

	// Create a unified timer in C language. This API is defined in udrv_timer.h. It will be replaced by api.system.timer.create() after story #1195 is done.
	udrv_timer_create(TIMER_0, sensor_handler, HTMR_PERIODIC);
	if (g_lorawan_settings.send_repeat_time != 0)
	{
		// Start a unified C timer in C language. This API is defined in udrv_timer.h. It will be replaced by api.system.timer.start() after story #1195 is done.
		udrv_timer_start(TIMER_0, g_lorawan_settings.send_repeat_time, NULL);
	}

	// Register the custom AT command to set the send frequency
	MYLOG("SETUP", "Custom AT command %s", init_frequency_at() ? "Success" : "Fail");

	// If a GNSS module was found, setup a timer for the GNSS aqcuisions
	if (found_sensors[GNSS_ID].found_sensor)
	{
		// MYLOG("SETUP", "Create timer for GNSS polling");
		// Create a unified timer in C language. This API is defined in udrv_timer.h. It will be replaced by api.system.timer.create() after story #1195 is done.
		udrv_timer_create(TIMER_1, gnss_handler, HTMR_PERIODIC); // HTMR_ONESHOT);
	}

	// Start Join
	api.lorawan.join();

	// Show found modules
	announce_modules();
	digitalWrite(LED_BLUE, LOW);
}

/**
 * @brief sensor_handler is a timer function called every
 * g_lorawan_settings.send_repeat_time milliseconds. Default is 120000. Can be
 * changed in main.h
 *
 */
void sensor_handler(void *)
{
	// MYLOG("UPLINK", "Start");
	digitalWrite(LED_BLUE, HIGH);

	// Reset trigger time
	last_trigger = millis();

	// Check if the node has joined the network
	if (!api.lorawan.njs.get())
	{
		MYLOG("UPLINK", "Skip sending");
		return;
	}

	// Just for debug, show if the call is because of a motion detection
	if (motion_detected)
	{
		MYLOG("UPLINK", "ACC IRQ");
		motion_detected = false;
		clear_int_rak1904();
		if (gnss_active)
		{
			// digitalWrite(LED_BLUE, LOW);
			// GNSS is already active, do nothing
			return;
		}
	}

	// Clear payload
	g_solution_data.reset();

	// Helium Mapper ignores sensor and sends only location data
	if (gnss_format != HELIUM_MAPPER)
	{
		// Read sensor data
		get_sensor_values();

		// Add battery voltage
		g_solution_data.addVoltage(LPP_CHANNEL_BATT, api.system.bat.get());
	}

	// If it is a GNSS location tracker, start the timer to aquire the location
	if ((found_sensors[GNSS_ID].found_sensor) && !gnss_active)
	{
		// Set flag for GNSS active to avoid retrigger */
		gnss_active = true;
		// Startup GNSS module
		init_gnss();
		// Start the timer
		udrv_timer_start(TIMER_1, 2500, NULL);
		check_gnss_counter = 0;
		// Max location aquisition time is half of send frequency
		check_gnss_max_try = g_lorawan_settings.send_repeat_time / 2 / 2500;
	}
	else if (gnss_active)
	{
		return;
	}
	else
	{
		// No GNSS module, just send the packet with the sensor data
		send_packet();
	}
}

/**
 * @brief This example is complete timer
 * driven. The loop() does nothing than
 * sleep.
 *
 */
void loop()
{
	api.system.sleep.all();
}

/**
 * @brief Send the data packet that was prepared in
 * Cayenne LPP format by the different sensor and location
 * aqcuision functions
 *
 */
void send_packet(void)
{
	MYLOG("UPLINK", "Send %d", g_solution_data.getSize());

	// Send the packet
	if (api.lorawan.send(g_solution_data.getSize(), g_solution_data.getBuffer(), 2, g_lorawan_settings.confirmed_msg_enabled, 1))
	{
		MYLOG("UPLINK", "Enqueued");
	}
	else
	{
		MYLOG("UPLINK", "Send failed");
	}
}
