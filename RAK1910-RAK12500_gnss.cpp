/**
 * @file RAK1910-RAK12500_gnss.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief GNSS functions
 * @version 0.1
 * @date 2022-04-12
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "main.h"

#include "RAK12500_GNSS.h"

/** Instance for RAK1910 GNSS sensor */
// TinyGPSPlus my_rak1910_gnss;
/** Instance for RAK12500 GNSS sensor */
RAK12500_GNSS my_gnss;

// GNSS functions
#define NO_GNSS_INIT 0
#define GNSS_INIT_OK 1

// Fake GPS Enable (1) Disable (0)
#define FAKE_GPS 0

/** GNSS polling function */
bool poll_gnss(void);

/** Flag if location was found */
volatile bool last_read_ok = false;

/** Flag if GNSS is serial or I2C */
bool i2c_gnss = false;

/** The GPS module to use */
uint8_t g_gnss_init = 0;

/**
 * @brief Initialize GNSS module
 *
 * @return true if GNSS module was found
 * @return false if no GNSS module was found
 */
bool init_gnss(void)
{
	// Power on the GNSS module
	digitalWrite(WB_IO2, HIGH);

	// Give the module some time to power up
	delay(500);

	if (g_gnss_init == NO_GNSS_INIT)
	{
		if (found_sensors[GNSS_ID].found_sensor)
		{
			Wire.begin();
			if (!my_gnss.begin(Wire))
			{
				// MYLOG("GNSS", "Could not initialize RAK12500 on Wire");
				i2c_gnss = false;
			}
			else
			{
				i2c_gnss = true;
				// MYLOG("GNSS", "RAK12500 found on I2C");
				i2c_gnss = true;

				my_gnss.setI2COutput(COM_TYPE_UBX); // Set the I2C port to output UBX only (turn off NMEA noise)
				g_gnss_init = GNSS_INIT_OK;

				my_gnss.saveConfiguration(); // Save the current settings to flash and BBR

				my_gnss.setNavigationFrequency(1);

				MYLOG("GNSS", "Rate %d",my_gnss.getNavigationFrequency());
				return true;
			}
		}
		return false;
	}
	else
	{
		if (g_gnss_init == GNSS_INIT_OK)
		{
			my_gnss.begin(Wire);
		}
		return true;
	}
	return false;
}

/**
 * @brief Check GNSS module for position
 *
 * @return true Valid position found
 * @return false No valid position
 */
bool poll_gnss(void)
{
	last_read_ok = false;

	time_t time_out = millis();
	int64_t latitude = 0;
	int64_t longitude = 0;
	int32_t altitude = 0;
	int32_t accuracy = 0;

	time_t check_limit = g_lorawan_settings.send_repeat_time / 2;

	bool has_pos = false;
	bool has_alt = false;

	if (g_gnss_init == GNSS_INIT_OK)
	{
		if (my_gnss.getGnssFixOk())
		{
			byte fix_type = my_gnss.getFixType(); // Get the fix type
			// char fix_type_str[32] = {0};
			// if (fix_type == 0)
			// 	sprintf(fix_type_str, "No Fix");
			// else if (fix_type == 1)
			// 	sprintf(fix_type_str, "Dead reckoning");
			// else if (fix_type == 2)
			// 	sprintf(fix_type_str, "Fix type 2D");
			// else if (fix_type == 3)
			// 	sprintf(fix_type_str, "Fix type 3D");
			// else if (fix_type == 4)
			// 	sprintf(fix_type_str, "GNSS fix");
			// else if (fix_type == 5)
			// 	sprintf(fix_type_str, "Time fix");

			if ((fix_type >= 3) && (my_gnss.getSIV() >= 5)) /** Fix type 3D and at least 5 satellites */
															// if (fix_type >= 3) /** Fix type 3D */
			{
				last_read_ok = true;
				latitude = my_gnss.getLatitude();
				longitude = my_gnss.getLongitude();
				altitude = my_gnss.getAltitude();
				accuracy = my_gnss.getPositionDOP();

				// MYLOG("GNSS", "Fixtype: %d %s", my_gnss.getFixType(), fix_type_str);
				// MYLOG("GNSS", "Lat: %.4f Lon: %.4f", latitude / 10000000.0, longitude / 10000000.0);
				// MYLOG("GNSS", "Alt: %.2f", altitude / 1000.0);
				// MYLOG("GNSS", "Acy: %.2f ", accuracy / 100.0);

				if ((my_gnss.getTimeValid()) && (my_gnss.getDateValid()))
				{
					MYLOG("GNSS", "%d.%02d.%02d %d:%02d:%02d", my_gnss.getYear(), my_gnss.getMonth(), my_gnss.getDay(),
						  my_gnss.getHour(), my_gnss.getMinute(), my_gnss.getSecond());
				}
			}
		}
	}

	char disp_str[255];
	if (last_read_ok)
	{
		if ((latitude == 0) && (longitude == 0))
		{
			last_read_ok = false;
			return false;
		}

		// Enable power save mode after a fix
		my_gnss.powerSaveMode(true);

		switch (gnss_format)
		{
		case LPP_4_DIGIT:
			g_solution_data.addGNSS_4(LPP_CHANNEL_GPS, latitude, longitude, altitude);
			break;
		case LPP_6_DIGIT:
			g_solution_data.addGNSS_6(LPP_CHANNEL_GPS, latitude, longitude, altitude);
			break;
		case HELIUM_MAPPER:
			g_solution_data.addGNSS_H(latitude, longitude, altitude, accuracy, api.system.bat.get());
			break;
		}

		// if (found_sensors[OLED_ID].found_sensor)
		// {
		// 	sprintf(disp_str, "%.2f %.2f %.2f", latitude / 10000000.0, longitude / 10000000.0, altitude / 1000.0, accuracy / 100.0);
		// 	rak1921_add_line(disp_str);
		// }
		return true;
	}
	else
	{
		// No location found
#if FAKE_GPS > 0
		// MYLOG("GNSS", "Faking GPS");
		// 14.4213730, 121.0069140, 35.000
		latitude = 144213730;
		longitude = 1210069140;
		altitude = 35000;
		accuracy = 100;

		switch (gnss_format)
		{
		case LPP_4_DIGIT:
			g_solution_data.addGNSS_4(LPP_CHANNEL_GPS, latitude, longitude, altitude);
			break;
		case LPP_6_DIGIT:
			g_solution_data.addGNSS_6(LPP_CHANNEL_GPS, latitude, longitude, altitude);
			break;
		case HELIUM_MAPPER:
			g_solution_data.addGNSS_H(latitude, longitude, altitude, accuracy, api.system.bat.get());
			break;
		}
		last_read_ok = true;

		// if (found_sensors[OLED_ID].found_sensor)
		// {
		// 	sprintf(disp_str, "%.2f %.2f %.2f", latitude / 10000000.0, longitude / 10000000.0, altitude / 1000.0, accuracy / 100.0);
		// 	rak1921_add_line(disp_str);
		// }

		// Enable power save mode after a fix
		my_gnss.powerSaveMode(false);

		return true;
#endif
	}

	// if (found_sensors[OLED_ID].found_sensor)
	// {
	// 	sprintf(disp_str, "No valid location found");
	// 	rak1921_add_line(disp_str);
	// }
	// MYLOG("GNSS", "No valid location found");
	last_read_ok = false;
	return false;
}
