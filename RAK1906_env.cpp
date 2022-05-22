/**
 * @file RAK1906_env.cpp
 * @author Bernd Giesecke (bernd@giesecke.tk)
 * @brief BME680 sensor functions
 * @version 0.1
 * @date 2022-04-10
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "main.h"
#include "rak1906.h"

/** BME680 instance for Wire */
rak1906 bme;

/**
 * @brief Initialize the BME680 sensor
 *
 * @return true if sensor was found
 * @return false if sensor was not found
 */
bool init_rak1906(void)
{
	Wire.begin();

	if (!bme.init())
	{
		MYLOG("BME", "Could not find a valid BME680 sensor, check wiring!");
		return false;
	}

	// Set up oversampling and filter initialization
	bme.setOversampling(TemperatureSensor, Oversample8);
	bme.setOversampling(HumiditySensor, Oversample2);
	bme.setOversampling(PressureSensor, Oversample4);
	bme.setIIRFilter(IIR4);
	bme.setGas(320, 150); // 320*C for 150 ms

	return true;
}

/**
 * @brief Read environment data from BME680
 *     Data is added to Cayenne LPP payload as channels
 *     LPP_CHANNEL_HUMID_2, LPP_CHANNEL_TEMP_2,
 *     LPP_CHANNEL_PRESS_2 and LPP_CHANNEL_GAS_2
 *
 *
 * @return true if reading was successful
 * @return false if reading failed
 */
bool read_rak1906()
{
	// MYLOG("BME", "Reading BME680");
	if (bme.update())
	{
		g_solution_data.addRelativeHumidity(LPP_CHANNEL_HUMID_2, bme.humidity());
		g_solution_data.addTemperature(LPP_CHANNEL_TEMP_2, bme.temperature());
		g_solution_data.addBarometricPressure(LPP_CHANNEL_PRESS_2, bme.pressure());
		g_solution_data.addAnalogInput(LPP_CHANNEL_GAS_2, bme.gas());
	}
	else
	{
return false;
	}
	return true;
}

/**********************************************************************************************/
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BME680.h>

// /** BME680 instance for Wire */
// Adafruit_BME680 bme(&Wire);

// /**
//  * @brief Initialize the BME680 sensor
//  *
//  * @return true if sensor was found
//  * @return false if sensor was not found
//  */
// bool init_rak1906(void)
// {
// 	Wire.begin();

// 	if (!bme.begin(0x76))
// 	{
// 		MYLOG("BME", "Could not find a valid BME680 sensor, check wiring!");
// 		return false;
// 	}

// 	// Set up oversampling and filter initialization
// 	bme.setTemperatureOversampling(BME680_OS_8X);
// 	bme.setHumidityOversampling(BME680_OS_2X);
// 	bme.setPressureOversampling(BME680_OS_4X);
// 	bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
// 	bme.setGasHeater(320, 150); // 320*C for 150 ms

// 	return true;
// }

// /**
//  * @brief Read environment data from BME680
//  *     Data is added to Cayenne LPP payload as channels
//  *     LPP_CHANNEL_HUMID_2, LPP_CHANNEL_TEMP_2,
//  *     LPP_CHANNEL_PRESS_2 and LPP_CHANNEL_GAS_2
//  *
//  *
//  * @return true if reading was successful
//  * @return false if reading failed
//  */
// bool read_rak1906()
// {
// 	// MYLOG("BME", "Reading BME680");
// 	bme.beginReading();
// 	time_t wait_start = millis();
// 	bool read_success = false;
// 	while ((millis() - wait_start) < 5000)
// 	{
// 		if (bme.endReading())
// 		{
// 			read_success = true;
// 			break;
// 		}
// 	}

// 	if (!read_success)
// 	{
// 		// MYLOG("BME", "BME reading timeout");
// 		return false;
// 	}

// 	g_solution_data.addRelativeHumidity(LPP_CHANNEL_HUMID_2, bme.humidity);
// 	g_solution_data.addTemperature(LPP_CHANNEL_TEMP_2, bme.temperature);
// 	g_solution_data.addBarometricPressure(LPP_CHANNEL_PRESS_2, bme.pressure / 100);
// 	g_solution_data.addAnalogInput(LPP_CHANNEL_GAS_2, (float)(bme.gas_resistance) / 1000.0);
// 	return true;
// }
