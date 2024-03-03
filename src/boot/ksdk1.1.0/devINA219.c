
	// Authored 2016-2018. Phillip Stanley-Marbell. Additional contributors,
	// 2018-onwards, see git log.

	// All rights reserved.

	// Redistribution and use in source and binary forms, with or without
	// modification, are permitted provided that the following conditions
	// are met:

	// *	Redistributions of source code must retain the above
	// 	copyright notice, this list of conditions and the following
	// 	disclaimer.

	// *	Redistributions in binary form must reproduce the above
	// 	copyright notice, this list of conditions and the following
	// 	disclaimer in the documentation and/or other materials
	// 	provided with the distribution.

	// *	Neither the name of the author nor the names of its
	// 	contributors may be used to endorse or promote products
	// 	derived from this software without specific prior written
	// 	permission.

	// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	// POSSIBILITY OF SUCH DAMAGE.

#include <stdlib.h>

/*
 *	config.h needs to come first
 */
#include "config.h"

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devINA219.h"


extern volatile WarpI2CDeviceState		deviceINA219State;
extern volatile uint32_t				gWarpI2cBaudRateKbps;
extern volatile uint32_t				gWarpI2cTimeoutMilliseconds;
extern volatile uint32_t				gWarpSupplySettlingDelayMilliseconds;

uint32_t 		ina219_currentMultiplier_uA;
int16_t 		ina219_powerMultiplier_uW;
uint16_t 		ina219_calValue;

void setCalibration();
void setCalibration_double_cal();

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
initINA219(const uint8_t i2cAddress, uint16_t operatingVoltageMillivolts)
{
	deviceINA219State.i2cAddress					= i2cAddress;
	deviceINA219State.operatingVoltageMillivolts	= operatingVoltageMillivolts;
	setCalibration_double_cal();

	return;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t payload)
{
	uint8_t			payloadByte[2], commandByte[1];
	i2c_status_t	status;

	switch (deviceRegister)
	{
		case 0x00: case 0x05:		//only these 2 registers can write
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
		{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	commandByte[0] = deviceRegister;
	// two bytes received
	payloadByte[0] = (payload>>8) & 0xFF; /* MSB first */;
	payloadByte[1] = payload & 0xFF; /* LSB */
	warpEnableI2Cpins();

	status = I2C_DRV_MasterSendDataBlocking(
				0 /* I2C instance */,
				&slave,
				commandByte,
				1,
				payloadByte,
				2,
				gWarpI2cTimeoutMilliseconds);
	
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

WarpStatus
configureSensorINA219(uint16_t payload_Config, uint16_t payload_Calibration) // Configuration register at 0x00 and Calibration register at 0x05.
{
	WarpStatus	i2cWriteStatus1, i2cWriteStatus2;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	i2cWriteStatus1 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_Configuration /* register address added to warp.h */,
												  payload_Config /* payload: Disable FIFO */
	);

	i2cWriteStatus2 = writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_Calibration /* register address added to warp.h */,
												  payload_Calibration /* payload */
	);

	return (i2cWriteStatus1 | i2cWriteStatus2);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;


	USED(numberOfBytes); 										//numberOfBytes = 2
	switch (deviceRegister)
	{
		case 0x00: case 0x01: case 0x02: case 0x03: // all five registers can read
		case 0x04: case 0x05: 
		{
			/* OK */
			break;
		}

		default:
		{
			return kWarpStatusBadDeviceCommand;
		}
	}

	i2c_device_t slave =
	{
					.address = deviceINA219State.i2cAddress,
					.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);
	
	cmdBuf[0] = deviceRegister;
	warpEnableI2Cpins();

	status = I2C_DRV_MasterReceiveDataBlocking(
					0 /* I2C peripheral instance */,
					&slave,
					cmdBuf,
					1,
					(uint8_t *)deviceINA219State.i2cBuffer,
					numberOfBytes,
					gWarpI2cTimeoutMilliseconds);

	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
printSensorDataINA219(bool hexModeFlag)
{
	uint16_t	readSensorRegisterValueLSB;
	uint16_t	readSensorRegisterValueMSB;
	int16_t		readSensorRegisterValueCombined; //uint16_t?
	WarpStatus	i2cReadStatus;


	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	/*
	 *	All INA219 registers 16-bit registers are actually two 8-bit bytes via the I2C interface
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.

	kWarpSensorOutputRegisterINA219Configuration 
	kWarpSensorOutputRegisterINA219Shunt
	kWarpSensorOutputRegisterINA219Bus
	kWarpSensorOutputRegisterINA219Power
	kWarpSensorOutputRegisterINA219Current
	kWarpSensorOutputRegisterINA219Calibration
	 
	*/

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Configuration, 2);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Shunt, 2);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Bus, 2);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}


	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Power, 2);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Current, 2);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}

	i2cReadStatus = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Calibration, 2);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);


	if (i2cReadStatus != kWarpStatusOK)
	{
		warpPrint(" ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			warpPrint(" 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			warpPrint(" %d,", readSensorRegisterValueCombined);
		}
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

uint8_t
appendSensorDataINA219(uint8_t* buf)
{
	uint8_t index = 0;
	uint16_t readSensorRegisterValueLSB;
	uint16_t readSensorRegisterValueMSB;
	int16_t readSensorRegisterValueCombined;
	WarpStatus i2cReadStatus;

	warpScaleSupplyVoltage(deviceINA219State.operatingVoltageMillivolts);

	/*
	 *	From the MMA8451Q datasheet:
	 *
	 *		"A random read access to the LSB registers is not possible.
	 *		Reading the MSB register and then the LSB register in sequence
	 *		ensures that both bytes (LSB and MSB) belong to the same data
	 *		sample, even if a new data sample arrives between reading the
	 *		MSB and the LSB byte."
	 *
	 *	We therefore do 2-byte read transactions, for each of the registers.
	 *	We could also improve things by doing a 6-byte read transaction.
	 */
	i2cReadStatus                   = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Shunt, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                  	= readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Bus, 2);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}

	i2cReadStatus                   = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Power, 2);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	return index;

	i2cReadStatus                   = readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Current, 2);
	readSensorRegisterValueMSB      = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB      = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB & 0xFF) << 8) | (readSensorRegisterValueLSB & 0xFF);

	/*
	 *	Sign extend the 14-bit value based on knowledge that upper 2 bit are 0:
	 */

	if (i2cReadStatus != kWarpStatusOK)
	{
		buf[index] = 0;
		index += 1;

		buf[index] = 0;
		index += 1;
	}
	else
	{
		/*
		 * MSB first
		 */
		buf[index] = (uint8_t)(readSensorRegisterValueCombined >> 8);
		index += 1;

		buf[index] = (uint8_t)(readSensorRegisterValueCombined);
		index += 1;
	}
	return index;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void
setCalibration()
{


    ina219_calValue = 58514;

    // Set multipliers to convert raw current/power values
    ina219_currentMultiplier_uA = 7;    // Current LSB = 5uA per bit 
    ina219_powerMultiplier_uW = 0.14 * 1000; // Power LSB = 0.1mW per bit

    // Set Calibration register to 'Cal' calculated above
    writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_Calibration, ina219_calValue);

    // Set Config register to take into account the settings above
    // uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
    //                   INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
    //                   INA219_CONFIG_SADCRES_12BIT_1S_532US |
    //                   INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    writeSensorRegisterINA219(kWarpSensorOutputRegisterINA219Configuration, (uint16_t) 0b0000000110011111);
}

void
setCalibration_double_cal()
{


    ina219_calValue = 58514;

    // Set multipliers to convert raw current/power values
    ina219_currentMultiplier_uA = 7;    // Current LSB = 5uA per bit 
    ina219_powerMultiplier_uW = 0.14 * 1000; // Power LSB = 0.1mW per bit

    // Set Calibration register to 'Cal' calculated above
    writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_Calibration, ina219_calValue);

    // Set Config register to take into account the settings above
    // uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
    //                   INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
    //                   INA219_CONFIG_SADCRES_12BIT_1S_532US |
    //                   INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

    writeSensorRegisterINA219(kWarpSensorOutputRegisterINA219Configuration, (uint16_t) 0b0000000110011111);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int16_t getBusVoltage_raw_INA219()
{
    uint16_t value;

    readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Bus, 2);
    value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) + deviceINA219State.i2cBuffer[1];

    // Shift to the right 3 to drop CNVR and OVF and multiply by LSB
    return (int16_t)((value >> 3) * 4);
}

/*!
 *  @brief  Gets the raw shunt voltage (16-bit signed integer, so +-32767)
 *  @return the raw shunt voltage reading
 */
int16_t getShuntVoltage_raw_INA219()
{
    uint16_t value;
    readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Shunt, 2 /* numberOfBytes */);
    value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) + deviceINA219State.i2cBuffer[1];

    return value;
}

/*!
 *  @brief  Gets the raw current value (16-bit signed integer, so +-32767)
 *  @return the raw current reading
 */
int16_t getCurrent_raw_INA219()
{
    uint16_t value;

    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available. avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_Calibration /* register address added to warp.h */,
												  ina219_calValue /* payload */
	);

    // Now we can safely read the CURRENT register!
    readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Current, 2);
    value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) + deviceINA219State.i2cBuffer[1];
    return value;
}

/*!
 *  @brief  Gets the raw power value (16-bit signed integer, so +-32767)
 *  @return raw power reading
 */
int16_t getPower_raw_INA219()
{
    uint16_t value;

    // Sometimes a sharp load will reset the INA219, which will
    // reset the cal register, meaning CURRENT and POWER will
    // not be available. avoid this by always setting a cal
    // value even if it's an unfortunate extra step
    writeSensorRegisterINA219(kWarpSensorConfigurationRegisterINA219_Calibration /* register address added to warp.h */,
												  ina219_calValue /* payload */
	);

    // Now we can safely read the POWER register!
    readSensorRegisterINA219(kWarpSensorOutputRegisterINA219Power, 2);
    value = ((uint16_t)deviceINA219State.i2cBuffer[0] << 8) + deviceINA219State.i2cBuffer[1];
    return value;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*!
 *  @brief  Gets the shunt voltage in uV (so +-327000uV)
 *  @return the shunt voltage converted to microvolts
 */
int32_t getShuntVoltage_uV_INA219()
{
    int32_t value;
    value = getShuntVoltage_raw_INA219();
    return value * 10; // LSB is 10uV
}

/*!
 *  @brief  Gets the bus voltage in millivolts
 *  @return the bus voltage converted to millivolts
 */
int32_t getBusVoltage_mV_INA219()
{
    int32_t value = getBusVoltage_raw_INA219();
    return value;
}

/*!
 *  @brief  Gets the current value in uA, taking into account the
 *          config settings and current LSB
 *  @return the current reading convereted to microamps
 */
int32_t getCurrent_uA_INA219()
{
    int32_t valueDec = getCurrent_raw_INA219();
    valueDec *= ina219_currentMultiplier_uA;
    return valueDec;
}


/*!
 *  @brief  Gets the power value in uW, taking into account the
 *          config settings and power LSB
 *  @return power reading converted to microwatts
 */
int32_t getPower_uW_INA219()
{
    int32_t valueDec = getPower_raw_INA219();
    valueDec *= ina219_powerMultiplier_uW;
    return valueDec;
}