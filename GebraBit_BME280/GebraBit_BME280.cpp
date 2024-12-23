/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Sepehr Azimi
 * ________________________________________________________________________________________________________
 */
#include "GebraBit_BME280.h"
#include <SPI.h>
#include <math.h>

// Define the Chip Select pin for the BME280 sensor
#define SPI_CS_Pin 10  // Change this if your CS pin is different

/*=========================================================================================================================================
 * @brief     Read data from a specific register.
 * @param     regAddr Register Address of BME280
 * @param     data    Pointer to Variable where register value is saved.
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t GB_BME280_Read_Reg_Data(uint8_t regAddr, uint8_t* data) {    
    uint8_t status = 1;  // Assume 1 for error, 0 for success
    uint8_t txBuf = regAddr | 0x80; // Read operation: set the 8th-bit to 1
    uint8_t rxBuf;

    // Select the BME280 by pulling the chip select pin low
    digitalWrite(SPI_CS_Pin, LOW);

    // Transmit the register address and read the data
    SPI.transfer(txBuf);
    rxBuf = SPI.transfer(0xFF);

    // Deselect the BME280 by pulling the chip select pin high
    digitalWrite(SPI_CS_Pin, HIGH);

    *data = rxBuf;
    status = 0;  // Indicate success
    return status;
}

/*=========================================================================================================================================
 * @brief     Read specific bits from a register.
 * @param     regAddr     Register Address of BME280.
 * @param     start_bit   Start Bit location (0 to 7).
 * @param     len         Number of bits to read (1 to 8).
 * @param     data        Pointer to Variable where register bits value is saved.
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_BME280_Read_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data) {
    uint8_t status = 1;
    uint8_t tempData = 0;

    if (len > 8 || start_bit > 7) {
        return 1;  // Error
    }

    if (GB_BME280_Read_Reg_Data(regAddr, &tempData) == 0) {
        uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); // Create mask to extract desired bits
        tempData &= mask; // Isolate the desired bits
        tempData >>= (start_bit - len + 1); // Align the bits to LSB
        *data = tempData;
        status = 0;  // Success
    } else {
        status = 1;  // Error
        *data = 0;
    }
    return status;
}

/*=========================================================================================================================================
 * @brief     Read multiple bytes starting from a specific register address.
 * @param     regAddr First Register Address of BME280.
 * @param     data    Pointer to buffer where data is saved.
 * @param     byteQuantity Number of bytes to read.
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_BME280_Burst_Read(uint8_t regAddr, uint8_t* data, uint16_t byteQuantity) {
    uint8_t status = 1;  // Assume 1 for error, 0 for success

    // Select the BME280 by pulling the chip select pin low
    digitalWrite(SPI_CS_Pin, LOW);

    // Transmit the register address with read bit
    SPI.transfer(regAddr | 0x80);

    // Read the data bytes
    for (uint16_t i = 0; i < byteQuantity; i++) {
        data[i] = SPI.transfer(0xFF);
    }

    // Deselect the BME280 by pulling the chip select pin high
    digitalWrite(SPI_CS_Pin, HIGH);

    status = 0;  // Indicate success
    return status;
}

/*=========================================================================================================================================
 * @brief     Write data to a specific register.
 * @param     regAddr Register Address of BME280.
 * @param     data    Value to be written to register.
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_BME280_Write_Reg_Data(uint8_t regAddr, uint8_t data) {
    uint8_t status = 1;  // Assume 1 for error, 0 for success

    // Select the BME280 by pulling the chip select pin low
    digitalWrite(SPI_CS_Pin, LOW);

    // Transmit the register address and data
    SPI.transfer(regAddr & 0x7F); // Write operation: clear the 8th-bit
    SPI.transfer(data);

    // Deselect the BME280 by pulling the chip select pin high
    digitalWrite(SPI_CS_Pin, HIGH);

    status = 0;  // Indicate success
    return status;
}

/*=========================================================================================================================================
 * @brief     Write specific bits to a register.
 * @param     regAddr     Register Address of BME280.
 * @param     start_bit   Start Bit location (0 to 7).
 * @param     len         Number of bits to write (1 to 8).
 * @param     data        Value to be written to register bits.
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_BME280_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data) {
    uint8_t status = 1;
    uint8_t tempData = 0;

    if (len > 8 || start_bit > 7) {
        return 1;  // Error
    }

    if (GB_BME280_Read_Reg_Data(regAddr, &tempData) == 0) {
        uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
        data <<= (start_bit - len + 1); // Shift data into correct position
        data &= mask; // Zero out non-important bits in data
        tempData &= ~mask; // Zero out important bits in existing byte
        tempData |= data; // Combine data with existing byte

        // Write the modified data back to the register
        status = GB_BME280_Write_Reg_Data(regAddr&0x7F, tempData);
    }
    return status;
}

/*=========================================================================================================================================
 * @brief     Reset BME280.
 * @param     BME280   BME280 Struct RESET  variable.
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BME280_Soft_Reset(GebraBit_BME280* BME280) {
    while (1) {
        GB_BME280_Write_Reg_Data(BME280_RESET, BME280_SOFT_RESET_CMD);
        delay(4);
        GB_BME280_Check_NVM_Data(BME280);
        if (BME280->NVM_DATA == IS_Ready) {
            BME280->RESET = DONE;
            break;
        }
    }
}

/*=========================================================================================================================================
 * @brief     Get Device ID.
 * @param     BME280     BME280 Struct DEVICE_ID variable.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Get_Device_ID(GebraBit_BME280* BME280) {
    GB_BME280_Read_Reg_Data(BME280_ID, &BME280->DEVICE_ID);
}

/*=========================================================================================================================================
 * @brief     Disable Humidity Sensor.
 * @param     BME280   BME280 Struct HUMIDITY  variable.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Turn_Humidity_OFF(GebraBit_BME280* BME280) {
    GB_BME280_Write_Reg_Bits(BME280_CTRL_HUM, START_MSB_BIT_AT_2, BIT_LENGTH_3, BME280_TURN_SENSOR_OFF);
    BME280->HUMIDITY = Disable;
}

/*=========================================================================================================================================
 * @brief     Set Humidity OverSampling.
 * @param     BME280   BME280 Struct HUMIDITY_OVERSAMPLING & HUMIDITY variable.
 * @param     hum_over   Values are according to BME280_Sensor_Oversampling Enum.
 * @return    Nothing
 ========================================================================================================================================*/  
void GB_BME280_Humidity_OverSampling(GebraBit_BME280* BME280, BME280_Sensor_Oversampling hum_over) {
    GB_BME280_Write_Reg_Bits(BME280_CTRL_HUM, START_MSB_BIT_AT_2, BIT_LENGTH_3, hum_over);
    BME280->HUMIDITY_OVERSAMPLING = hum_over;
    BME280->HUMIDITY = Enable;
}

/*=========================================================================================================================================
 * @brief     Disable Temperature Sensor.
 * @param     BME280   BME280 Struct TEMPERATURE variable.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Turn_Temperature_OFF(GebraBit_BME280* BME280) {
    GB_BME280_Write_Reg_Bits(BME280_CTRL_MEAS, START_MSB_BIT_AT_7, BIT_LENGTH_3, BME280_TURN_SENSOR_OFF);
    BME280->TEMPERATURE = Disable;
}

/*=========================================================================================================================================
 * @brief     Set Temperature OverSampling.
 * @param     BME280   BME280 Struct TEMPERATURE_OVERSAMPLING & TEMPERATURE variable.
 * @param     temp_over   Values are according to BME280_Sensor_Oversampling Enum.
 * @return    Nothing
 ========================================================================================================================================*/  
void GB_BME280_Temperature_OverSampling(GebraBit_BME280* BME280, BME280_Sensor_Oversampling temp_over) {
    GB_BME280_Write_Reg_Bits(BME280_CTRL_MEAS, START_MSB_BIT_AT_7, BIT_LENGTH_3, temp_over);
    BME280->TEMPERATURE_OVERSAMPLING = temp_over;
    BME280->TEMPERATURE = Enable;
}

/*=========================================================================================================================================
 * @brief     Disable Pressure Sensor.
 * @param     BME280   BME280 Struct PRESSURE variable.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Turn_Pressure_OFF(GebraBit_BME280* BME280) {
    GB_BME280_Write_Reg_Bits(BME280_CTRL_MEAS, START_MSB_BIT_AT_4, BIT_LENGTH_3, BME280_TURN_SENSOR_OFF);
    BME280->PRESSURE = Disable;
}

/*=========================================================================================================================================
 * @brief     Set Pressure OverSampling.
 * @param     BME280   BME280 Struct PRESSURE_OVERSAMPLING & PRESSURE variable.
 * @param     press_over   Values are according to BME280_Sensor_Oversampling Enum.
 * @return    Nothing
 ========================================================================================================================================*/  
void GB_BME280_Pressure_OverSampling(GebraBit_BME280* BME280, BME280_Sensor_Oversampling press_over) {
    GB_BME280_Write_Reg_Bits(BME280_CTRL_MEAS, START_MSB_BIT_AT_4, BIT_LENGTH_3, press_over);
    BME280->PRESSURE_OVERSAMPLING = press_over;
    BME280->PRESSURE = Enable;
}

/*=========================================================================================================================================
 * @brief     Set BME280 Power Mode.
 * @param     BME280   BME280 Struct POWER_MODE variable.
 * @param     pmode    Values are according to BME280_Power_Mode Enum.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Power_Mode(GebraBit_BME280* BME280, BME280_Power_Mode pmode) {
    GB_BME280_Write_Reg_Bits(BME280_CTRL_MEAS, START_MSB_BIT_AT_1, BIT_LENGTH_2, pmode);
    BME280->POWER_MODE = pmode;
}

/*=========================================================================================================================================
 * @brief     Set Inactive Duration in normal mode.
 * @param     BME280  BME280 struct INACTIVE_DURATION variable.
 * @param     dur     Values are according to BME280_Inactive_Duration Enum.
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BME280_Inactive_Duration(GebraBit_BME280* BME280, BME280_Inactive_Duration dur) {
    GB_BME280_Write_Reg_Bits(BME280_CONFIG, START_MSB_BIT_AT_7, BIT_LENGTH_3, dur);
    BME280->INACTIVE_DURATION = dur;
}

/*=========================================================================================================================================
 * @brief     Set IIR Filter Coefficient.
 * @param     BME280  BME280 Struct IIR_FILTER variable.
 * @param     filter     Values are according to BME280_IIR_Filter_Coefficient Enum.
 * @return    Nothing 
 ========================================================================================================================================*/        
void GB_BME280_IIR_Filter_Coefficient(GebraBit_BME280* BME280, BME280_IIR_Filter_Coefficient filter) {
    GB_BME280_Write_Reg_Bits(BME280_CONFIG, START_MSB_BIT_AT_4, BIT_LENGTH_3, filter);
    BME280->IIR_FILTER_TIME_CONATANT = filter;
}

/*=========================================================================================================================================
 * @brief     Converts value from Two's Complement to real value.
 * @param     value    Pointer to data to be converted.
 * @param     length   Length of value in bits.
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BME280_Twos_Complement_Converter(int32_t* value, uint8_t length) {
    if (*value & ((uint32_t)1 << (length - 1))) {
        *value -= (uint32_t)1 << length;
    }
}

/*=========================================================================================================================================
 * @brief     Calculate Calibration Coefficients.
 * @param     BME280   BME280 struct.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Calculate_Calibration_Coefficients(GebraBit_BME280* BME280) {
    GB_BME280_Burst_Read(BME280_TEMP_PRESS_CALIB00_CALIB25, BME280->PRESS_TEMP_CALIBRATION_DATA, PRESS_TEMP_CALIBRATION_DATA_BUFFER_SIZE);
    GB_BME280_Burst_Read(BME280_HUMIDITY_CALIB26_CALIB41, BME280->HUMIDITY_CALIBRATION_DATA, HUMIDITY_CALIBRATION_DATA_BUFFER_SIZE);
    
    BME280->dig_t1 = ((uint32_t)BME280->PRESS_TEMP_CALIBRATION_DATA[1] << 8) | ((uint32_t)BME280->PRESS_TEMP_CALIBRATION_DATA[0]);
    GB_BME280_Twos_Complement_Converter(&BME280->dig_t1, 16);
    BME280->dig_t2 = ((uint32_t)BME280->PRESS_TEMP_CALIBRATION_DATA[3] << 8) | ((uint32_t)BME280->PRESS_TEMP_CALIBRATION_DATA[2]);
    GB_BME280_Twos_Complement_Converter(&BME280->dig_t2, 16);    
    BME280->dig_t3 = ((uint32_t)BME280->PRESS_TEMP_CALIBRATION_DATA[5] << 8) | ((uint32_t)BME280->PRESS_TEMP_CALIBRATION_DATA[4]);
    GB_BME280_Twos_Complement_Converter(&BME280->dig_t3, 16);
    BME280->dig_p1 = (uint16_t)((BME280->PRESS_TEMP_CALIBRATION_DATA[7] << 8) | BME280->PRESS_TEMP_CALIBRATION_DATA[6]);
    BME280->dig_p2 = (int16_t)((BME280->PRESS_TEMP_CALIBRATION_DATA[9] << 8) | BME280->PRESS_TEMP_CALIBRATION_DATA[8]);
    BME280->dig_p3 = (int16_t)((BME280->PRESS_TEMP_CALIBRATION_DATA[11] << 8) | BME280->PRESS_TEMP_CALIBRATION_DATA[10]);
    BME280->dig_p4 = (int16_t)((BME280->PRESS_TEMP_CALIBRATION_DATA[13] << 8) | BME280->PRESS_TEMP_CALIBRATION_DATA[12]);
    BME280->dig_p5 = (int16_t)((BME280->PRESS_TEMP_CALIBRATION_DATA[15] << 8) | BME280->PRESS_TEMP_CALIBRATION_DATA[14]);
    BME280->dig_p6 = (int16_t)((BME280->PRESS_TEMP_CALIBRATION_DATA[17] << 8) | BME280->PRESS_TEMP_CALIBRATION_DATA[16]);
    BME280->dig_p7 = (int16_t)((BME280->PRESS_TEMP_CALIBRATION_DATA[19] << 8) | BME280->PRESS_TEMP_CALIBRATION_DATA[18]);
    BME280->dig_p8 = (int16_t)((BME280->PRESS_TEMP_CALIBRATION_DATA[21] << 8) | BME280->PRESS_TEMP_CALIBRATION_DATA[20]);
    BME280->dig_p9 = (int16_t)((BME280->PRESS_TEMP_CALIBRATION_DATA[23] << 8) | BME280->PRESS_TEMP_CALIBRATION_DATA[22]);
    BME280->dig_h1 = (int32_t)BME280->PRESS_TEMP_CALIBRATION_DATA[25];
    GB_BME280_Twos_Complement_Converter(&BME280->dig_h1, 8);
    BME280->dig_h2 = ((uint32_t)BME280->HUMIDITY_CALIBRATION_DATA[1] << 8) | ((uint32_t)BME280->HUMIDITY_CALIBRATION_DATA[0]);
    GB_BME280_Twos_Complement_Converter(&BME280->dig_h2, 16);
    BME280->dig_h3 = (int32_t)BME280->HUMIDITY_CALIBRATION_DATA[2];
    GB_BME280_Twos_Complement_Converter(&BME280->dig_h3, 8); 
    BME280->dig_h4 = ((uint32_t)BME280->HUMIDITY_CALIBRATION_DATA[3] << 4) | ((uint32_t)(BME280->HUMIDITY_CALIBRATION_DATA[4] & 0x0F));
    GB_BME280_Twos_Complement_Converter(&BME280->dig_h4, 12);
    BME280->dig_h5 = ((uint32_t)BME280->HUMIDITY_CALIBRATION_DATA[5] << 4) | ((uint32_t)(BME280->HUMIDITY_CALIBRATION_DATA[4] >> 4));
    GB_BME280_Twos_Complement_Converter(&BME280->dig_h5, 12);
    BME280->dig_h6 = (int32_t)BME280->HUMIDITY_CALIBRATION_DATA[6];
    GB_BME280_Twos_Complement_Converter(&BME280->dig_h6, 8);
}

/*=========================================================================================================================================
 * @brief     Initialize BME280.
 * @param     BME280     initialize BME280.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_initialize(GebraBit_BME280* BME280) {
    delay(3);
    GB_BME280_Soft_Reset(BME280);
    GB_BME280_Calculate_Calibration_Coefficients(BME280);
    GB_BME280_Get_Device_ID(BME280);
}

/*=========================================================================================================================================
 * @brief     Configure BME280.
 * @param     BME280  Configure BME280.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Configuration(GebraBit_BME280* BME280) {
    // Set to lowest oversampling rates for faster updates
    GB_BME280_Humidity_OverSampling(BME280, X1_OVERSAMPLING);
    GB_BME280_IIR_Filter_Coefficient(BME280, FILTER_OFF);  // Disable filter for fastest response
    GB_BME280_Inactive_Duration(BME280, INACTIVE_DURATION_5_mS);  // Use the shortest available standby time
    GB_BME280_Temperature_OverSampling(BME280, X1_OVERSAMPLING);
    GB_BME280_Pressure_OverSampling(BME280, X1_OVERSAMPLING);
    GB_BME280_Power_Mode(BME280, FORCED_MODE);  // Switch to Forced Mode
}



/*=========================================================================================================================================
 * @brief     Get Raw Data Of Temperature, Pressure, and Humidity from Register.
 * @param     BME280  store Raw Data Of Temperature in GebraBit_BME280 Struct REGISTER_RAW_TEMPERATURE, REGISTER_RAW_PRESSURE & REGISTER_RAW_HUMIDITY.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Get_Register_Raw_Pressure_Temperature_Humidity(GebraBit_BME280* BME280) {
    GB_BME280_Check_Conversion_Transferred_Register(BME280);
    if (BME280->CONVERSION_RESULT == IS_Ready) {
        GB_BME280_Burst_Read(BME280_DATA_ADDR, BME280->REGISTER_RAW_DATA_BUFFER, REGISTER_RAW_DATA_BYTE_QTY);
        BME280->REGISTER_RAW_PRESSURE = ((uint32_t)BME280->REGISTER_RAW_DATA_BUFFER[0] << 12) |
                                        ((uint32_t)BME280->REGISTER_RAW_DATA_BUFFER[1] << 4) |
                                        ((uint32_t)(BME280->REGISTER_RAW_DATA_BUFFER[2] & 0x0F));
        BME280->REGISTER_RAW_TEMPERATURE = ((uint32_t)BME280->REGISTER_RAW_DATA_BUFFER[3] << 12) |
                                           ((uint32_t)BME280->REGISTER_RAW_DATA_BUFFER[4] << 4) |
                                           ((uint32_t)(BME280->REGISTER_RAW_DATA_BUFFER[5] & 0x0F));
        BME280->REGISTER_RAW_HUMIDITY = ((uint32_t)BME280->REGISTER_RAW_DATA_BUFFER[6] << 8) |
                                        ((uint32_t)BME280->REGISTER_RAW_DATA_BUFFER[7]);
    }
} 

/*=========================================================================================================================================
 * @brief     Calculate BME280 Temperature according to calibration coefficients.
 * @param     BME280     BME280 Struct calibration coefficients variable.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Compensate_Temperature(GebraBit_BME280* BME280) {
    double var1, var2, temperature_min = -40, temperature_max = 85;

    var1 = ((double)BME280->REGISTER_RAW_TEMPERATURE) / 16384.0 - ((double)BME280->dig_t1) / 1024.0;
    var1 = var1 * ((double)BME280->dig_t2);
    var2 = (((double)BME280->REGISTER_RAW_TEMPERATURE) / 131072.0 - ((double)BME280->dig_t1) / 8192.0);
    var2 = (var2 * var2) * ((double)BME280->dig_t3);
    BME280->FINE_TEMP_RESOLUTIN = (int32_t)(var1 + var2);
    BME280->COMPENSATED_TEMPERATURE = (var1 + var2) / 5120.0;

    if (BME280->COMPENSATED_TEMPERATURE < temperature_min) {
        BME280->COMPENSATED_TEMPERATURE = temperature_min;
    } else if (BME280->COMPENSATED_TEMPERATURE > temperature_max) {
        BME280->COMPENSATED_TEMPERATURE = temperature_max;
    }
}

/*=========================================================================================================================================
 * @brief     Check if NVM DATA is ready.
 * @param     BME280   BME280 Struct NVM_DATA variable.
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BME280_Check_NVM_Data(GebraBit_BME280 * BME280)  
{
    uint8_t tempData;
    GB_BME280_Read_Reg_Bits(BME280_STATUS, START_MSB_BIT_AT_0, BIT_LENGTH_1, &tempData);
    BME280->NVM_DATA = static_cast<BME280_Preparation>(tempData);
}

/*=========================================================================================================================================
 * @brief     Check if Conversion result transferred to the register.
 * @param     BME280   BME280 Struct CONVERSION_RESULT variable.
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BME280_Check_Conversion_Transferred_Register(GebraBit_BME280 * BME280)
{
    uint8_t tempData;
    GB_BME280_Read_Reg_Bits(BME280_STATUS, START_MSB_BIT_AT_3, BIT_LENGTH_1, &tempData);
    BME280->CONVERSION_RESULT = static_cast<BME280_Preparation>(tempData);
}

/*=========================================================================================================================================
 * @brief     Calculate BME280 Temperature Compensated Pressure according to calibration coefficients.
 * @param     BME280     BME280 Struct calibration coefficients variable.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Compensate_Pressure(GebraBit_BME280* BME280) {
    double var1, var2, var3, pressure, pressure_min = 30000.0, pressure_max = 110000.0;

    var1 = ((double)BME280->FINE_TEMP_RESOLUTIN / 2.0) - 64000.0;
    var2 = var1 * var1 * ((double)BME280->dig_p6) / 32768.0;
    var2 = var2 + var1 * ((double)BME280->dig_p5) * 2.0;
    var2 = (var2 / 4.0) + (((double)BME280->dig_p4) * 65536.0);
    var3 = ((double)BME280->dig_p3) * var1 * var1 / 524288.0;
    var1 = (var3 + ((double)BME280->dig_p2) * var1) / 524288.0;
    var1 = (1.0 + var1 / 32768.0) * ((double)BME280->dig_p1);

    /* avoid exception caused by division by zero */
    if (var1 > 0.0) {
        pressure = 1048576.0 - (double)BME280->REGISTER_RAW_PRESSURE;
        pressure = (pressure - (var2 / 4096.0)) * 6250.0 / var1;
        var1 = ((double)BME280->dig_p9) * pressure * pressure / 2147483648.0;
        var2 = pressure * ((double)BME280->dig_p8) / 32768.0;
        pressure = pressure + (var1 + var2 + ((double)BME280->dig_p7)) / 16.0;

        if (pressure < pressure_min) {
            pressure = pressure_min;
        } else if (pressure > pressure_max) {
            pressure = pressure_max;
        }
    } else { /* Invalid case */
        pressure = pressure_min;
    }

    BME280->COMPENSATED_PRESSURE = pressure / 100.0;
}

/*=========================================================================================================================================
 * @brief     Calculate BME280 Humidity according to calibration coefficients.
 * @param     BME280     BME280 Struct calibration coefficients variable.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Compensate_Humidity(GebraBit_BME280* BME280) {
    double humidity, humidity_min = 0.0, humidity_max = 100.0;
    double var1, var2, var3, var4, var5, var6;

    var1 = ((double)BME280->FINE_TEMP_RESOLUTIN) - 76800.0;
    var2 = (((double)BME280->dig_h4) * 64.0 + (((double)BME280->dig_h5) / 16384.0) * var1);
    var3 = BME280->REGISTER_RAW_HUMIDITY - var2;
    var4 = ((double)BME280->dig_h2) / 65536.0;
    var5 = (1.0 + (((double)BME280->dig_h3) / 67108864.0) * var1);
    var6 = 1.0 + (((double)BME280->dig_h6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    humidity = var6 * (1.0 - ((double)BME280->dig_h1) * var6 / 524288.0);

    if (humidity > humidity_max) {
        humidity = humidity_max;
    } else if (humidity < humidity_min) {
        humidity = humidity_min;
    }

    BME280->COMPENSATED_HUMIDITY = humidity;
}

/*=========================================================================================================================================
 * @brief     Convert Pressure to Altitude.
 * @param     BME280   BME280 struct ALTITUDE Variable.
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_BME280_Altitude(GebraBit_BME280* BME280) {
    BME280->ALTITUDE = ((1 - pow((BME280->COMPENSATED_PRESSURE * 100) / SEA_LEVEL_PRESSURE, 1 / 5.257)) / 0.0000225577);
}

/*=========================================================================================================================================
 * @brief     Determine if Getting Data from Register.
 * @param     BME280    BME280 struct.
 * @return    Nothing
 ========================================================================================================================================*/
void GB_BME280_Get_Data(GebraBit_BME280* BME280) {
    GB_BME280_Power_Mode(BME280, FORCED_MODE);
    delay(10);
    GB_BME280_Get_Register_Raw_Pressure_Temperature_Humidity(BME280);
    GB_BME280_Compensate_Temperature(BME280);
    GB_BME280_Compensate_Pressure(BME280);
    GB_BME280_Compensate_Humidity(BME280);
    GB_BME280_Altitude(BME280);
}
