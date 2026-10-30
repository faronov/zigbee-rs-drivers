# Sensor catalogue

Target list of drivers, grouped by family. One crate can cover several
compatible parts. Status: `planned`, `mock-tested`, `hw: <board>`.

ZCL column: the cluster the main quantity maps to.

## A. Sensirion

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| sensirion-common | Word+CRC framing, I2C interface, general-call reset | I2C | - | mock-tested |
| sht3x | SHT30, SHT31, SHT35 (GXHT30 clone) | I2C | 0x0402, 0x0405 | mock-tested |
| sht4x | SHT40, SHT41, SHT43, SHT45 | I2C | 0x0402, 0x0405 | mock-tested |
| shtc3 | SHTC1, SHTC3 | I2C | 0x0402, 0x0405 | mock-tested |
| sts4x | STS40, STS45 | I2C | 0x0402 | mock-tested |
| scd4x | SCD40, SCD41, SCD43 | I2C | 0x040D | mock-tested |
| scd30 | SCD30 | I2C | 0x040D | mock-tested |
| stcc4 | STCC4 | I2C | 0x040D | mock-tested |
| sgp4x | SGP40, SGP41 (+ gas index algorithm) | I2C | VOC/NOx index | mock-tested |
| sgp30 | SGP30 | I2C | TVOC, eCO2 | mock-tested |
| sps30 | SPS30 | I2C | 0x042A | mock-tested |
| sen5x | SEN50, SEN54, SEN55 | I2C | 0x042A, 0x0402, 0x0405 | mock-tested |
| sen6x | SEN60, SEN62, SEN63C, SEN65, SEN66, SEN68, SEN69C | I2C | 0x042A, 0x040D | mock-tested |
| sfa3x | SFA30 | I2C | 0x042B | mock-tested |

## B. Pressure and environmental (Bosch, Infineon, TE, ST, Goertek, TDK)

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| bme280 | BME280, BMP280 | I2C/SPI | 0x0403, 0x0402, 0x0405 | planned |
| bme680 | BME680, BME688 (raw gas resistance) | I2C/SPI | 0x0403, 0x0402, 0x0405 | planned |
| bmp180 | BMP180, BMP085 | I2C | 0x0403 | planned |
| bmp3xx | BMP388, BMP390 | I2C/SPI | 0x0403 | planned |
| bmp5xx | BMP580, BMP581, BMP585 | I2C/SPI | 0x0403 | planned |
| dps310 | DPS310, DPS368 | I2C/SPI | 0x0403 | planned |
| spl06 | SPL06-001, SPL07-003 | I2C/SPI | 0x0403 | planned |
| ms5611 | MS5611, MS5607 | I2C/SPI | 0x0403 | planned |
| ms5637 | MS5637, MS5837 | I2C | 0x0403 | planned |
| lps22hb | LPS22HB | I2C/SPI | 0x0403 | planned |
| lps22hh | LPS22HH | I2C/SPI | 0x0403 | planned |
| lps2xdf | LPS22DF, LPS28DFW | I2C/SPI | 0x0403 | planned |
| lps25hb | LPS25HB | I2C/SPI | 0x0403 | planned |
| icp101xx | ICP-10100, ICP-10101, ICP-10110, ICP-10111 | I2C | 0x0403 | planned |
| hp206c | HP206C, HP203B | I2C | 0x0403 | planned |
| omron2smpb | 2SMPB-02E | I2C | 0x0403 | planned |
| xgzp68xxd | XGZP6847D, XGZP6897D | I2C | 0x0403 | planned |

## C. Temperature + humidity (other vendors)

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| si70xx | Si7006, Si7013, Si7020, Si7021, HTU21D, SHT20/21/25 | I2C | 0x0402, 0x0405 | planned |
| si705x | Si7050, Si7051, Si7053, Si7054, Si7055 | I2C | 0x0402 | planned |
| hdc1080 | HDC1010, HDC1080 | I2C | 0x0402, 0x0405 | planned |
| hdc2080 | HDC2010, HDC2021, HDC2022, HDC2080 | I2C | 0x0402, 0x0405 | planned |
| hdc302x | HDC3020, HDC3021, HDC3022 | I2C | 0x0402, 0x0405 | planned |
| aht | AHT10, AHT20, AHT21, AHT25, AHT30 | I2C | 0x0402, 0x0405 | planned |
| dht | DHT11, DHT22/AM2302 | GPIO | 0x0402, 0x0405 | planned |
| am2320 | AM2320, AM2315C | I2C | 0x0402, 0x0405 | planned |
| hs300x | HS3001, HS3002, HS3003, HS3004 | I2C | 0x0402, 0x0405 | planned |
| hs400x | HS4001, HS4002, HS4003, HS4004 | I2C | 0x0402, 0x0405 | planned |
| htu31d | HTU31D | I2C | 0x0402, 0x0405 | planned |
| hts221 | HTS221 | I2C/SPI | 0x0402, 0x0405 | planned |
| ens210 | ENS210 | I2C | 0x0402, 0x0405 | planned |
| cht8305 | CHT8305 | I2C | 0x0402, 0x0405 | planned |
| cht831x | CHT8310, CHT8315 | I2C | 0x0402, 0x0405 | planned |

## D. Temperature

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| tmp102 | TMP102, TMP112 | I2C | 0x0402 | mock-tested |
| tmp108 | TMP108 | I2C | 0x0402 | mock-tested |
| tmp114 | TMP114 | I2C | 0x0402 | mock-tested |
| tmp11x | TMP116, TMP117, TMP119 | I2C | 0x0402 | mock-tested |
| tmp1075 | TMP1075 | I2C | 0x0402 | mock-tested |
| lm75 | LM75A, LM75B, PCT2075, TMP75, NCT75 | I2C | 0x0402 | mock-tested |
| mcp9808 | MCP9808, JC-42.4 sensors | I2C | 0x0402 | mock-tested |
| max31875 | MAX31875 | I2C | 0x0402 | mock-tested |
| p3t1755 | P3T1755 | I2C | 0x0402 | mock-tested |
| tcn75a | TCN75A, MCP9800/01/02/03 | I2C | 0x0402 | mock-tested |
| stts22h | STTS22H | I2C | 0x0402 | mock-tested |
| stts751 | STTS751 | I2C | 0x0402 | mock-tested |
| adt7410 | ADT7410, ADT7420 | I2C | 0x0402 | mock-tested |
| mlx90614 | MLX90614, MLX90615 (IR) | SMBus | 0x0402 | mock-tested |
| onewire | 1-Wire bus (bit-banged GPIO) | GPIO | - | mock-tested |
| ds18b20 | DS18B20, DS18S20, DS1822, MAX31820 | 1-Wire | 0x0402 | mock-tested |
| max31855 | MAX31855 (thermocouple) | SPI | 0x0402 | mock-tested |
| max6675 | MAX6675 (thermocouple) | SPI | 0x0402 | mock-tested |
| max31865 | MAX31865 (PT100/PT1000) | SPI | 0x0402 | mock-tested |
| mcp9600 | MCP9600, MCP9601 (thermocouple) | I2C | 0x0402 | mock-tested |
| ntc | NTC thermistor via ADC (Beta, Steinhart-Hart) | ADC | 0x0402 | mock-tested |

## E. Light, colour, UV, proximity

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| bh1750 | BH1750FVI, GY-302 | I2C | 0x0400 | planned |
| bh1730 | BH1730FVC | I2C | 0x0400 | planned |
| opt3001 | OPT3001, OPT3004, OPT3006 | I2C | 0x0400 | planned |
| opt4001 | OPT4001, OPT4003 | I2C | 0x0400 | planned |
| veml7700 | VEML7700, VEML6030 | I2C | 0x0400 | planned |
| veml6031 | VEML6031, VEML6035 | I2C | 0x0400 | planned |
| tsl2561 | TSL2560, TSL2561 | I2C | 0x0400 | planned |
| tsl2591 | TSL2591 | I2C | 0x0400 | planned |
| tsl2540 | TSL2540, TSL25403 | I2C | 0x0400 | planned |
| ltr390 | LTR-390UV | I2C | 0x0400, UV index | planned |
| ltr303 | LTR-303ALS, LTR-329ALS | I2C | 0x0400 | planned |
| ltr55x | LTR-553ALS, LTR-559ALS (+ proximity) | I2C | 0x0400 | planned |
| ltrf216a | LTR-F216A | I2C | 0x0400 | planned |
| apds9306 | APDS-9306, APDS-9306-065 | I2C | 0x0400 | planned |
| apds9960 | APDS-9960 (ALS, colour, proximity) | I2C | 0x0400 | planned |
| max44009 | MAX44009, GY-49 | I2C | 0x0400 | planned |
| isl29035 | ISL29035 | I2C | 0x0400 | planned |
| tcs34725 | TCS34725 | I2C | 0x0400, colour | planned |
| as7341 | AS7341 (spectral) | I2C | 0x0400 | planned |
| vcnl4040 | VCNL4040 (ALS + proximity) | I2C | 0x0400 | planned |
| si1145 | Si1145, Si1146, Si1147 (UV index, ALS, IR) | I2C | 0x0400, UV index | planned |

## F. Air quality, CO2, particulate

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| ens160 | ENS160, ENS161 | I2C/SPI | TVOC, eCO2, AQI | planned |
| ccs811 | CCS811 | I2C | TVOC, eCO2 | planned |
| ags10 | AGS10 | I2C | TVOC | planned |
| iaqcore | iAQ-Core C/P | I2C | TVOC, eCO2 | planned |
| mhz19 | MH-Z19B, MH-Z19C, MH-Z19E | UART | 0x040D | planned |
| senseair | Senseair S8, Sunrise/Sunlight (Modbus) | UART | 0x040D | planned |
| cm1106 | Cubic CM1106, CM1106SL-NS | UART/I2C | 0x040D | planned |
| explorir | GSS ExplorIR-M, SprintIR, CozIR | UART | 0x040D | planned |
| pmsx003 | PMS5003, PMS7003, PMSA003, PMSA003I | UART/I2C | 0x042A | planned |
| hm330x | HM3301, HM3302 | I2C | 0x042A | planned |
| sds011 | SDS011, SDS021 | UART | 0x042A | planned |
| ze08 | ZE08-CH2O, ZE07-CH2O | UART | 0x042B | planned |

## G. Power, energy, battery

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| ina219 | INA219 | I2C | 0x0B04 | planned |
| ina226 | INA226, INA230, INA231 | I2C | 0x0B04 | planned |
| ina260 | INA260 | I2C | 0x0B04 | planned |
| ina228 | INA228, INA229 | I2C/SPI | 0x0B04, 0x0702 | planned |
| ina3221 | INA3221 | I2C | 0x0B04 | planned |
| pac194x | PAC1941, PAC1942, PAC1943, PAC1944 | I2C | 0x0B04 | planned |
| max17048 | MAX17043, MAX17044, MAX17048, MAX17049 | I2C | 0x0001 | planned |
| max17055 | MAX17055 | I2C | 0x0001 | planned |
| max1726x | MAX17260, MAX17261, MAX17262, MAX17263 | I2C | 0x0001 | planned |
| bq27441 | BQ27441, BQ27421, BQ27426 | I2C | 0x0001 | planned |
| lc709203f | LC709203F | I2C | 0x0001 | planned |
| bl0942 | BL0942 | UART/SPI | 0x0B04, 0x0702 | planned |
| bl0937 | BL0937, HLW8012 (pulse output) | GPIO pulse | 0x0B04, 0x0702 | planned |
| hlw8032 | HLW8032, CSE7759 | UART | 0x0B04, 0x0702 | planned |
| cse7766 | CSE7766 | UART | 0x0B04, 0x0702 | planned |
| pzem004t | PZEM-004T v3, PZEM-016, PZEM-017 (Modbus) | UART | 0x0B04, 0x0702 | planned |
| ade7953 | ADE7953 | I2C/SPI | 0x0B04, 0x0702 | planned |
| battery | Divider + discharge curves (CR2032, 2xAA, Li-ion, LiFePO4) | ADC | 0x0001 | planned |

## H. Presence, distance, contact, weight

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| ld2410 | HLK-LD2410, LD2410B, LD2410C (24 GHz radar) | UART | 0x0406 | planned |
| ld2450 | HLK-LD2450 (multi-target radar) | UART | 0x0406 | planned |
| s3km1110 | Iclegend S3KM1110 (24 GHz radar) | UART | 0x0406 | planned |
| xbr818 | Phosense XBR818 (10 GHz radar) | I2C | 0x0406 | planned |
| vl53l0x | VL53L0X (ToF) | I2C | distance | planned |
| vl53l1x | VL53L1X, VL53L4CD (ToF) | I2C | distance | planned |
| hcsr04 | HC-SR04, RCWL-1601 (ultrasonic) | GPIO | distance | planned |
| a01nyub | DFRobot A01NYUB, A02YYUW (ultrasonic) | UART | distance | planned |
| si7210 | Si7210 (hall) | I2C | contact | planned |
| tmag5273 | TMAG5273 (3D hall) | I2C | contact | planned |
| als31300 | ALS31300 (3D hall) | I2C | contact | planned |
| digital | PIR, reed, water-leak, hall switch, vibration switch (debounced GPIO) | GPIO | 0x0406, IAS zone | planned |
| hx711 | HX711 (load cell) | GPIO | mass | planned |
| fdc1004 | FDC1004 (capacitance) | I2C | 0x0408 | planned |
| soil | Capacitive soil probe via ADC (calibrated) | ADC | 0x0408 | planned |

## I. Motion: accelerometers, IMUs, magnetometers

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| adxl345 | ADXL343, ADXL345 | I2C/SPI | vibration, tilt | mock-tested |
| adxl362 | ADXL362 | SPI | vibration, tilt | mock-tested |
| adxl367 | ADXL367 | I2C/SPI | vibration, tilt | mock-tested |
| lis2dh | LIS2DH12, LIS3DH, SC7A20 | I2C/SPI | vibration, tilt | mock-tested |
| lis2dw12 | LIS2DW12 | I2C/SPI | vibration, tilt | mock-tested |
| lis2du12 | LIS2DU12 | I2C/SPI | vibration, tilt | mock-tested |
| bma400 | BMA400 | I2C/SPI | vibration, tilt | mock-tested |
| mc3419 | MC3419, MC3479 | I2C/SPI | vibration, tilt | mock-tested |
| kxtj3 | KXTJ3-1057 | I2C | vibration, tilt | mock-tested |
| da217 | DA217, DA218 | I2C | vibration, tilt | mock-tested |
| lsm6dso | LSM6DS3, LSM6DSL, LSM6DSO, LSM6DSOX | I2C/SPI | vibration, tilt | planned |
| mpu6050 | MPU-6050, MPU-6500, MPU-9250 (accel/gyro) | I2C | vibration, tilt | planned |
| icm426xx | ICM-42605, ICM-42688-P, ICM-42670-P | I2C/SPI | vibration, tilt | planned |
| bmi160 | BMI160 | I2C/SPI | vibration, tilt | planned |
| bmi270 | BMI270 | I2C/SPI | vibration, tilt | planned |
| qmi8658 | QMI8658A, QMI8658C | I2C/SPI | vibration, tilt | planned |
| lis3mdl | LIS3MDL | I2C/SPI | contact | planned |
| lis2mdl | LIS2MDL, IIS2MDC | I2C/SPI | contact | planned |
| mmc56x3 | MMC5603, MMC5633 | I2C | contact | planned |
| hmc5883l | HMC5883L | I2C | contact | planned |
| qmc5883l | QMC5883L, QMC5883P | I2C | contact | planned |
| ak09918 | AK09916, AK09918 | I2C | contact | planned |
| bmm150 | BMM150, BMM350 | I2C/SPI | contact | planned |
