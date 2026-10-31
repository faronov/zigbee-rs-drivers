# Sensor and display catalogue

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
| bme280 | BME280, BMP280 | I2C/SPI | 0x0403, 0x0402, 0x0405 | mock-tested |
| bme680 | BME680, BME688 (raw gas resistance) | I2C/SPI | 0x0403, 0x0402, 0x0405 | mock-tested |
| bmp180 | BMP180, BMP085 | I2C | 0x0403 | mock-tested |
| bmp3xx | BMP388, BMP390 | I2C/SPI | 0x0403 | mock-tested |
| bmp5xx | BMP580, BMP581, BMP585 | I2C/SPI | 0x0403 | mock-tested |
| dps310 | DPS310, DPS368 | I2C/SPI | 0x0403 | mock-tested |
| spl06 | SPL06-001, SPL07-003 | I2C/SPI | 0x0403 | mock-tested |
| ms5611 | MS5611, MS5607 | I2C/SPI | 0x0403 | mock-tested |
| ms5637 | MS5637, MS5837 | I2C | 0x0403 | mock-tested |
| lps22hb | LPS22HB | I2C/SPI | 0x0403 | mock-tested |
| lps22hh | LPS22HH | I2C/SPI | 0x0403 | mock-tested |
| lps2xdf | LPS22DF, LPS28DFW | I2C/SPI | 0x0403 | mock-tested |
| lps25hb | LPS25HB | I2C/SPI | 0x0403 | mock-tested |
| icp101xx | ICP-10100, ICP-10101, ICP-10110, ICP-10111 | I2C | 0x0403 | mock-tested |
| hp206c | HP206C, HP203B | I2C | 0x0403 | mock-tested |
| omron2smpb | 2SMPB-02E | I2C | 0x0403 | mock-tested |
| xgzp68xxd | XGZP6847D, XGZP6897D | I2C | 0x0403 | mock-tested |

## C. Temperature + humidity (other vendors)

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| si70xx | Si7006, Si7013, Si7020, Si7021, HTU21D, SHT20/21/25 | I2C | 0x0402, 0x0405 | mock-tested |
| si705x | Si7050, Si7051, Si7053, Si7054, Si7055 | I2C | 0x0402 | mock-tested |
| hdc1080 | HDC1010, HDC1080 | I2C | 0x0402, 0x0405 | mock-tested |
| hdc2080 | HDC2010, HDC2021, HDC2022, HDC2080 | I2C | 0x0402, 0x0405 | mock-tested |
| hdc302x | HDC3020, HDC3021, HDC3022 | I2C | 0x0402, 0x0405 | mock-tested |
| aht | AHT10, AHT20, AHT21, AHT25, AHT30 | I2C | 0x0402, 0x0405 | mock-tested |
| dht | DHT11, DHT22/AM2302 | GPIO | 0x0402, 0x0405 | mock-tested |
| am2320 | AM2320, AM2315C | I2C | 0x0402, 0x0405 | mock-tested |
| hs300x | HS3001, HS3002, HS3003, HS3004 | I2C | 0x0402, 0x0405 | mock-tested |
| hs400x | HS4001, HS4002, HS4003, HS4004 | I2C | 0x0402, 0x0405 | mock-tested |
| htu31d | HTU31D | I2C | 0x0402, 0x0405 | mock-tested |
| hts221 | HTS221 | I2C/SPI | 0x0402, 0x0405 | mock-tested |
| ens210 | ENS210 | I2C | 0x0402, 0x0405 | mock-tested |
| cht8305 | CHT8305 | I2C | 0x0402, 0x0405 | mock-tested |
| cht831x | CHT8310, CHT8315 | I2C | 0x0402, 0x0405 | mock-tested |

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
| bh1750 | BH1750FVI, GY-302 | I2C | 0x0400 | mock-tested |
| bh1730 | BH1730FVC | I2C | 0x0400 | mock-tested |
| opt3001 | OPT3001, OPT3004, OPT3006 | I2C | 0x0400 | mock-tested |
| opt4001 | OPT4001, OPT4003 | I2C | 0x0400 | mock-tested |
| veml7700 | VEML7700, VEML6030 | I2C | 0x0400 | mock-tested |
| veml6031 | VEML6031, VEML6035 | I2C | 0x0400 | mock-tested |
| tsl2561 | TSL2560, TSL2561 | I2C | 0x0400 | mock-tested |
| tsl2591 | TSL2591 | I2C | 0x0400 | mock-tested |
| tsl2540 | TSL2540, TSL25403 | I2C | 0x0400 | mock-tested |
| ltr390 | LTR-390UV | I2C | 0x0400, UV index | mock-tested |
| ltr303 | LTR-303ALS, LTR-329ALS | I2C | 0x0400 | mock-tested |
| ltr55x | LTR-553ALS, LTR-559ALS (+ proximity) | I2C | 0x0400 | mock-tested |
| ltrf216a | LTR-F216A | I2C | 0x0400 | mock-tested |
| apds9306 | APDS-9306, APDS-9306-065 | I2C | 0x0400 | mock-tested |
| apds9960 | APDS-9960 (ALS, colour, proximity) | I2C | 0x0400 | mock-tested |
| max44009 | MAX44009, GY-49 | I2C | 0x0400 | mock-tested |
| isl29035 | ISL29035 | I2C | 0x0400 | mock-tested |
| tcs34725 | TCS34725 | I2C | 0x0400, colour | mock-tested |
| as7341 | AS7341 (spectral) | I2C | 0x0400 | mock-tested |
| vcnl4040 | VCNL4040 (ALS + proximity) | I2C | 0x0400 | mock-tested |
| si1145 | Si1145, Si1146, Si1147 (UV index, ALS, IR) | I2C | 0x0400, UV index | mock-tested |

## F. Air quality, CO2, particulate

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| ens160 | ENS160, ENS161 | I2C/SPI | TVOC, eCO2, AQI | mock-tested |
| ccs811 | CCS811 | I2C | TVOC, eCO2 | mock-tested |
| ags10 | AGS10 | I2C | TVOC | mock-tested |
| iaqcore | iAQ-Core C/P | I2C | TVOC, eCO2 | mock-tested |
| mhz19 | MH-Z19B, MH-Z19C, MH-Z19E | UART | 0x040D | mock-tested |
| senseair | Senseair S8, Sunrise/Sunlight (Modbus) | UART | 0x040D | mock-tested |
| cm1106 | Cubic CM1106, CM1106SL-NS | UART/I2C | 0x040D | mock-tested |
| explorir | GSS ExplorIR-M, SprintIR, CozIR | UART | 0x040D | mock-tested |
| pmsx003 | PMS5003, PMS7003, PMSA003, PMSA003I | UART/I2C | 0x042A | mock-tested |
| hm330x | HM3301, HM3302 | I2C | 0x042A | mock-tested |
| sds011 | SDS011, SDS021 | UART | 0x042A | mock-tested |
| ze08 | ZE08-CH2O, ZE07-CH2O | UART | 0x042B | mock-tested |

## G. Power, energy, battery

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| ina219 | INA219 | I2C | 0x0B04 | mock-tested |
| ina226 | INA226, INA230, INA231 | I2C | 0x0B04 | mock-tested |
| ina260 | INA260 | I2C | 0x0B04 | mock-tested |
| ina228 | INA228, INA229 | I2C/SPI | 0x0B04, 0x0702 | mock-tested |
| ina3221 | INA3221 | I2C | 0x0B04 | mock-tested |
| pac194x | PAC1941, PAC1942, PAC1943, PAC1944 | I2C | 0x0B04 | mock-tested |
| max17048 | MAX17043, MAX17044, MAX17048, MAX17049 | I2C | 0x0001 | mock-tested |
| max17055 | MAX17055 | I2C | 0x0001 | mock-tested |
| max1726x | MAX17260, MAX17261, MAX17262, MAX17263 | I2C | 0x0001 | mock-tested |
| bq27441 | BQ27441, BQ27421, BQ27426 | I2C | 0x0001 | mock-tested |
| lc709203f | LC709203F | I2C | 0x0001 | mock-tested |
| bl0942 | BL0942 | UART/SPI | 0x0B04, 0x0702 | mock-tested |
| bl0937 | BL0937, HLW8012 (pulse output) | GPIO pulse | 0x0B04, 0x0702 | mock-tested |
| hlw8032 | HLW8032, CSE7759 | UART | 0x0B04, 0x0702 | mock-tested |
| cse7766 | CSE7766 | UART | 0x0B04, 0x0702 | mock-tested |
| pzem004t | PZEM-004T v3, PZEM-016, PZEM-017 (Modbus) | UART | 0x0B04, 0x0702 | mock-tested |
| ade7953 | ADE7953 | I2C/SPI | 0x0B04, 0x0702 | mock-tested |
| battery | Divider + discharge curves (CR2032, 2xAA, Li-ion, LiFePO4) | ADC | 0x0001 | mock-tested |

## H. Presence, distance, contact, weight

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| ld2410 | HLK-LD2410, LD2410B, LD2410C (24 GHz radar) | UART | 0x0406 | mock-tested |
| ld2450 | HLK-LD2450 (multi-target radar) | UART | 0x0406 | mock-tested |
| s3km1110 | Iclegend S3KM1110 (24 GHz radar) | UART | 0x0406 | mock-tested |
| xbr818 | Phosense XBR818 (10 GHz radar) | I2C | 0x0406 | mock-tested |
| vl53l0x | VL53L0X (ToF) | I2C | distance | mock-tested |
| vl53l1x | VL53L1X, VL53L4CD (ToF) | I2C | distance | mock-tested |
| hcsr04 | HC-SR04, RCWL-1601 (ultrasonic) | GPIO | distance | mock-tested |
| a01nyub | DFRobot A01NYUB, A02YYUW (ultrasonic) | UART | distance | mock-tested |
| si7210 | Si7210 (hall) | I2C | contact | mock-tested |
| tmag5273 | TMAG5273 (3D hall) | I2C | contact | mock-tested |
| als31300 | ALS31300 (3D hall) | I2C | contact | mock-tested |
| digital | PIR, reed, water-leak, hall switch, vibration switch (debounced GPIO) | GPIO | 0x0406, IAS zone | mock-tested |
| hx711 | HX711 (load cell) | GPIO | mass | mock-tested |
| fdc1004 | FDC1004 (capacitance) | I2C | 0x0408 | mock-tested |
| soil | Capacitive soil probe via ADC (calibrated) | ADC | 0x0408 | mock-tested |

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
| lsm6dso | LSM6DS3, LSM6DSL, LSM6DSO, LSM6DSOX | I2C/SPI | vibration, tilt | mock-tested |
| mpu6050 | MPU-6050, MPU-6500, MPU-9250 (accel/gyro) | I2C | vibration, tilt | mock-tested |
| icm426xx | ICM-42605, ICM-42688-P, ICM-42670-P | I2C/SPI | vibration, tilt | mock-tested |
| bmi160 | BMI160 | I2C/SPI | vibration, tilt | mock-tested |
| bmi270 | BMI270 | I2C/SPI | vibration, tilt | mock-tested |
| qmi8658 | QMI8658A, QMI8658C | I2C/SPI | vibration, tilt | mock-tested |
| lis3mdl | LIS3MDL | I2C/SPI | contact | mock-tested |
| lis2mdl | LIS2MDL, IIS2MDC | I2C/SPI | contact | mock-tested |
| mmc56x3 | MMC5603, MMC5633 | I2C | contact | mock-tested |
| hmc5883l | HMC5883L | I2C | contact | mock-tested |
| qmc5883l | QMC5883L, QMC5883P | I2C | contact | mock-tested |
| ak09918 | AK09916, AK09918 | I2C | contact | mock-tested |
| bmm150 | BMM150 | I2C/SPI | contact | mock-tested |
| bmm350 | BMM350 | I2C | contact | mock-tested |

## J. Displays

Display drivers depend on the shared [`zigbee-display`](../drivers/display)
crate (traits, SPI/I2C interface, embedded-graphics frame buffers), not on
`zigbee-sensor` alone, so sensor-only builds do not pull in
embedded-graphics. ZCL column: none (displays are output only). RAM is the
caller-provided frame buffer.

| Crate | Parts | Bus | ZCL | Status |
|---|---|---|---|---|
| display | shared core: `Display` trait, `SpiInterface`/`I2cInterface`, `MonoFrame`/`TriFrame`/`PageFrame`/`Rgb565Frame` | SPI/I2C | — | mock-tested |
| sharp-memlcd | Sharp LS013B7DH03 128x128 (SiLabs WSTK BRD4001A), LS013B7DH05 144x168, LS011B7DH03 160x68, LS027B7DH01 400x240 (RAM 2 048 / 3 024 / 1 360 / 12 000 B) | SPI (CS active high) | — | mock-tested, not run on hardware |
| ssd1306 | SSD1306 128x64 / 128x32, SH1106 128x64 (RAM 1 024 / 512 B) | I2C/SPI | — | mock-tested, not run on hardware |
| ssd1680 | SSD1680 2.13" 122x250 and 2.9" 128x296, SSD1681 1.54" 200x200, SSD1675 2.13"; B/W and B/W/R (RAM 4 000–5 000 B, ×2 tricolour) | SPI + BUSY | — | mock-tested, not run on hardware |
| uc8151 | UC8151 2.9" 128x296 / 2.13" 104x212 B/W, IL0373 2.9"/2.13" B/W/R (RAM 4 736 / 2 756 B, ×2 tricolour) | SPI + BUSY_N | — | mock-tested, not run on hardware |
| il3820 | IL3820 2.9" 128x296, IL3895 2.13" 122x250, IL3829 1.54" 200x200 B/W (RAM 4 736 / 4 000 / 5 000 B) | SPI + BUSY | — | mock-tested, not run on hardware |
| st7789 | ST7789 240x240 / 240x320 / 135x240 / 170x320, ST7735 128x160 / 128x128 / 80x160, RGB565 — mains-powered only | SPI | — | mock-tested, not run on hardware |
