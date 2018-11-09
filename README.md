# MC3635 & EV3635

Arduino Accelerometer sensor driver for mCube Product MC3635.
==================================================================

The MC3635 is an ultra-low power, low noise, integrated digital output 3-axis accelerometer with a feature set optimized for wearables and the Internet of Moving Things (IoMT) devices. With its low power modes and tiny footprint this accelerometer extends sensor battery life and reduces the space required for the sensor by up to 3X. Low noise and low power are inherent in the monolithic fabrication approach. 

The EVA3635 is a prebuilt circuit board with the MC3635 accelerometer sensor, with an internal sample rate from 14 to 1300 samples/second. It measures acceleration with a wide usage range, from +/-2g up to +/-16g. It offers 8-bit to 14-bit high-precision ADC output which is easy mounted onto a microcontroller platform, such as Arduino. The accelerometer communicates via I2C (or SPI) and gives out motion detection or sample acquisition conditions to trigger an interrupt toward a MCU. 

Sensor data is easily read by simply connecting DVDD to 3.3V, GND to ground, and SPI (DOUT, SCK, SCL) pins to your Arduino SPI (MISO, MOSI, CLK) respectively. Download the MC3635 library from mCube’s website onto the board, run the example sketch, and then sensor data shortly comes out in raw data count and SI unit accelerometer measurements. 
