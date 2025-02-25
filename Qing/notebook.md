# 02/24/2025
## ESP32-S3

### GPIO
more info:
https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/gpio.html
ESP32-S3 features 45 GPIO pads.
Some GPIO pads cannot be used or do not have the corresponding pin on the chip package.
Each pad can be used as a general purpose I/O or can be connected to an internal peripheral signal.

### ADC (analog to digital converter)
Detailed info of ADC:
https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html
https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/adc.html
We will use this peripheral to monitor the battery of the battlebot.
This peripheral will convert analog signal, such as voltage, into digital signal so that it can be read and processed by the microcontroller

These channels are supported:

ADC1:

        10 channels: GPIO1 - GPIO10

ADC2:

        10 channels: GPIO11 - GPIO20

For now, I choose **GPIO1** and **GPIO2** for **ADC** usage (I may change it if we come up with better design)
#### ADC Attenuation
The largest voltage ESP32S3 could handle with is 1.1V, so ADC attenuation is needed for processing larger voltage.
The largest voltage ESP32S3 can handle with is 3.3 V, which is still larger than the battery voltage, so we decide to use a voltage divider.
##### Calculation:
TBD

### Inter-Integrated Circuit (I2C)
https://docs.espressif.com/projects/arduino-esp32/en/latest/api/i2c.html
https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/i2c.html#i2c-api-configure-driver
I2C is a serial, synchronous, half-duplex communication protocol that allows co-existence of multiple masters and slaves on the same bus. The I2C bus consists of two lines: serial data line (SDA) and serial clock (SCL). Both lines require pull-up resistors.

**MPU 6050** uses this protocol to communicate with ESP32-S3
On the Generic ESP32 the default I2C pins are:

    sdaPin GPIO21

    sclPin GPIO22

However, there is no GPIO22 on ESP32-S3. Instead, I will choose **GPIO8** for **sdaPin** and **GPIO9** for **sclPin**
reference: 
https://docs.espressif.com/projects/esp-idf/en/v4.4/esp32s3/api-reference/peripherals/index.html
https://docs.espressif.com/projects/arduino-esp32/en/latest/libraries.html#datasheet