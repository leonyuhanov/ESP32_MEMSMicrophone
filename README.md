# ESP32_MEMSMicrophone
Audio input using the SPH0645LM4H I2S MEMS Mirophone Module

ESP32 Module used: Node32S Module
Mic Module Used: SPH0645LM4H I2S MEMS Microphone https://www.adafruit.com/product/3421

Notes on Getting Data from the Mic:
Adafruits module description states its preconfigured as the LEFT mic by default, however it is actualy configured to be the RIGTH
The SPH0645LM4H module outputs data in MSB but use LSB in the I2S config
