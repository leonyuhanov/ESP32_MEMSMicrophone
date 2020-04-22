# ESP32_MEMSMicrophone
Audio input using the SPH0645LM4H I2S MEMS Mirophone Module

ESP32 Module used: Node32S Module
Mic Module Used: SPH0645LM4H I2S MEMS Microphone https://www.adafruit.com/product/3421

Notes on Getting Data from the Mic:
Adafruits module description states its preconfigured as the LEFT mic by default, however it is actualy configured to be the RIGTH
The SPH0645LM4H module outputs data in MSB but use LSB in the I2S config

# Notes about FFT

I have no idea what im doing :)

Im using this lib https://github.com/kosme/arduinoFFT


# ESP32_MEMSMIC_FFT_To_Unity

A demo that processes audio aquired using the above system and TX over UDP to a UNIT3D Virtual Environment taht simulates a Sptecrum VU meter of the data from the ESP32
