# ESP32_MEMSMicrophone
Audio input using the SPH0645LM4H I2S MEMS Mirophone Module

ESP32 Module used: Node32S Module
Mic Module Used: SPH0645LM4H I2S MEMS Microphone https://www.adafruit.com/product/3421

Notes on Getting Data from the Mic:
Adafruits module description states its preconfigured as the LEFT mic by default: use I2S_CHANNEL_FMT_ONLY_LEFT

The SPH0645LM4H module outputs data in MSB but use LSB in the I2S config

# Getting clear data round the 0 line
The code in my FFT folders has some fixes for getting clean audio and automatic gain at boot:

1. In a right loop read 1000 samples from the mic, do nothing with them
2. In a second tight loop, read 1000 samples and work out the average value, use that as the 0 gain you subtract from. EG your values at boot will average around 7000 for example. When you read a sample subtract your reading from 7000 to get a nice cleaner output
3. In a 3rd tight loop, read a further 1000 samples, using a counter ADD the (GAIN-sample) then work out teh avgerage flutter gain. Store that as your AVGFlutter

When you actualy begin to take real world readings, after boot, your reading will look like (AVGGain-Sample)-AVGFlutter


# Notes about FFT

I have no idea what im doing, but it works!

Im using this lib https://github.com/kosme/arduinoFFT

# ESP32_MEMSMIC_FFT

Does FFT with 512 samples (bins). We then use the 1st half of these bins as data to TX over ESPNOW. The variable "maxESPNowSamples" dicates teh number of averaged BINS you send to your device.

# ESP32_MEMSMIC_FFT_To_Unity

A demo that processes audio aquired using the above system and TX over UDP to a UNIT3D Virtual Environment taht simulates a Sptecrum VU meter of the data from the ESP32
