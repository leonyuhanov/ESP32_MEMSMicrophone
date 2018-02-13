/*
MEMS i2S Mic: https://www.adafruit.com/product/3421
Node32S Micro
*/
#include <WiFi.h>
#include "driver/i2s.h"

/*
Configuration structure for the i2s interface
	  
	  The Adafruit breakout for the SPH0645LM4H MEMS mic states it works on LEFT by default. this is not true, so use the RIGHT ONLY config
		channel_format: I2S_CHANNEL_FMT_ONLY_RIGHT,
	  
	  The SPH0645LM4H sends out data in MSB but only LSB works in this config
		communication_format: (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB)
*/

i2s_config_t i2s_config = {
      mode: (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      sample_rate: 44100,
      bits_per_sample: I2S_BITS_PER_SAMPLE_32BIT,
      channel_format: I2S_CHANNEL_FMT_ONLY_RIGHT,
      communication_format: (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB),
      intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
      dma_buf_count: 8,
      dma_buf_len: 8
};

//	Configure whatever pins you have available and make sure you set them up with PiNmode() Before setting up i2s system
i2s_pin_config_t pin_config = {
    .bck_io_num = 21, //this is BCK pin
    .ws_io_num = 25, // this is LRCK pin
    .data_out_num = I2S_PIN_NO_CHANGE, // this is DATA output pin
    .data_in_num = 19   //DATA IN
};
const int i2s_num=0;
int retStat = 0;
int32_t sampleIn=0, avgGain=0, peaks[2]={0,0}, avgSampleIn=0;
unsigned short int numOfBlanks=2000, smoothingReads=10;

void setup()
{
  Serial.begin(115200); 
  //Turn off WIFI to save power
  WiFi.mode(WIFI_OFF);
  //Set up pin 19 for data IN from the Mic to the esp32
  pinMode(19, INPUT);
  //Set up pin 21 and 25 as the BCK and LRCK pins
  pinMode(21, OUTPUT);
  pinMode(25, OUTPUT);
  //Init the i2s device
  i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
  i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
  i2s_start((i2s_port_t)i2s_num);
  
  Serial.print("\r\nBegin Level detect...");
  Serial.print("\r\n\tRead 4000 samples to level out...");
  //This pulls in a bunch of samples and does nothing, its just used to settle the mics output
  for(retStat=0; retStat<numOfBlanks*2; retStat++)
  {
    i2s_pop_sample((i2s_port_t)i2s_num, (char*)&sampleIn, portMAX_DELAY);
    delay(1);
  }
  //Pull in x number of samples (IN A QUITE ENVIRONMENT) and create the base gain average(the zero point for the mic)
  Serial.print("\r\n\tRead 2000 samples to get avg...");
  for(retStat=0; retStat<numOfBlanks; retStat++)
  {
    i2s_pop_sample((i2s_port_t)i2s_num, (char*)&sampleIn, portMAX_DELAY);
    sampleIn>>=14;
    avgGain -= sampleIn;
    delay(1);
  }
  avgGain = avgGain/numOfBlanks;
  Serial.printf("\t\tAVG Gain=\t%i", avgGain);
  peaks[0]=avgGain;
  peaks[1]=-avgGain;
  Serial.printf("\r\n\tSetting MAX gain to\t%i\tMin Gain to\t%i\r\n", peaks[0], peaks[1]);
  delay(1000);
}

void loop()
{
  sampleIn=0;
  avgSampleIn=0;

  //read in smoothingReads number of times and use the average
  for(retStat=0; retStat<smoothingReads; retStat++)
  {
    //this reads 32bits as 4 chars into a 32bit INT variable
	i2s_pop_sample((i2s_port_t)i2s_num, (char*)&sampleIn, portMAX_DELAY);
	//this pushes out all the unwanted bits as we only need right channel data.
    sampleIn>>=14;
    avgSampleIn+=sampleIn;
  }
  sampleIn = round((float)avgSampleIn/smoothingReads);
  
  // I want the output to look like a wave on the serial ploter with silence around the 0 point and 100 and -100 max and min
  if(sampleIn>0)
  {
    avgSampleIn = round( ((float)(sampleIn+avgGain)/peaks[0])*100 );
    if(avgSampleIn>100)
    {
      avgSampleIn = 100;
    }
  }
  else if (sampleIn<0)
  {
    avgSampleIn = round( ((float)(sampleIn+avgGain)/peaks[1])*100 );
    if(avgSampleIn<-100)
    {
      avgSampleIn = -100;
    }
  }
  Serial.println( avgSampleIn );
  yield();
}
