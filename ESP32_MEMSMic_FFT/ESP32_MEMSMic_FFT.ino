/*
  MASK ADDRESS  Address 3C:71:BF:29:59:26
 */
#include <esp_now.h>
#include <WiFi.h>
#include "driver/i2s.h"
#include "arduinoFFT.h"

#define WIFI_CHANNEL 1
#define I2S_SampleRate  44100
#include "arduinoFFT.h"
i2s_config_t i2s_config = {
      mode: (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
      sample_rate: I2S_SampleRate,
      bits_per_sample: I2S_BITS_PER_SAMPLE_32BIT,
      channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
      communication_format: (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB),
      intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
      dma_buf_count: 8,
      dma_buf_len: 8
};
   
i2s_pin_config_t pin_config = {
    .bck_io_num = 21, //this is BCK pin
    .ws_io_num = 25, // this is LRCK pin
    .data_out_num = I2S_PIN_NO_CHANGE, // this is DATA output pin
    .data_in_num = 19   //DATA IN
};
const int i2s_num=0;
int retStat = 0;
int32_t sampleIn=0, avgGain=0, avgGainFilter=0, peaks[2]={60000,-60000}, avgSampleIn=0;
int32_t currentRead=0;
unsigned short int numOfBlanks=1000;
int returnResult=0;

//ESPNOW Stuff
esp_now_peer_info_t slave;
uint8_t remoteMac[] = {0x3C, 0x71, 0xBF, 0x29, 0x59, 0x26};
const uint8_t maxDataFrameSize=50, maxESPNowSamples = 19;
const esp_now_peer_info_t *peer = &slave;
uint8_t dataToSend[maxDataFrameSize];

//FFT Stuff
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
const uint16_t samples = 512; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = I2S_SampleRate;
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[samples];
double vImag[samples];
double espNowSamples [maxESPNowSamples];
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200); 
  Serial.printf("\r\n\r\n\r\n");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  //ESPNOW STUFF
  esp_now_init();
  delay(10);
  memcpy( &slave.peer_addr, &remoteMac, 6 );
  slave.channel = WIFI_CHANNEL;
  slave.encrypt = 0;
  esp_now_add_peer(peer);
  
  pinMode(19, INPUT);
  returnResult = i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
  returnResult = i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
  returnResult = i2s_start((i2s_port_t)i2s_num);
  Serial.print("\r\nBegin Level detect...");
  Serial.printf("\r\n\tRead\t%d\tsamples to level out...", numOfBlanks);
  for(retStat=0; retStat<numOfBlanks; retStat++)
  {
    i2s_pop_sample((i2s_port_t)i2s_num, (char*)&sampleIn, portMAX_DELAY);
    delay(1);
  }
  Serial.printf("\r\n\tRead\t%d\tsamples to get avg...", numOfBlanks);
  for(retStat=0; retStat<numOfBlanks; retStat++)
  {
    i2s_pop_sample((i2s_port_t)i2s_num, (char*)&sampleIn, portMAX_DELAY);
    sampleIn>>=14;
    avgGain += sampleIn;
    delay(1);
  }
  avgGain = avgGain/numOfBlanks;
  
  //Grab filtered gain
  Serial.printf("\r\n\tRead\t%d\tsamples to get avg filter gain...", numOfBlanks);
  for(retStat=0; retStat<numOfBlanks; retStat++)
  {
    i2s_pop_sample((i2s_port_t)i2s_num, (char*)&sampleIn, portMAX_DELAY);
    sampleIn>>=14;
    avgGainFilter += avgGain-sampleIn;
    delay(1);
  }
  avgGainFilter = avgGainFilter/numOfBlanks;
  //set peaks
  peaks[0]=-avgGain;
  peaks[1]=avgGain;

  Serial.printf("\r\nAVG Gain=\t%i\tAVG Filter\t%i\tMaxGain\t%i\tMinGain\t%i", avgGain, avgGainFilter, peaks[0], peaks[1]);
  delay(1000);
}

void loop()
{
  /*
  Serial.printf("\r\n%d", readMic(10, avgGain, avgGainFilter));
  currentRead = readMic(10, avgGain, avgGainFilter);
  if(currentRead>avgGainFilter || currentRead<-avgGainFilter)
  {
    Serial.printf("\r\n%d", currentRead);
  }
  else
  {
    Serial.printf("\r\n0");
  }
  */
  microseconds = micros();
  for(int i=0; i<samples; i++)
  {
      vReal[i] = readMic(2, avgGain, avgGainFilter);
      vImag[i] = 0;
      
      while(micros() - microseconds < sampling_period_us)
      {
      }
      microseconds += sampling_period_us;
  }
  
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  // Weigh data 
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); // Compute FFT
  FFT.ComplexToMagnitude(vReal, vImag, samples); // Compute magnitudes
  txToMask(vReal);
  yield();
}

int32_t readMic(unsigned short int numberOfSamples, int32_t gain, int32_t fGain)
{
  unsigned short int samps=numberOfSamples;
  int32_t inValues=0;
  
  for(samps; samps>0; samps--)
  {
    i2s_pop_sample((i2s_port_t)i2s_num, (char*)&sampleIn, portMAX_DELAY);
    sampleIn>>=14;
    inValues += (gain-sampleIn)-fGain;
    yield();
  }
  inValues = inValues/numberOfSamples;
  return inValues;
}

void txToMask(double *vData)
{
  unsigned short int bCnt=0, blocks=((samples-2)/2)/maxESPNowSamples, blockCnt=0, bIndex=0;

  //Clear TX buffer
  for(bCnt=0; bCnt<maxESPNowSamples; bCnt++)
  {
    espNowSamples[bCnt]=0;
  }
  //compile sample avgearges
  bIndex=1;
  for(bCnt=0; bCnt<maxESPNowSamples; bCnt++)
  {
    for(blockCnt=0; blockCnt<blocks; blockCnt++)
    {
      espNowSamples[bCnt] += vData[bIndex];
      bIndex++;
    }
    //grab avegarge of blocks
    espNowSamples[bCnt] = espNowSamples[bCnt]/blocks;
    espNowSamples[bCnt] = (espNowSamples[bCnt]/10000)*255;
  }
  
  for(bCnt=0; bCnt<maxESPNowSamples; bCnt++)
  {
    
    dataToSend[bCnt] = espNowSamples[bCnt];
  }
  dataToSend[maxDataFrameSize-1]=1;
  esp_now_send(slave.peer_addr, dataToSend, maxDataFrameSize);
}
