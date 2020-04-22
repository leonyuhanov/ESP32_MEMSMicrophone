
#include <WiFi.h>
#include <WiFiUdp.h>
#include "driver/i2s.h"
#include "arduinoFFT.h"

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
const uint8_t maxDataFrameSize=64;
uint8_t dataToSend[maxDataFrameSize+1];

//FFT Stuff
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */
const uint16_t samples = maxDataFrameSize; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = I2S_SampleRate;
unsigned int sampling_period_us;
unsigned long microseconds;
double vReal[samples];
double vImag[samples];
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

const char * ssid = "WIFISSID";
const char * password = "WIFIKEY";
//IP ADDRESS OF THE PC RUNNING UNITY
IPAddress unityVirtualEnvironment(192,168,1,10);   
unsigned int txPort=1000;
WiFiUDP udp;

void setup()
{
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  Serial.begin(115200); 
  Serial.printf("\r\n\r\n\r\n");
    
  pinMode(19, INPUT);
  returnResult = i2s_driver_install((i2s_port_t)i2s_num, &i2s_config, 0, NULL);
  //Serial.printf("\r\nDriver Install Result\t%d", returnResult);
  returnResult = i2s_set_pin((i2s_port_t)i2s_num, &pin_config);
  //Serial.printf("\r\nSET PIN Result\t%d", returnResult);
  returnResult = i2s_start((i2s_port_t)i2s_num);
  //Serial.printf("\r\nI2S Start Result\t%d", returnResult);
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

  //Enable WIFI
  WiFi.mode(WIFI_STA);
  WiFi.setHostname("Server");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
        delay(100);
        Serial.print(".");
  }
  Serial.printf("\r\nConnected & TXing...");
}

void loop()
{
  //Serial.printf("\r\n%d", readMic(10, avgGain, avgGainFilter));
  /*
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
      vReal[i] = readMic(5, avgGain, avgGainFilter);
      vImag[i] = 0;
     
      while(micros() - microseconds < sampling_period_us)
      {
        //empty loop
      }
      
      microseconds += sampling_period_us;
  }
  // Print the results of the sampling according to time
  
  //Serial.println("Data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_FLT_TOP, FFT_FORWARD);  // Weigh data  FFT_WIN_TYP_HAMMING
  //Serial.println("Weighed data:");
  //PrintVector(vReal, samples, SCL_TIME);
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); // Compute FFT
  //Serial.println("Computed Real values:");
  //PrintVector(vReal, samples, SCL_INDEX);
  //Serial.println("Computed Imaginary values:");
  //PrintVector(vImag, samples, SCL_INDEX);
  //FFT.ComplexToMagnitude(vReal, vImag, samples); // Compute magnitudes
  //Serial.println("Computed magnitudes:");
  //PrintVector(vReal, (samples >> 1), SCL_FREQUENCY);
  //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
  //Serial.println(x, 6); //Print out what frequency is the most dominant.
  renderUNITY(vReal);
  yield();
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
  for (uint16_t i = 0; i < bufferSize; i++)
  {
    double abscissa;
    /* Print abscissa value */
    switch (scaleType)
    {
      case SCL_INDEX:
        abscissa = (i * 1.0);
  break;
      case SCL_TIME:
        abscissa = ((i * 1.0) / samplingFrequency);
  break;
      case SCL_FREQUENCY:
        abscissa = ((i * 1.0 * samplingFrequency) / samples);
  break;
    }
    //Serial.print(abscissa, 6);
    //if(scaleType==SCL_FREQUENCY)
    //  Serial.print("Hz");
    //Serial.print(" ");
    //Serial.println(vData[i], 4);
    Serial.printf("\t%d", ((int)(roundf(vData[i])/10000))%255 );
    //Serial.printf("\t%f", roundf(vData[i]));
  }
  //Serial.println();
  Serial.printf("\r\n");
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

void renderUNITY(double *vData)
{
  unsigned short int bCnt=0;
  for(bCnt=0; bCnt<samples; bCnt++)
  {
    dataToSend[bCnt] = ((int)(vData[bCnt]/10000))*100;
    //Serial.printf("\t%d", dataToSend[bCnt]);
  }
  //Serial.println();
  dataToSend[maxDataFrameSize]=1;
  udp.beginPacket(unityVirtualEnvironment, txPort);
  udp.write(dataToSend, maxDataFrameSize+1);
  udp.endPacket();
  //Serial.printf("SENT\r\n");
  
}
