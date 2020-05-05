/*
  EGOO  Address 3C:71:BF:29:50:D7
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
uint8_t remoteMac[] = {0x3C, 0x71, 0xBF, 0x29, 0x50, 0xD7};
const uint8_t maxDataFrameSize=50, maxESPNowSamples = 16;
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
double maxFFTPoints[samples/2] = {1902818,812638,1158489,1515211,1649099,1635141,1522206,1491714,1483937,1466099,1469292,1477698,1463164,1464090,1466766,1448747,1426930,1409263,1415557,1299615,1467219,1476826,1459626,1516967,1514021,1510431,1477621,1307049,1466322,1491902,1498201,1491062,1468483,1460184,776415,992915,1416923,1461952,1429634,1447295,1417831,1414938,1408484,1361142,1077517,430195,1365995,1416491,1432279,1438010,1550123,1594991,1719352,1836724,1812289,2033056,2006598,2045222,2191969,2417146,2472360,924071,2070110,2600669,2554013,2554622,2617561,2597690,2721693,2810980,2797480,2921645,2824582,2825142,732789,3181733,3396843,3586562,3595704,3578140,3685053,3654834,3623698,3539335,3239472,1579163,3440546,3440503,3490238,3473477,3592368,3721979,3790024,3757636,3865781,1168731,2418224,4177941,4418507,4557955,4570595,4830108,4721616,4528597,5567947,5839055,4333307,464434,47804,488054,4705400,6216413,6568639,6370753,6158616,5979965,5406579,4744524,4116116,3494911,3090889,3237333,3242504,1907249,1623199,3375703,3274631,3147729,2861758,2844337,3111586,2858381,2875522,2922826,2324496,2821322,2907468,2831482,2693312,2139731,2118775,2437877,1043957,17547,18808,19430,17110,15528,137321,1399705,1873084,1801317,1746250,1683993,1631478,1559803,825185,1472169,1408243,1397282,1267452,1241822,1240933,986467,1181911,783582,1048972,908726,1063719,1126462,711480,1090471,924479,1081884,1078315,708461,35579,14234,14414,15290,14771,14893,14095,15114,21167,312824,1304902,1295197,1460066,1614068,1655863,1601221,1460733,1711687,1846936,1877763,1966671,1874616,1954802,2278464,2497882,2155430,2905576,2480636,302760,27567,29970,295097,3004058,5083421,5564746,6129488,6296214,4178016,7217727,7557353,7227618,4961446,7398438,7332301,5500692,6658740,6673541,6692312,5198651,6220106,2528658,5207652,4522832,5342114,2032093,45172,41925,42977,40854,37645,34082,31229,29428,26977,25630,25187,27264,29616,32269,35379,37718,37078,39196,566327,3185879,3022835,3270015,2754567,297667,49393};
//double maxFFTPoints[samples/2];
double espNowSamples [maxESPNowSamples];
#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03
//timers
//unsigned long consoleOutputTimer[3];

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
  
  //set up maxPoints
  /*
  for(retStat=0; retStat<samples/2; retStat++)
  {
    maxFFTPoints[retStat] = 0;
  }
  */
  Serial.printf("\r\nAVG Gain=\t%i\tAVG Filter\t%i\tMaxGain\t%i\tMinGain\t%i", avgGain, avgGainFilter, peaks[0], peaks[1]);
  //startTimer(1000, consoleOutputTimer);
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
  //computeMaxValues(vReal);
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

/*
void computeMaxValues(double *vData)
{
  for(retStat=0; retStat<samples/2; retStat++)
  {
    if(vData[retStat]>maxFFTPoints[retStat])
    {
      maxFFTPoints[retStat] = vData[retStat];
    }
  }
  if( hasTimedOut(consoleOutputTimer) )
  {
    Serial.printf("\r\n");
    for(retStat=0; retStat<samples/2; retStat++)
    {
      Serial.printf("%f,",maxFFTPoints[retStat]);
    }
    startTimer(1000, consoleOutputTimer);
  }
}
*/

void txToMask(double *vData)
{
  unsigned short int bCnt=0, blocks=((samples-2)/2)/maxESPNowSamples, blockCnt=0, bIndex=0;

  //compile sample avgearges
  bIndex=1;
  for(bCnt=0; bCnt<maxESPNowSamples; bCnt++)
  {
    espNowSamples[bCnt]=0;
    for(blockCnt=0; blockCnt<blocks; blockCnt++)
    {
      espNowSamples[bCnt] += (vData[bIndex]/maxFFTPoints[bIndex]);
      bIndex++;
    }
    //grab avegarge of blocks
    espNowSamples[bCnt] = (espNowSamples[bCnt]/blocks)*255;
    dataToSend[bCnt] = (int)espNowSamples[bCnt]; 
  }
  /*
  Serial.printf("\r\n");
  for(bCnt=0; bCnt<maxESPNowSamples; bCnt++)
  {
    dataToSend[bCnt] = espNowSamples[bCnt];
    Serial.printf("%d\t", dataToSend[bCnt]);
  }
  */
  dataToSend[maxDataFrameSize-1]=1;
  esp_now_send(slave.peer_addr, dataToSend, maxDataFrameSize);
}

byte hasTimedOut(unsigned long* timerArray)
{
  timerArray[1] = millis();
  if(timerArray[2] < timerArray[1]-timerArray[0])
  {
    return 1;
  }
  return 0;
}
void startTimer(unsigned long durationInMillis, unsigned long* timerArray)
{
  timerArray[0] = millis(); 
  timerArray[2] = durationInMillis;
}
