/*
 * Project v4-prototype
 * Description: v4-prototype code
 * Author: Allen Nikka
 * Date: 4.1.2018
 */

#include "ADS6838SR.h"

ads6838 MY_ADC;
void setup(){
  MY_ADC.init(1);
  pinMode(D0, OUTPUT);
  // digitalWrite(D0, HIGH);
  pinMode(D1, OUTPUT);
  // digitalWrite(D1, HIGH);
  pinMode(D2, OUTPUT);
  // digitalWrite(D2, HIGH);
  pinMode(D3, OUTPUT);
  // digitalWrite(D3, HIGH);
  pinMode(D7, OUTPUT);
  // digitalWrite(D7, HIGH);
  delay(1000);
}

uint16_t rx_dest[TX_BUFFER_SIZE];
uint8_t flag = 0;
void loop(){
  digitalWrite(D7, LOW);
  MY_ADC.read8();
  // MY_ADC.read8_DMA(flag);
  // if(flag) MY_ADC.get8_DMA(rx_dest);
  digitalWrite(D7, HIGH);
}

//
//  #include "waveform.h"
//  #include "powerWave.h"
//  #include "circuitVal.h"
//  #include "SparkIntervalTimer.h"
//
// const size_t FILTER_KERNAL_SIZE = 10; //must be even
// const size_t SAMPLE_BUF_SIZE =  512 + FILTER_KERNAL_SIZE; //these rates are doubled from base
// const long SAMPLE_RATE = 4096*2;
// const int SAMPLE_RATE_SHIFT = log(SAMPLE_RATE)/log(2);
// const int ipin1 = A1;
// const int vpin1 = A0;
// const int relay1 = D0;
// const bool SERIAL_DEBUG = true;
//
// enum State { STATE_INIT, STATE_COLLECT, STATE_TRANSFER, STATE_PROCESS };
//
// typedef struct {
// 	volatile bool free;
// 	volatile size_t  index;
// 	double v_data[SAMPLE_BUF_SIZE];
// 	double i_data[SAMPLE_BUF_SIZE];
// 	unsigned long t_data[SAMPLE_BUF_SIZE];
// } SampleBuf;
//
// // Object Instatiation:
// IntervalTimer timer;
// SampleBuf samples;
// circuitVal circuit1(0.02, 1500);
// // NOTE: set to differentiate between loads that
// // have at least 40 mA difference over time periods of 1.
// // lowering current threshold reduces number of reads, but
// // increases the variability between readings
//
// State state = STATE_INIT;
// const int waveSize = SAMPLE_BUF_SIZE;
//
// waveform vWave(waveSize, FILTER_KERNAL_SIZE);
// waveform iWave(waveSize, FILTER_KERNAL_SIZE);
// powerWave pWave(waveSize, SAMPLE_RATE);
//
// // Helper Functions
// void dataHandler(const char *event, const char *data)
// {
//   if(strcmp("setRelay1",event)==0)
//   {
//     relayHandler(relay1, data);
//   }
//   if(strcmp("sendEvent",event)==0)
//   {
//     sendEvent();
//   }
// }
//
// void relayHandler(int relayPin, const char *data)
// {
//   if(strcmp(data, "OFF")==0) digitalWrite(relayPin, LOW);
//   if(strcmp(data, "ON")==0) digitalWrite(relayPin, HIGH);
// }
//
// bool pushDataFlag = false;
// void sendEvent()
// {
//   pushDataFlag = true;
// }
//
//
// const double currentThreshold_diff = 0.05;
// unsigned long sendInterval;
// const unsigned long sendIntervalDelta = 20*3600; // 20 minutes
//
// void pushData(circuitVal& c1, bool& pushDataFlag)
// {
//   if((millis() - sendInterval) > sendIntervalDelta > 0 && !SERIAL_DEBUG)
//   {
//     String outStr = c1.get_data_string();
//     Particle.publish("send-i,v,pf,s,p,q", outStr, 5, PRIVATE);
//   }
//   if(pushDataFlag && !SERIAL_DEBUG)
//   {
//     pushDataFlag = false;
//     String outStr = c1.get_data_string();
//     Particle.publish("send-i,v,pf,s,p,q", outStr, 5, PRIVATE);
//   }
// }
//
// void logCircuit(waveform& iWave, waveform& vWave, powerWave& pWave, circuitVal& c1){
//   double iRMS = iWave.getRMS();
//   double vRMS = vWave.getRMS();
//   double pf = pWave.getPF();
//   double apparentP = pWave.getApparentP();
//   double realP = pWave.getRealP();
//   double reactiveP = pWave.getReactiveP();
//   size_t numHarmoics = pWave.getNumHarmoics();
//   float harmonics[numHarmoics];
//   for(size_t i = 0; i < numHarmoics; i++)
//   {
//     harmonics[i] = pWave.getHarmonicAngle(i);
//   }
//   c1.addData(iRMS, vRMS, pf, apparentP, realP, reactiveP, harmonics);
//   String out = c1.get_data_string();
//   // Serial.println(out); //NOTE: Continuous Serial Debug Option, uncomment
//   if(c1.data_ready()){
//     digitalWrite(D7, HIGH);
//     if(SERIAL_DEBUG){
//       Serial.print(System.freeMemory());Serial.print(" , "); //FIXME
//       Serial.println(out);
//     } else {
//       sendInterval = millis();
//       Particle.publish("send-i,v,pf,s,p,q", out, 5, PRIVATE);
//     }
//     digitalWrite(D7, LOW);
//   }
// }
//
//
// void transferBuff(SampleBuf& buff, bool& outFlag, waveform& vWave, waveform& iWave){
//   for(size_t i = 0; i < SAMPLE_BUF_SIZE; i++)
//   {
// 		unsigned long time_val = buff.t_data[i];
// 		//all values now in mV
// 		double v_ratio = 0.265; // * 1.06;
// 		double i_ratio = 0.035;
// 		//values without ratios above are in mV detected by microcontroller
// 		double i_val = (i_ratio * buff.i_data[i] * 3.3 * 100000) / 4096;
// 		double v_val = (v_ratio * buff.v_data[i] * 3.3 * 100000) / 4096;
// 		iWave.addData(time_val, floorf(i_val)/100);
// 		vWave.addData(time_val, floorf(v_val)/100);
//
// 	}
//   outFlag = true;
//   // Serial.println("");
// }
//
//
// void setup() {
// 	// Put initialization like pinMode and begin functions here.
//   Serial.begin(9600);
//   pinMode(relay1, OUTPUT);
//   digitalWrite(relay1, HIGH);
// 	pinMode(D7, OUTPUT); //FIXME
//   Particle.subscribe("setRelay", dataHandler);
//   Particle.subscribe("sendEvent", dataHandler);
//   setADCSampleTime(ADC_SampleTime_3Cycles);
// 	pWave.addComponents(vWave, iWave);
//   sendInterval = millis();
// 	delay(1000); //FIXME
//
// }
//
// bool outFlag = false;
// void loop() {
//   switch(state)
// 	{
// 		case STATE_INIT:
//   		// Reset the buffers
//       // TODO: Check the oferall runtime without any outputs using the follwing
//       // Serial.println(micros());
//       samples.free = true;
//       samples.index = 0;
// 			outFlag = false;
// 			// We want to sample at 16 KHz
// 			// 16000 samples/sec = 62.5 microseconds
// 			// The minimum timer period is about 10 micrseconds
// 			timer.begin(timerISR, 1000000 >> SAMPLE_RATE_SHIFT, uSec); //122 uSec sample time
// 			state = STATE_COLLECT;
// 			break;
// 		case STATE_COLLECT:
//       if(!(samples.free)) {
//         state = STATE_TRANSFER;
//       }
// 			break;
// 		case STATE_TRANSFER:
// 			timer.end();
//       transferBuff(samples, outFlag, vWave, iWave);
// 			state = STATE_PROCESS;
// 			if(outFlag) state = STATE_PROCESS;
// 		break;
// 		case STATE_PROCESS:
// 			outFlag = false;
//
// 			iWave.movingAvgFilter();
// 			vWave.movingAvgFilter();
// 			iWave.getRMS();
// 			vWave.getRMS();
// 			pWave.calcP();
// 			pWave.computeFFT();
//       logCircuit(iWave, vWave, pWave, circuit1);
//       pushData(circuit1, pushDataFlag);
// 			pWave.clearWave();
// 			vWave.resetWave();
// 			iWave.resetWave();
// 			if(SERIAL_DEBUG) delay(15);
// 			state = STATE_INIT;
// 			break;
// 		default:
// 			//default to dump buffers and start over
// 			timer.end();
// 			state = STATE_INIT;
// 			break;
// 	}
//
//
// }
//
// void timerISR() {
// 	// This is an interrupt service routine. Don't put any heavy calculations here
// 	// or call anything that's not interrupt-safe, such as:
// 	// Serial, String, any memory allocation (new, malloc, etc.), Particle.publish and other Particle methods, and more.
//   SampleBuf *sb = &samples;
//   if(sb->free){
//     sb->i_data[sb->index] = (double) (analogRead(ipin1));
// 		sb->v_data[sb->index] = (double) (analogRead(vpin1));
// 		sb->t_data[sb->index++] = micros();
//   }
//   if (sb->index >= SAMPLE_BUF_SIZE) {
//     // Buffer has been filled. Let it be sent via TCP.
//     sb->free = false;
//     sb->index = 0;
//   }
// }
