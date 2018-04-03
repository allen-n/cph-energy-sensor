/*
 * Project v4-prototype
 * Description: v4-prototype code
 * Author: Allen Nikka
 * Date: 4.1.2018
 */

  #include "waveform.h"
  #include "powerWave.h"
  #include "ADS6838SR.h"
  #include "SparkIntervalTimer.h"

 const size_t SAMPLE_BUF_SIZE =  512 + 10; //these rates are doubled from base
 const long SAMPLE_RATE = 4096*2; //8192/2;
 const int SAMPLE_RATE_SHIFT = log(SAMPLE_RATE)/log(2);
 const unsigned long MAX_RECORDING_LENGTH_MS = 250;
 const int ipin1 = A1;
 const int vpin1 = A0;
 const int relay1 = D0;
 unsigned long recordingStart;
 enum State { STATE_INIT, STATE_COLLECT, STATE_TRANSFER, STATE_PROCESS };

 typedef struct {
 	volatile bool free;
 	volatile size_t  index;
 	double v_data[SAMPLE_BUF_SIZE];
 	double i_data[SAMPLE_BUF_SIZE];
 	unsigned long t_data[SAMPLE_BUF_SIZE];
 } SampleBuf;

 IntervalTimer timer;
 SampleBuf samples;

 State state = STATE_INIT;
 const int waveSize = SAMPLE_BUF_SIZE;
 // ioLib reader;
 waveform vWave(waveSize);
 waveform iWave(waveSize);
 powerWave pWave(waveSize, SAMPLE_RATE);


 void dataHandler(const char *event, const char *data)
 {
   if(strcmp("setRelay1",event)==0)
   {
     relayHandler(relay1, data);
   }
   if(strcmp("sendEvent",event)==0)
   {
     sendEvent();
   }
 }

 void relayHandler(int relayPin, const char *data)
 {
   if(strcmp(data, "OFF")==0) digitalWrite(relayPin, LOW);
   if(strcmp(data, "ON")==0) digitalWrite(relayPin, HIGH);
 }

 bool pushDataFlag = false;
 void sendEvent()
 {
   pushDataFlag = true;
 }

 void pushData(waveform& iWave, waveform& vWave, powerWave& pWave, bool& pushDataFlag)
 {
   if(pushDataFlag)
   {
     pushDataFlag = false;
     double iRMS = iWave.getRMS();
     double vRMS = vWave.getRMS();
     double pf = pWave.getPF();
     double apparentP = pWave.getApparentP();
     double realP = pWave.getRealP();
     double reactiveP = pWave.getReactiveP();
     size_t numHarmoics = pWave.getNumHarmoics();
     float harmonicsRE[numHarmoics];
     float harmonicsIM[numHarmoics];

     String iS = String(iRMS, 2) + ",";
     String vS = String(vRMS, 2) + ",";
     String pfS = String(pf, 2) + ",";
     String sS = String(apparentP, 2) + ",";
     String pS = String(realP, 2) + ",";
     String qS = String(reactiveP, 2) + ",";
     String hS;

     for(size_t i = 1; i < numHarmoics; i+=2)
     {
       harmonicsRE[i] = pWave.getHarmonic(i); //TODO: Test if the magnitude @ 60Hz match with RMS power
       harmonicsIM[i] = pWave.getHarmonicAngle(i); //TODO: Test if this is a good device distinguisher
       hS += String(harmonicsRE[i], 2) + ",";
       hS += String(harmonicsIM[i], 2) + ",";
     }

     Particle.publish("send-i,v,pf,s,p,q", iS + vS + pfS + sS + pS + qS + hS, 5, PRIVATE);
   }
 }

 double prevCurrent[4] = {0,0,0,0};
 int prevCurrentindex = 4;
 //FIXME: Affects the measurement sensitivity, 2 is too low
 const double currentThreshold = 0.03;
 const double currentThreshold_diff = 0.05;
 unsigned long sendInterval;
 const unsigned long sendIntervalDelta = 20*3600; // 20 minutes
 void logCurrentChange(waveform& iWave, waveform& vWave, powerWave& pWave, bool serial)
 {
 		// Determining whether moving average has shifted by threshold amount,
 		// if so log a current change

     int now = prevCurrentindex % 4;
     int min1 = (prevCurrentindex - 1) % 4;
     int min2 = (prevCurrentindex - 2) % 4;
     int min3 = (prevCurrentindex - 3) % 4;
     prevCurrent[now] = iWave.getRMS();

 		int lenDiff = 4;
     double diff[lenDiff] = {
       (prevCurrent[now] - prevCurrent[min1]),
       (prevCurrent[min1] - prevCurrent[min2]),
       (prevCurrent[min2] - prevCurrent[now]),
 			(prevCurrent[now] - prevCurrent[min3]),
     };

 		//abs wasn't working, use the following loop to take absolute value
     for(int i = 0; i < lenDiff; ++i)
     {
       if(diff[i] < 0) diff[i] = diff[i] * -1;
     }

     // Send data if there has been a jump or it has been 20
     // minutes since the last one
     if(
         (diff[0] < currentThreshold &&
         diff[1] < currentThreshold &&
 				diff[2] < currentThreshold &&
         diff[3] > currentThreshold_diff) ||
         (millis() - sendInterval >= sendIntervalDelta)
       )
     {
       sendInterval = millis();
       double vRMS = vWave.getRMS();
       double pf = pWave.getPF();
       double apparentP = pWave.getApparentP();
       double realP = pWave.getRealP();
       double reactiveP = pWave.getReactiveP();
 			size_t numHarmoics = pWave.getNumHarmoics();
 			float harmonics[numHarmoics];
 			for(size_t i = 0; i < numHarmoics; i++)
 			{
 				// harmonics[i] = pWave.getHarmonic(i); //TODO: Test if the magnitude @ 60Hz match with RMS power
         harmonics[i] = pWave.getHarmonicAngle(i); //TODO: Test if this is a good device distinguisher
 			}

 			if(serial)
 			{
 				String iS = String(prevCurrent[now], 2) + " ";
 	      String vS = String(vRMS, 2) + " ";
 	      String pfS = String(pf, 2) + " ";
 	      String sS = String(apparentP, 2) + " ";
 	      String pS = String(realP, 2) + " ";
 	      String qS = String(reactiveP, 2) + " ";
 				String hS;
 				for(size_t i = 0; i < numHarmoics; i++)
 				{
 					hS += String(harmonics[i], 2) + " ";
 				}
 				Serial.println( iS + vS + pfS + sS + pS + qS + hS);
 			} else {
 				String iS = String(prevCurrent[now], 2) + ",";
 	      String vS = String(vRMS, 2) + ",";
 	      String pfS = String(pf, 2) + ",";
 	      String sS = String(apparentP, 2) + ",";
 	      String pS = String(realP, 2) + ",";
 	      String qS = String(reactiveP, 2) + ",";
 				String hS;
 				for(size_t i = 0; i < numHarmoics; i++)
 				{
 					hS += String(harmonics[i], 2) + ",";
 				}
 	      Particle.publish("send-i,v,pf,s,p,q", iS + vS + pfS + sS + pS + qS + hS, 5, PRIVATE);
 			}
     }
     prevCurrentindex++;
     if(prevCurrentindex >= 400) prevCurrentindex = 4;
 }


 void setup() {
 	// Put initialization like pinMode and begin functions here.
   Serial.begin(9600);
   pinMode(relay1, OUTPUT);
   digitalWrite(relay1, HIGH);
 	pinMode(D7, OUTPUT); //FIXME
   Particle.subscribe("setRelay", dataHandler);
   Particle.subscribe("sendEvent", dataHandler);
   setADCSampleTime(ADC_SampleTime_3Cycles);
 	pWave.addComponents(vWave, iWave);
   sendInterval = millis();
 	delay(1000); //FIXME

 }


 void transferBuff(SampleBuf& buff, bool& outFlag, waveform& vWave, waveform& iWave){
   // Serial.println("Begin Data: ");
   for(size_t i = 0; i < SAMPLE_BUF_SIZE; i++)
   {
 		unsigned long time_val = buff.t_data[i];
 		//all values now in mV
 		double v_ratio = 0.265; // * 1.06;
 		double i_ratio = 0.035;
 		//values without ratios above are in mV detected by microcontroller
 		double i_val = (i_ratio * buff.i_data[i] * 3.3 * 100000) / 4096;
 		double v_val = (v_ratio * buff.v_data[i] * 3.3 * 100000) / 4096;
 		iWave.addData(time_val, floorf(i_val)/100);
 		vWave.addData(time_val, floorf(v_val)/100);

 	}
   outFlag = true;
   // Serial.println("");
 }

 bool outFlag = false;
 void loop() {
   switch(state)
 	{
 		case STATE_INIT:
   		// Reset the buffers
       // TODO: Check the oferall runtime without any outputs using the follwing
       // Serial.println(micros());
       samples.free = true;
       samples.index = 0;
 			outFlag = false;
 			// We want to sample at 16 KHz
 			// 16000 samples/sec = 62.5 microseconds
 			// The minimum timer period is about 10 micrseconds
 			timer.begin(timerISR, 1000000 >> SAMPLE_RATE_SHIFT, uSec); //122 uSec sample time
 			recordingStart = millis();
 			state = STATE_COLLECT;
 			break;
 		case STATE_COLLECT:
       if(!(samples.free)) {
         state = STATE_TRANSFER;
       }
 			// else if (millis() - recordingStart >= MAX_RECORDING_LENGTH_MS) {
 			// 	state = STATE_TRANSFER;
 			// 	// Serial.println("in collect, going to transfer"); //FIXME timer
 			// }
 			break;
 		case STATE_TRANSFER:
 			timer.end();
       transferBuff(samples, outFlag, vWave, iWave);
 			state = STATE_PROCESS;
 			if(outFlag) state = STATE_PROCESS;
 		break;
 		case STATE_PROCESS:
       // TODO: check memory usage with following line after timing
       // Serial.println(System.freeMemory());
 			digitalWrite(D7, HIGH);
 			outFlag = false;
 			//Calculating wave parameters
 			iWave.movingAvgFilter();
 			vWave.movingAvgFilter();
 			iWave.getFrequency();
 			iWave.getRMS();
 			vWave.getFrequency();
 			vWave.getRMS();
 			pWave.calcP();
 			// printVect(pWave); //FIXME
 			// Serial.println("Post, FFT, Data"); //FIXME timer
 			pWave.computeFFT();
 			// printVect(pWave); //FIXME
       // printWaveInfo(vWave, iWave, pWave, true);

       // TODO: test timing with the serial argument as false in the following:
 			logCurrentChange(iWave, vWave, pWave, false);
       pushData(iWave, vWave, pWave, pushDataFlag);
 			pWave.clearWave();
 			vWave.resetWave();
 			iWave.resetWave();
 			digitalWrite(D7, LOW);
 			delay(100); //FIXME: Only necessary if we are doing continuous serial printing
 			state = STATE_INIT;
 			break;
 		default:
 			//default to dump buffers and start over
 			timer.end();
 			state = STATE_INIT;
 			break;
 	}


 }

 void timerISR() {
 	// This is an interrupt service routine. Don't put any heavy calculations here
 	// or call anything that's not interrupt-safe, such as:
 	// Serial, String, any memory allocation (new, malloc, etc.), Particle.publish and other Particle methods, and more.
   SampleBuf *sb = &samples;
   if(sb->free){
     sb->i_data[sb->index] = (double) (analogRead(ipin1));
 		sb->v_data[sb->index] = (double) (analogRead(vpin1));
 		sb->t_data[sb->index++] = micros();
   }
   if (sb->index >= SAMPLE_BUF_SIZE) {
     // Buffer has been filled. Let it be sent via TCP.
     sb->free = false;
     sb->index = 0;
   }
 }

 void printWaveInfo(waveform& vWave, waveform& iWave, powerWave& pWave, bool waves)
 /*printWaveInfo(waveform vWave, waveform iWave, waveform pWave)*/
 {
 	//FIXME: Print current and/or voltage waveforms to serial out
 	if(waves)
 	{
 		int waveSize = iWave._numPts;
 		Serial.println("Current Wave");
 		printWaveSerial(waveSize, iWave);
     Serial.println("");
 		Serial.println("Voltage Wave");
 		Serial.println("");
 		printWaveSerial(waveSize, vWave);
     Serial.println("");
 	}

 	Serial.println("Wave Information:");
 	Serial.print("I Frequency (Hz), ");
 	Serial.print(iWave.getFrequency());
 	Serial.print(", I RMS , ");
 	Serial.print(iWave.getRMS());
 	Serial.print(", V RMS , ");
 	Serial.print(vWave.getRMS());
 	Serial.print(", V Frequency (Hz), ");
 	Serial.println(vWave.getFrequency());

 	Serial.print(", PowerFactor , ");
 	Serial.print(pWave.getPF());
 	Serial.print(", S (VA) , ");
 	Serial.print(pWave.getApparentP());
 	Serial.print(", P (W), ");
 	Serial.print(pWave.getRealP());
 	Serial.print(", Q (VAR) , ");
 	Serial.println(pWave.getReactiveP());
 }

 void printWaveSerial(int waveSize, waveform vWave)
 /*wiring code to print current time and data values of input waveform
 to serial monitor, printWaveSerial(int waveSize, waveform vWave)
 wave size is the integer length of the wave, vWave is the
 waveform object to be printed*/
 {
   for(int i = 0; i < waveSize; i++)
   {
     unsigned long iTime = vWave.printTime(i);
     double iData = vWave.printData(i);
     double t = iData;
     unsigned long t2 = iTime;
     unsigned long iPeak = vWave.printPeaks(i);
     unsigned long t3 = iPeak;
 		double t4 = vWave.printFilterData(i);
     Serial.print(t);
     Serial.print(" , ");
     Serial.print(t2);
     Serial.print(" , ");
     Serial.print(t3);
 		Serial.print(" , ");
 		Serial.println(t4);
     /*Serial.print(" , ");
     Serial.println(t4);*/
   }
 }


 void printVect(powerWave pWave)
 /*wiring code to print current time and data values of input waveform
 to serial monitor, printWaveSerial(int waveSize, waveform vWave)
 wave size is the integer length of the wave, vWave is the
 waveform object to be printed*/
 {
   Serial.println("Spectrum Start:, check, below ");
   for(int i = 0; i < pWave.getPDataSize(); ++i)
   {

     Serial.print(i);
     Serial.print(", ");
     Serial.println(pWave.getSpectrum(i));
   }
 	Serial.println("");

   Serial.println("Harmonics Start:, check, below ");
   for(int i = 0; i < 6; ++i)
   {

     Serial.print(i*60);
     Serial.print(", ");
     Serial.println(pWave.getHarmonic(i));
   }
 	Serial.println("");
 }
