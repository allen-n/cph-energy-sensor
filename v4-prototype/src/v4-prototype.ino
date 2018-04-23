/*
 * Project v4-prototype
 * Description: v4-prototype code
 * Author: Allen Nikka
 * Date: 4.1.2018
 */

// #include "ADS6838SR.h"
// // FIXME:
// // replace SparkIntervalTimer with:
// // https://docs.particle.io/reference/firmware/electron/#class-member-callbacks
//
// ads6838 MY_ADC;
// void setup(){
//   MY_ADC.init(20);
//   pinMode(D0, OUTPUT);
//   // digitalWrite(D0, HIGH);
//   pinMode(D1, OUTPUT);
//   // digitalWrite(D1, HIGH);
//   pinMode(D2, OUTPUT);
//   // digitalWrite(D2, HIGH);
//   pinMode(D3, OUTPUT);
//   // digitalWrite(D3, HIGH);
//   pinMode(D7, OUTPUT);
//   // digitalWrite(D7, HIGH);
//   delay(1000);
// }
//
// unsigned long ct = micros();
// const unsigned long dt = 2000;
//
// uint16_t out[8];
// void loop(){
//   digitalWrite(D7, LOW);
//   // Serial.println(micros());
//   if((micros() - ct) > dt){
//     MY_ADC.read8(out, ADS8638_REG_MANUAL, ADS8638_RANGE_5V);
//     Serial.println(out[1]);
//     ct = micros();
//   }
//   // runs at 5.2 kHz with all 8 channels and 20 MHz clk
//   // runs at 17.8 kHz for a single channel and 20MHz clk
//   // Serial.println(micros());
//   digitalWrite(D7, HIGH);
//   // delay(100);
// }


#include "waveform.h"
#include "powerWave.h"
#include "circuitVal.h"
#include "SparkIntervalTimer.h"
#include "ADS6838SR.h"

// Defining int constants cooresponding to ADS6838SR channels
#define ADS8638_VOLT1 0x0
#define ADS8638_CURR1 0x1 //100A Branch
#define ADS8638_CURR2 0x2 //100A Branch
#define ADS8638_CURR3 0x3
#define ADS8638_CURR4 0x4
#define ADS8638_CURR5 0x5
#define ADS8638_CURR6 0x6
#define ADS8638_VOLT2 0x7

// Setting sample rates for branch circuits and 2 mains
const size_t FILTER_KERNAL_SIZE = 2; //must be even
const size_t SAMPLE_BUF_SIZE =  512 + FILTER_KERNAL_SIZE; //these rates are doubled from base
// main #1
const long SAMPLE_RATE_VOLT1 = 1024;
const int SAMPLE_RATE_SHIFT_VOLT1 = log(SAMPLE_RATE_VOLT1)/log(2);
// main #2
const long SAMPLE_RATE_VOLT2 = 1024;
const int SAMPLE_RATE_SHIFT_VOLT2 = log(SAMPLE_RATE_VOLT2)/log(2);
// Branch circuits
const long SAMPLE_RATE_BRANCH = 8192;
const int SAMPLE_RATE_SHIFT_BRANCH = log(SAMPLE_RATE_BRANCH)/log(2);

// defining LED Constants //FIXME? don't these need to be defined?
// const int LED1 = D0;
// const int LED2 = D1;
// const int LED3 = D2;
// const int LED4 = D3;

// Configuring Serial output (SERIAL_DEBUG) or webhook output (!SERIAL_DEBUG)
const bool SERIAL_DEBUG = true;

// # of circuits being monitored, i.e. # ADC Channels - # mains voltages
const uint8_t NUM_CIRCUITS = 6;

// states for data processing in main loop()
enum State { STATE_INIT = 0, STATE_COLLECT, STATE_TRANSFER, STATE_PROCESS };

// states for active circuit being monitored
enum ACTIVE_CIRCUIT { CURR1 = 0, CURR2 = 1, CURR3 = 2, CURR4 = 3, CURR5 = 4, CURR6 = 5 };

// states for active timer used in the main loop
enum ACTIVE_TIMER { VOLT1_TIMER = 0, VOLT2_TIMER, BRANCH_TIMER };

// struct to hold samples gathered using timerISR() routines
typedef struct {
	volatile bool free;
	volatile size_t  index;
	double v_data[SAMPLE_BUF_SIZE];
	double i_data[SAMPLE_BUF_SIZE];
	// unsigned long t_data[SAMPLE_BUF_SIZE];
} SampleBuf;

//NOTE: 20504 free memory
// Object Instatiation:
IntervalTimer volt1_timer;
IntervalTimer volt2_timer;
IntervalTimer branch_timer;
SampleBuf samples[NUM_CIRCUITS];
circuitVal circuit[NUM_CIRCUITS]; //circuit1(0.02, 1500); //NOTE: occupy 1.192 kB of mem
ads6838 MY_ADC;
State state = STATE_INIT;
ACTIVE_CIRCUIT circuit_state = CURR1;
uint8_t isrBranchCurrent;
uint8_t isrBranchVoltage;
ACTIVE_TIMER timer_state = VOLT1_TIMER;

const int waveSize = SAMPLE_BUF_SIZE;
waveform vWave(waveSize, FILTER_KERNAL_SIZE); //NOTE: Occupies 6.28 kB of mem
waveform iWave(waveSize, FILTER_KERNAL_SIZE);
powerWave pWave_VOLT1(waveSize, SAMPLE_RATE_VOLT1); //NOTE: Occupies 128 kB of mem
powerWave pWave_VOLT2(waveSize, SAMPLE_RATE_VOLT2);
powerWave pWave_BRANCH(waveSize, SAMPLE_RATE_BRANCH);

// Helper Functions
void dataHandler(const char *event, const char *data)
{
  // if(strcmp("setRelay1",event)==0)
  // {
  //   relayHandler(relay1, data);
  // }
  if(strcmp("sendEvent",event)==0)
  {
    sendEvent();
  }
}

// void relayHandler(int relayPin, const char *data)
// {
//   if(strcmp(data, "OFF")==0) digitalWrite(relayPin, LOW);
//   if(strcmp(data, "ON")==0) digitalWrite(relayPin, HIGH);
// }

bool pushDataFlag[NUM_CIRCUITS] = {false, false, false, false, false, false};
void sendEvent()
{
  pushDataFlag[circuit_state] = true;
}

unsigned long sendInterval;
const unsigned long sendIntervalDelta = 20*3600; // 20 minutes

void pushData(circuitVal& c1, bool& pushDataFlag)
{
  if((millis() - sendInterval) > sendIntervalDelta > 0 && !SERIAL_DEBUG)
  {
    String outStr = c1.get_data_string();
    Particle.publish("send-i,v,pf,s,p,q", outStr, 5, PRIVATE);
  }
  if(pushDataFlag && !SERIAL_DEBUG)
  {
    pushDataFlag = false;
    String outStr = c1.get_data_string();
    Particle.publish("send-i,v,pf,s,p,q", outStr, 5, PRIVATE);
  }
}

void logCircuit(waveform& iWave, waveform& vWave, powerWave& pWave, circuitVal& c1){
  double iRMS = iWave.getRMS();
  double vRMS = vWave.getRMS();
  double pf = pWave.getPF();
  double apparentP = pWave.getApparentP();
  double realP = pWave.getRealP();
  double reactiveP = pWave.getReactiveP();
  size_t numHarmoics = pWave.getNumHarmoics();
  float harmonics[numHarmoics];
  for(size_t i = 0; i < numHarmoics; i++)
  {
    harmonics[i] = pWave.getHarmonicAngle(i);
  }
  c1.addData(iRMS, vRMS, pf, apparentP, realP, reactiveP, harmonics);
  String out = c1.get_data_string();
  // Serial.println(out); //NOTE: Continuous Serial Debug Option, uncomment
	Serial.println(System.freeMemory());Serial.print(" , "); //FIXME
  if(c1.data_ready()){
    digitalWrite(D7, HIGH);
    if(SERIAL_DEBUG){
      // Serial.print(System.freeMemory());Serial.print(" , "); //FIXME
      Serial.println(out);
    } else {
      sendInterval = millis();
      Particle.publish("send-i,v,pf,s,p,q", out, 5, PRIVATE);
    }
    digitalWrite(D7, LOW);
  }
}

double v_ratio = (3.3 * 100000) / 4095;
double i_ratio = (5 * 100000) / (4095 * 10);
// vref * 100000 / (adc counts * turns_ratio/(burdenR*1000))
void transferBuff(SampleBuf& buff, bool& outFlag, waveform& vWave, waveform& iWave){
  for(size_t i = 0; i < SAMPLE_BUF_SIZE; i++)
  {
		// unsigned long time_val = buff.t_data[i];
		//all values now in mV

		//values without ratios above are in mV detected by microcontroller
		double i_val = i_ratio * buff.i_data[i];
		double v_val = v_ratio * buff.v_data[i];
		iWave.addData(floorf(i_val)/100);
		vWave.addData(floorf(v_val)/100);
    // FIXME:
    // Serial.print(i);Serial.print(" , ");Serial.print(time_val);Serial.print(" , ");
    // Serial.println(buff.i_data[i]*1000/4095.0);
	}
  // Serial.println();
  outFlag = true;
  // Serial.println("");
}


void timerLoop(powerWave& pWave, ACTIVE_CIRCUIT& circuit_state, bool& outFlag,
	circuitVal& circuit, ACTIVE_TIMER& timer_state){
	switch(state)
	{
		case STATE_INIT:
  		// Reset the buffers
      // TODO: Check the oferall runtime without any outputs using the follwing
      // Serial.println(micros());
      samples[circuit_state].free = true;
      samples[circuit_state].index = 0;
			outFlag = false;
			// We want to sample at 16 KHz
			// 16000 samples/sec = 62.5 microseconds
			// The minimum timer period is about 10 micrseconds
			state = STATE_COLLECT;
			break;
		case STATE_COLLECT:
      if(!(samples[circuit_state].free)) {
        state = STATE_TRANSFER;
      }
			break;
		case STATE_TRANSFER:
      transferBuff(samples[circuit_state], outFlag, vWave, iWave);
			state = STATE_PROCESS;
			if(outFlag) state = STATE_PROCESS;
		break;
		case STATE_PROCESS:
			outFlag = false;

			iWave.movingAvgFilter();
			vWave.movingAvgFilter();
			iWave.getRMS();
			vWave.getRMS();
			pWave.calcP();
			pWave.computeFFT();
      logCircuit(iWave, vWave, pWave, circuit);
      pushData(circuit, pushDataFlag[circuit_state]);
			pWave.clearWave();
			vWave.resetWave();
			iWave.resetWave();
			if(SERIAL_DEBUG) delay(15);
			state = STATE_INIT;
			break;
		default:
			//default to dump buffers and start over
			volt1_timer.begin(timerISR_VOLT1, 1000000 >> SAMPLE_RATE_SHIFT_VOLT1, uSec); //122 uSec sample time
			volt2_timer.begin(timerISR_VOLT2, 1000000 >> SAMPLE_RATE_SHIFT_VOLT2, uSec); //122 uSec sample time
			branch_timer.begin(timerISR_BRANCH, 1000000 >> SAMPLE_RATE_SHIFT_BRANCH, uSec); //122 uSec sample time
			state = STATE_INIT;
			break;
	}
}

void setup() {
	// Put initialization like pinMode and begin functions here.
  Serial.begin(9600);
  pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);
	pinMode(LED4, OUTPUT);

	digitalWrite(LED1, HIGH);
  Particle.subscribe("setRelay", dataHandler);
	digitalWrite(LED2, HIGH);
  Particle.subscribe("sendEvent", dataHandler);
	digitalWrite(LED3, HIGH);

	pWave_VOLT1.addComponents(vWave, iWave);
	pWave_VOLT2.addComponents(vWave, iWave);
	pWave_BRANCH.addComponents(vWave, iWave);
	digitalWrite(LED4, HIGH);

	sendInterval = millis();
	delay(250);
	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);
	digitalWrite(LED3, LOW);
	digitalWrite(LED4, HIGH);
	volt1_timer.begin(timerISR_VOLT1, 1000000 >> SAMPLE_RATE_SHIFT_VOLT1, uSec); //122 uSec sample time
	volt2_timer.begin(timerISR_VOLT2, 1000000 >> SAMPLE_RATE_SHIFT_VOLT2, uSec); //122 uSec sample time
	branch_timer.begin(timerISR_BRANCH, 1000000 >> SAMPLE_RATE_SHIFT_BRANCH, uSec); //122 uSec sample time
	delay(250);

}

bool outFlag[NUM_CIRCUITS] = {false, false, false, false, false};
// ACTIVE_CIRCUIT circuit_state = CURR1;
// uint8_t isrBranchCurrent;
// uint8_t isrBranchVoltage;
void loop() {
	timerLoop(pWave_VOLT1, CURR1, outFlag[CURR1], circuit[CURR1]);
	timerLoop(pWave_VOLT2, CURR2, outFlag[CURR2], circuit[CURR2]);
	timerLoop(pWave_BRANCH, circuit_state, outFlag[circuit_state], circuit[circuit_state]);
}


// this timerISR is set to monitor the VOLT2 main
void timerISR_VOLT1() {
	// This is an interrupt service routine. Don't put any heavy calculations here
	// or call anything that's not interrupt-safe, such as:
	// Serial, String, any memory allocation (new, malloc, etc.), Particle.publish and other Particle methods, and more.
  SampleBuf *sb = &samples[CURR1];
  if(sb->free){
    sb->i_data[sb->index] = MY_ADC.read1(ADS8638_CURR1);
		sb->v_data[sb->index] = MY_ADC.read1(ADS8638_VOLT1);
		// sb->t_data[sb->index] = micros();
		sb->index++;
  }
  if (sb->index >= SAMPLE_BUF_SIZE) {
    // Buffer has been filled. Let it be sent via TCP.
    sb->free = false;
    sb->index = 0;
  }
}

void timerISR_VOLT2() {
	// This is an interrupt service routine. Don't put any heavy calculations here
	// or call anything that's not interrupt-safe, such as:
	// Serial, String, any memory allocation (new, malloc, etc.), Particle.publish and other Particle methods, and more.
  SampleBuf *sb = &samples[CURR2];
  if(sb->free){
    sb->i_data[sb->index] = MY_ADC.read1(ADS8638_CURR2);
		sb->v_data[sb->index] = MY_ADC.read1(ADS8638_VOLT2);
		// sb->t_data[sb->index] = micros();
		sb->index++;
  }
  if (sb->index >= SAMPLE_BUF_SIZE) {
    // Buffer has been filled. Let it be sent via TCP.
    sb->free = false;
    sb->index = 0;
  }
}

void timerISR_BRANCH() {
	// This is an interrupt service routine. Don't put any heavy calculations here
	// or call anything that's not interrupt-safe, such as:
	// Serial, String, any memory allocation (new, malloc, etc.), Particle.publish and other Particle methods, and more.
  SampleBuf *sb = &samples[circuit_state];
  if(sb->free){
    sb->i_data[sb->index] = MY_ADC.read1(isrBranchCurrent);
		sb->v_data[sb->index] = MY_ADC.read1(isrBranchVoltage);
		// sb->t_data[sb->index] = micros();
		sb->index++;
  }
  if (sb->index >= SAMPLE_BUF_SIZE) {
    // Buffer has been filled. Let it be sent via TCP.
    sb->free = false;
    sb->index = 0;
  }
}
