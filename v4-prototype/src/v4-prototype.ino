/*
 * Project v4-prototype
 * Description: v4-prototype code
 * Author: Allen Nikka
 * Date: 4.1.2018
 */

#include "waveform.h"
#include "powerWave.h"
#include "circuitVal.h"
#include "SparkIntervalTimer.h"
#include "ADS6838SR.h"

// Defining int constants cooresponding to ADS6838SR channels
const uint8_t ADS8638_VOLT1 = 0x0;
const uint8_t ADS8638_CURR1 = 0x1;
const uint8_t ADS8638_CURR2 = 0x2;
const uint8_t ADS8638_CURR3 = 0x3;
const uint8_t ADS8638_CURR4 = 0x4;
const uint8_t ADS8638_CURR5 = 0x5; //100A Branch
const uint8_t ADS8638_CURR6 = 0x6; //100A Branch
const uint8_t ADS8638_VOLT2 = 0x7;

// Setting sample rates for branch circuits and 2 mains
const size_t FILTER_KERNAL_SIZE = 8; //must be even
const size_t SAMPLE_BUF_SIZE =  512 + FILTER_KERNAL_SIZE; //these rates are doubled from base
// main #1
// const long SAMPLE_RATE_VOLT1 = 1024; //FIXME
const long SAMPLE_RATE_VOLT1 = 4096;
const int SAMPLE_RATE_SHIFT_VOLT1 = log(SAMPLE_RATE_VOLT1)/log(2);
// main #2
// const long SAMPLE_RATE_VOLT2 = 1024;
const long SAMPLE_RATE_VOLT2 = 4096;
const int SAMPLE_RATE_SHIFT_VOLT2 = log(SAMPLE_RATE_VOLT2)/log(2);
// Branch circuits
// const long SAMPLE_RATE_BRANCH = 8192;
const long SAMPLE_RATE_BRANCH = 4096;
const int SAMPLE_RATE_SHIFT_BRANCH = log(SAMPLE_RATE_BRANCH)/log(2);

// defining LED Constants
#define LED1 D0
#define LED2 D1
#define LED3 D3
#define LED4 D4

// Particle Wi-Fi and startup initializations, ANT_INTERNAL for internal
STARTUP(WiFi.selectAntenna(ANT_EXTERNAL)); // selects the u.FL antenna,
// STARTUP(WiFi.selectAntenna(ANT_INTERNAL)); // selects the internal antenna,

// Configuring Serial output (SERIAL_DEBUG) or webhook output (!SERIAL_DEBUG)
const bool SERIAL_DEBUG = false;

// Configuring antenna
const bool EXT_ANTENNA = true;

// # of circuits being monitored, i.e. # ADC Channels - # mains voltages
const uint8_t NUM_CIRCUITS = 6;

// # of branch circuits being monitored, i.e. NUM_CIRCUITS - # mains currents
const int NUM_BRANCH_CIRCUITS = NUM_CIRCUITS - 2;

// defining pairings of voltage readings with their cooresponding current

uint8_t CIRCUIT_PAIR[NUM_CIRCUITS] = {
	ADS8638_VOLT2, ADS8638_VOLT2, ADS8638_VOLT2,
	ADS8638_VOLT2, ADS8638_VOLT2, ADS8638_VOLT2
};

// states for data processing in main loop()
enum State { STATE_INIT = 0, STATE_COLLECT, STATE_TRANSFER, STATE_PROCESS };

// states for active circuit being monitored
// enum ACTIVE_CIRCUIT { MAIN1 = (int)ADS8638_CURR5, MAIN2 = (int)ADS8638_CURR6, BRANCH = (int)ADS8638_CURR1};
enum ACTIVE_CIRCUIT { MAIN1 = 0, MAIN2 = 1, BRANCH = 2};

// states for active timer used in the main loop
enum ACTIVE_TIMER { VOLT1_TIMER = 0, VOLT2_TIMER, BRANCH_TIMER };

// struct to hold samples gathered using timerISR() routines
typedef struct {
	volatile bool free;
	volatile size_t  index;
	uint16_t v_data[SAMPLE_BUF_SIZE];
	uint16_t i_data[SAMPLE_BUF_SIZE];
	unsigned long t_data[SAMPLE_BUF_SIZE];
} SampleBuf;

//NOTE: 20504 free memory
// Object Instatiation:
IntervalTimer volt1_timer;
IntervalTimer volt2_timer;
IntervalTimer branch_timer;

SampleBuf samples_curr1;
SampleBuf samples_curr2;
SampleBuf samples_branch;
SampleBuf* samples[NUM_CIRCUITS] = {&samples_curr1, &samples_curr2,
	&samples_branch, &samples_branch, &samples_branch, &samples_branch};

circuitVal circuit[NUM_CIRCUITS]; //circuit1(0.02, 1500); //NOTE: occupy 1.192 kB of mem

ads6838 MY_ADC;

State state_volt1 = STATE_INIT;
State state_volt2 = STATE_INIT;
State state_branch = STATE_INIT;

ACTIVE_CIRCUIT circuit_state = BRANCH;
ACTIVE_CIRCUIT circuit_state_curr1 = MAIN1;
ACTIVE_CIRCUIT circuit_state_curr2 = MAIN2;
uint8_t isrBranchCurrent = ADS8638_CURR1;
uint8_t isrBranchVoltage = ADS8638_VOLT1;

const int waveSize = SAMPLE_BUF_SIZE;
// TODO: Move to address memory allocation problem
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
	for (int i = 0; i < NUM_CIRCUITS; i++) {
		pushDataFlag[i] = true;
	}
  // pushDataFlag[circuit_state] = true;
}

unsigned long sendInterval;
const unsigned long sendIntervalDelta = 20*3600; // 20 minutes
// int circuit = (int)circuit_state + isrBranchCurrent;
// String out = String(circuit) + " "; //indicating which circuit is sending the data
// out += c1.get_data_string();
void pushData(circuitVal& c1, bool& pushDataFlag)
{
	int circuit = (int)circuit_state;
	if(circuit == 2) circuit += isrBranchCurrent;
	String out = String(circuit) + " "; //indicating which circuit is sending the data
  if((millis() - sendInterval) > sendIntervalDelta > 0 && !SERIAL_DEBUG)
  {
    String outStr = out + c1.get_data_string();
    Particle.publish("send-i,v,pf,s,p,q", outStr, 5, PRIVATE);
  }
  if(pushDataFlag && !SERIAL_DEBUG)
  {
    pushDataFlag = false;
    String outStr = out + c1.get_data_string();
		digitalWrite(LED1, HIGH);
    Particle.publish("send-i,v,pf,s,p,q", outStr, 5, PRIVATE);
		digitalWrite(LED1, LOW);
  }
}

void logCircuit(waveform& iWave, waveform& vWave, powerWave& pWave,
	circuitVal& c1, ACTIVE_CIRCUIT& circuit_state){
  double iRMS = iWave.getRMS();
  double vRMS = vWave.getRMS();
  double pf = pWave.getPF(); //FIXME: This is just a workaround!
  double apparentP = pWave.getApparentP();
  double realP = pWave.getRealP(); //FIXME: This is just a workaround!
  double reactiveP = pWave.getReactiveP();
  size_t numHarmoics = pWave.getNumHarmoics();
  float harmonics[numHarmoics];
  for(size_t i = 0; i < numHarmoics; i++)
  {
    harmonics[i] = pWave.getHarmonic(i);
  }
  c1.addData(iRMS, vRMS, pf, apparentP, realP, reactiveP, harmonics);
	int circuit = (int)circuit_state;
	if(circuit == 2) circuit += isrBranchCurrent - 1;
	String out = String(circuit + 1) + " "; //indicating which circuit is sending the data
	out += c1.get_data_string();
	// out += c1.data_ready();
  // Serial.println(out); //NOTE: Continuous Serial Debug Option, uncomment
  if(c1.data_ready()){
    digitalWrite(D7, HIGH);
    if(SERIAL_DEBUG){
      Serial.println(out);
    } else {
      sendInterval = millis();
      Particle.publish("send-i,v,pf,s,p,q", out, 5, PRIVATE);
    }
    digitalWrite(D7, LOW);
		c1.reset();
  }
}

const double v_divider_ratio = 100/4.0;
const double i_mains_ratio = 1000/29.5;
const double i_branch_ratio = 1000/41.0;

double v_ratio = 100;
double i_ratio = 100;
// vref * 100000 / (adc counts * turns_ratio/(burdenR*1000))
// ADC counts / 30 valid for 100A CT with +-2.5V ADC 12 bit ADC range
void transferBuff(SampleBuf& buff, bool& outFlag, waveform& vWave, waveform& iWave){
  for(size_t i = 0; i < SAMPLE_BUF_SIZE; i++)
  {
		// unsigned long time_val = buff.t_data[i];
		//all values now in mV

		//values without ratios above are in mV detected by microcontroller
		double i_val = (i_ratio * buff.i_data[i]); //account for CT offset
		double v_val = v_ratio * buff.v_data[i];
		iWave.addData(floorf(i_val)/100);
		vWave.addData(floorf(v_val)/100);
    // FIXME:
		if(SERIAL_DEBUG)
		{
			Serial.print(floorf(i_val)/100);
			Serial.print(" , ");
			Serial.print(floorf(v_val)/100);
			Serial.println(" , ");
		}

    if(SERIAL_DEBUG) Serial.println("New Measurement");
	}
  // Serial.println();
  outFlag = true;
  // Serial.println("");
}

void timerLoop(State& state, powerWave& pWave, ACTIVE_CIRCUIT& circuit_state, bool& outFlag,
	circuitVal& circuit){
	// Serial.print("Started timerLoop() for c");Serial.println(circuit_state);
	switch(state)
	{
		case STATE_INIT:
			switch(circuit_state){
				case MAIN1:
					volt1_timer.begin(timerISR_VOLT1, 1000000 >> SAMPLE_RATE_SHIFT_VOLT1, uSec, AUTO); //122 uSec sample time
					break;
				case MAIN2:
					volt2_timer.begin(timerISR_VOLT2, 1000000 >> SAMPLE_RATE_SHIFT_VOLT2, uSec, AUTO); //122 uSec sample time
					break;
				default:
					//pick next current branch, current branch values start at ADS8638_CURR1
					isrBranchCurrent-= ADS8638_CURR1;
					isrBranchCurrent = ((isrBranchCurrent + 1) % (NUM_BRANCH_CIRCUITS));

					isrBranchCurrent+= ADS8638_CURR1;
					// isrBranchVoltage = CIRCUIT_PAIR[isrBranchCurrent - ADS8638_CURR1];
					branch_timer.begin(timerISR_BRANCH, 1000000 >> SAMPLE_RATE_SHIFT_BRANCH, uSec, AUTO); //122 uSec sample time
					break;
			}
      samples[circuit_state]->free = true;
      samples[circuit_state]->index = 0;
			outFlag = false;
			// Serial.println("INIT0");Serial.println(System.freeMemory());
			// volt1_timer.begin(timerISR_VOLT1, 1000000 >> SAMPLE_RATE_SHIFT_VOLT1, uSec, AUTO); //122 uSec sample time
			// Serial.println("INIT1");Serial.println(System.freeMemory());
			state = STATE_COLLECT;
			break;
		case STATE_COLLECT:
      if(!(samples[circuit_state]->free)) {
        state = STATE_PROCESS;
				switch(circuit_state){
					case MAIN1:
						volt1_timer.end();
						break;
					case MAIN2:
						volt2_timer.end();
						break;
					default:
						branch_timer.end();
						break;
				}
      }
			// Serial.println("COLLECT");Serial.println(System.freeMemory());
			break;
		case STATE_TRANSFER:
      // transferBuff(*(samples[circuit_state]), outFlag, vWave, iWave);
			// Serial.println("TRANSFER");Serial.println(System.freeMemory());
			state = STATE_PROCESS;
			// if(outFlag) state = STATE_PROCESS;
			break;
		case STATE_PROCESS:
			outFlag = false;
			// FIXME: add different ratios for different circuits
			switch(circuit_state){ //once these values are finalized, put the final value to save computation time
				case MAIN1:
					v_ratio = v_divider_ratio;
					i_ratio = i_mains_ratio;
					break;
				case MAIN2:
					v_ratio = v_divider_ratio;
					i_ratio = i_mains_ratio;
					break;
				default:
					v_ratio = v_divider_ratio;
					i_ratio = i_branch_ratio;
					break;
			}
			transferBuff(*(samples[circuit_state]), outFlag, vWave, iWave);
			// Serial.print("Logging circuit for c");Serial.println(circuit_state);
			iWave.movingAvgFilter();
			vWave.movingAvgFilter();
			iWave.getRMS();
			vWave.getRMS();
			pWave.calcP();
			if(pWave.getPF() <= 0){
				int to_switch = circuit_state;
				if(to_switch == BRANCH) to_switch+= isrBranchCurrent - ADS8638_CURR1;
				swapCircuit(to_switch); //this is the wrong phase voltage, swap them
				// samples[circuit_state]->free = true;
	      // samples[circuit_state]->index = 0;
				// outFlag = false;
				// state = STATE_COLLECT;
				// break;
			}
			pWave.trimData();
			pWave.computeFFT();
      logCircuit(iWave, vWave, pWave, circuit, circuit_state);
      pushData(circuit, pushDataFlag[circuit_state]);
			pWave.clearWave();
			vWave.resetWave();
			iWave.resetWave();
			// FIXME

			if(SERIAL_DEBUG) delay(15);
			state = STATE_INIT;
			// Serial.println("PROCESS");Serial.println(System.freeMemory());
			break;
		default:
			//default to dump buffers and start over
			volt1_timer.end();
			volt2_timer.end();
			branch_timer.end();
			state = STATE_INIT;
			break;
	}
}

void setup() {
	// Put initialization like pinMode and begin functions here.
  if(SERIAL_DEBUG) Serial.begin(9600);
  pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);
	pinMode(LED4, OUTPUT);

	digitalWrite(LED1, HIGH);
  // Particle.subscribe("setRelay", dataHandler);
	digitalWrite(LED2, HIGH);
  Particle.subscribe("sendEvent", dataHandler);
	digitalWrite(LED3, HIGH);
	MY_ADC.init(20);
	delay(250);

	// TODO: Move to address memory allocation problem
	pWave_VOLT1.addComponents(vWave, iWave);
	pWave_VOLT2.addComponents(vWave, iWave);
	pWave_BRANCH.addComponents(vWave, iWave);
	digitalWrite(LED4, HIGH);

	delay(500);
	digitalWrite(LED1, LOW);
	digitalWrite(LED2, LOW);
	digitalWrite(LED3, LOW);
	digitalWrite(LED4, LOW);
	sendInterval = millis();
	delay(500);
	digitalWrite(LED1, HIGH);

}

bool outFlag[NUM_CIRCUITS] = {false, false, false, false, false};
// ACTIVE_CIRCUIT circuit_state = MAIN1;
// uint8_t isrBranchCurrent;
// uint8_t isrBranchVoltage;

void loop() {
	// Serial.println(System.freeMemory());Serial.print(" , "); //FIXME
	digitalWrite(LED2, HIGH);
	timerLoop(state_volt1, pWave_VOLT1, circuit_state_curr1, outFlag[circuit_state_curr1], circuit[circuit_state_curr1]);
	timerLoop(state_volt2, pWave_VOLT2, circuit_state_curr2, outFlag[circuit_state_curr2], circuit[circuit_state_curr2]);
	timerLoop(state_branch, pWave_BRANCH, circuit_state, outFlag[circuit_state], circuit[circuit_state]);
	digitalWrite(LED2, LOW);

	// Serial.print((int)MY_ADC.read1(ADS8638_CURR1));Serial.print(",");
	// Serial.print((int)MY_ADC.read1(ADS8638_CURR2));Serial.print(",");
	// Serial.print((int)MY_ADC.read1(ADS8638_CURR3));Serial.print(",");
	// Serial.print((int)MY_ADC.read1(ADS8638_CURR4));Serial.print(",");
	// Serial.print((int)MY_ADC.read1(ADS8638_CURR5));Serial.print(",");
	// Serial.print((int)MY_ADC.read1(ADS8638_CURR6));Serial.print(",");
	// Serial.print((int)MY_ADC.read1(ADS8638_VOLT1));Serial.print(",");
	// Serial.print((int)MY_ADC.read1(ADS8638_VOLT2));Serial.println(",");
	// Serial.print("state_volt1: ");Serial.print((int)MY_ADC.read1(ADS8638_CURR5));
	// Serial.print(" state_volt2: ");Serial.print((int)MY_ADC.read1(ADS8638_CURR6));
	// Serial.print(" state_branch: ");Serial.println((int)MY_ADC.read1(ADS8638_CURR3));
}

void swapCircuit(uint8_t position){
	digitalWrite(LED3, HIGH);
	if(CIRCUIT_PAIR[position] == ADS8638_VOLT1){
		CIRCUIT_PAIR[position] = ADS8638_VOLT2;
	}
	else {
		CIRCUIT_PAIR[position] = ADS8638_VOLT1;
	}
	digitalWrite(LED3, LOW);
}

// SampleBuf* curr1_samplebuff = &samples;
// this timerISR is set to monitor the VOLT2 main
void timerISR_VOLT1(void) {
	// This is an interrupt service routine. Don't put any heavy calculations here
	// or call anything that's not interrupt-safe, such as:
	// Serial, String, any memory allocation (new, malloc, etc.), Particle.publish and other Particle methods, and more.
	SampleBuf *sb = &samples_curr1;
  if(sb->free){
    sb->i_data[sb->index] = MY_ADC.read1(ADS8638_CURR5);
		// sb->v_data[sb->index] = MY_ADC.read1(ADS8638_VOLT2); //account for sample speed difference
		sb->v_data[sb->index] = MY_ADC.read1(CIRCUIT_PAIR[MAIN1]);
		sb->t_data[sb->index] = micros();
		sb->index++;
  }
  if (sb->index >= SAMPLE_BUF_SIZE) {
    // Buffer has been filled. Let it be sent via TCP.
    sb->free = false;
    sb->index = 0;
  }
}


void timerISR_VOLT2(void) {
	// This is an interrupt service routine. Don't put any heavy calculations here
	// or call anything that's not interrupt-safe, such as:
	// Serial, String, any memory allocation (new, malloc, etc.), Particle.publish and other Particle methods, and more.
  SampleBuf *sb = &samples_curr2;
  if(sb->free){
    sb->i_data[sb->index] = MY_ADC.read1(ADS8638_CURR6);
		// sb->v_data[sb->index] = MY_ADC.read1(ADS8638_VOLT2); //account for sample speed difference
		sb->v_data[sb->index] = MY_ADC.read1(CIRCUIT_PAIR[MAIN2]);
		sb->t_data[sb->index] = micros();
		sb->index++;
  }
  if (sb->index >= SAMPLE_BUF_SIZE) {

    // Buffer has been filled. Let it be sent via TCP.
    sb->free = false;
    sb->index = 0;
  }
}

void timerISR_BRANCH(void) {
	// This is an interrupt service routine. Don't put any heavy calculations here
	// or call anything that's not interrupt-safe, such as:
	// Serial, String, any memory allocation (new, malloc, etc.), Particle.publish and other Particle methods, and more.
	SampleBuf *sb = &samples_branch;
  if(sb->free){
    sb->i_data[sb->index] = MY_ADC.read1(isrBranchCurrent);
		// sb->v_data[sb->index] = MY_ADC.read1(ADS8638_VOLT2); //account for sample speed difference
		sb->v_data[sb->index] = MY_ADC.read1(CIRCUIT_PAIR[BRANCH + isrBranchCurrent - ADS8638_CURR1]); //account for sample speed difference
		sb->t_data[sb->index] = micros();



		sb->index++;
  }
  if (sb->index >= SAMPLE_BUF_SIZE) {
    // Buffer has been filled. Let it be sent via TCP.
    sb->free = false;
    sb->index = 0;
  }
}
