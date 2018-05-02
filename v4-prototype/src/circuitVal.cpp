#include "circuitVal.h"

circuitVal::circuitVal(double deltaCurrent, unsigned long deltaTime ){
  /**
  set deltaCurrent to specify the sensitivity to changes, i.e.
  deltaCurrent=.01 will trigger a change event when there is a
  steady state change in current of .01 A RMS
  **/
  this->deltaTime = deltaTime;
  this->deltaCurrent = deltaCurrent;
  this->old_iRMS = 0;
  this->prevTime = 0;
  this->circuit_state = CHANGE_WAIT;

  for (int i = 0; i < this->_buff_size; i++) {
    this->vRMS_buff[i] = 0;
    this->iRMS_buff[i] = 0;
    this->pf_buff[i] = 0;
    this->apparentP_buff[i] = 0;
    this->realP_buff[i] = 0;
    this->reactiveP_buff[i] = 0;
    for (int j = 0; j < this->_num_harmonics; j++) {
      this->harmonics_buff[i][j] = 0;
      this->harmonics[j] = 0;
    }
  }

}
// src: https://stackoverflow.com/questions/2025372/c-macro-question-x-vs-x
template<typename T> inline const T abs(T const & x)
{
    return ( x<0 ) ? -x : x;
}

bool circuitVal::data_ready(){
  double deltaCurrent = this->deltaCurrent;
  double di = abs(this->iRMS - this->old_iRMS);
  unsigned long dt = millis() - this->prevTime;
  // Serial.println(di);
  // add delta time trigger back
  // trigger on changes <= .01
  // the small loads change with di .01 x2 or x3 times which is why
  // they aren't getting picked up
  switch(this->circuit_state){
    case CHANGE_WAIT:
      if(di > deltaCurrent){
        this->prevTime = millis();
        this->circuit_state = CHANGE_START;
        // this->circuit_state = CHANGE_WAIT;
      }
      // else {
      //   this->prevTime = millis();
      //   this->circuit_state = CHANGE_START;
      // }
      return false;
    case CHANGE_START:
      if(dt > this->deltaTime){
        this->old_iRMS = this->iRMS;
        // this->circuit_state = CHANGE_STOP;
        this->circuit_state = CHANGE_WAIT;
        return true;
      }
      return false;
    case CHANGE_STOP:
      if(di > deltaCurrent){
        this->circuit_state = CHANGE_WAIT;
      }
      return false;
    default:
      return false;
  }
}


String circuitVal::get_data_string(){
  String iS = String(this->iRMS, 3) + " ";
  String vS = String(this->vRMS, 2) + " ";
  String pfS = String(this->pf, 2) + " ";
  String sS = String(this->apparentP, 2) + " ";
  String pS = String(this->realP, 2) + " ";
  String qS = String(this->reactiveP, 2) + " ";
  String hS;
  for(size_t i = 0; i < this->_num_harmonics; i++)
  {
    hS += String(this->harmonics[i], 2) + " ";
  }
  return iS + vS + pfS + sS + pS + qS + hS;
}

double circuitVal::get_iRMS(){
  return this->iRMS;
}

double circuitVal::get_vRMS(){
  return this->vRMS;
}

double circuitVal::get_pf(){
  return this->pf;
}

double circuitVal::get_apparentP(){
  return this->apparentP;
}

double circuitVal::get_realP(){
  return this->realP;
}

double circuitVal::get_reactiveP(){
  return this->reactiveP;
}

float* circuitVal::get_harmonics(){
  return this->harmonics;
}

void circuitVal::addData(
  double iRMS,
  double vRMS,
  double pf,
  double apparentP,
  double realP,
  double reactiveP,
  float* harmonics){
    this->old_iRMS = this->iRMS;
    this->iRMS = (this->iRMS*this->_buff_size - this->iRMS_buff[this->ptr] + iRMS)/(double)this->_buff_size;
    this->iRMS_buff[this->ptr] = iRMS;

    this->vRMS = (this->vRMS*this->_buff_size - this->vRMS_buff[this->ptr] + vRMS)/(double)this->_buff_size;
    this->vRMS_buff[this->ptr] = vRMS;

    this->pf = (this->pf*this->_buff_size - this->pf_buff[this->ptr] + pf)/(double)this->_buff_size;
    this->pf_buff[this->ptr] = pf;

    this->apparentP = (this->apparentP*this->_buff_size - this->apparentP_buff[this->ptr] + apparentP)/(double)this->_buff_size;
    this->apparentP_buff[this->ptr] = apparentP;

    this->realP = (this->realP*this->_buff_size - this->realP_buff[this->ptr] + realP)/(double)this->_buff_size;
    this->realP_buff[this->ptr] = realP;

    this->reactiveP = (this->reactiveP*this->_buff_size - this->reactiveP_buff[this->ptr] + reactiveP)/(double)this->_buff_size;
    this->reactiveP_buff[this->ptr] = reactiveP;

    for (int i = 0; i < this->_num_harmonics; i++) {
      this->harmonics[i] = (this->harmonics[i]*this->_buff_size - this->harmonics_buff[this->ptr][i] + harmonics[i])/(double)this->_buff_size;
      this->harmonics_buff[this->ptr][i] = harmonics[i];
    }
    this->ptr = (this->ptr+1) % this->_buff_size;

}
