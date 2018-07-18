#ifndef __CIRCUITVAL_INCLUDED__
#define __CIRCUITVAL_INCLUDED__

#include "Particle.h"
// #include <string>
// #include <sstream>

class circuitVal {
public:
  circuitVal(double deltaCurrent = 0.042, unsigned long deltaTime = 2000);
  // NOTE: set to differentiate between loads that
  // have at least 40 mA difference over time periods of 1.
  // lowering current threshold reduces number of reads, but
  // increases the variability between readings
  void addData(double iRMS, double vRMS, double pf, double apparentP,
               double realP, double reactiveP, float *harmonics);

  double get_iRMS();
  double get_vRMS();
  double get_pf();
  double get_apparentP();
  double get_realP();
  double get_reactiveP();
  float *get_harmonics();
  bool data_ready();
  String get_data_string();
  void reset();

protected:
  static const int _buff_size = 25;
  static const int _num_harmonics = 6;
  int ptr = 0;
  double vRMS_buff[_buff_size];
  double iRMS_buff[_buff_size];
  double pf_buff[_buff_size];
  double apparentP_buff[_buff_size];
  double realP_buff[_buff_size];
  double reactiveP_buff[_buff_size];
  float harmonics_buff[_buff_size][_num_harmonics];

  double vRMS = 0;
  double iRMS = 0;
  double pf = 0;
  double apparentP = 0;
  double realP = 0;
  double reactiveP = 0;
  float harmonics[_num_harmonics];

  double deltaCurrent;
  double old_iRMS;
  double prev_read;
  enum State { CHANGE_START, CHANGE_STOP, CHANGE_WAIT };
  State circuit_state;
  unsigned long deltaTime;
  unsigned long prevTime;
  bool is_ready;
};

#endif
