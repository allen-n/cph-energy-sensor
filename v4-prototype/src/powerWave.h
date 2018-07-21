#ifndef __POWERWAVE_INCLUDED__
#define __POWERWAVE_INCLUDED__

#include "waveform.h"
// #include <vector>
// #include <math.h>
// #include <algorithm>
// #include <fstream>
// #include <string>
// #include <sstream>
// #include "Particle.h" //FIXME

class powerWave {
public:
  void computeFFT();
  // powerWave(waveform& vWave, waveform& iWave);
  powerWave(int numPts, int SAMPLE_RATE);
  void calcP();
  double getApparentP();
  double getRealP();
  double getReactiveP();
  double getPF();
  int getPDataSize();
  double getSpectrum(int i);
  void clearWave();
  void addComponents(waveform &vWave, waveform &iWave);
  float getHarmonic(int i);
  float getHarmonicAngle(int i);
  size_t getNumHarmoics();
  void trimData();

  size_t _errVal;

protected:
  waveform *_vWave = NULL;
  waveform *_iWave = NULL;
  double _apparentP = NULL;
  double _realP = NULL;
  double _reactiveP = NULL;
  double _PF = NULL;
  std::vector<float> _pData;
  // std::vector<float> _pDataAngle;
  int _SAMPLE_RATE;
  int _pDataSize = NULL;
  void FFT(float data[], unsigned long number_of_complex_samples, int isign);
  static const size_t _numHarmonics = 6; // was 8, changed to 6 to match
                                         // database
  float _harmonics[_numHarmonics];
  float _harmonicsAngle[_numHarmonics];
};

#endif
