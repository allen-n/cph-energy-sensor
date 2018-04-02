#ifndef __WAVEFORM_INCLUDED__
#define __WAVEFORM_INCLUDED__

#include <vector>
#include <math.h>
// #include <fstream>
// #include <string>
// #include <sstream>
// #include <algorithm>

class waveform
{
public:
  void addData(unsigned long currentTime, double data);
  double getRMS(double offset = 0);//get RMS value for waveform
  double getAmplitude();//get amplitude
  double getFrequency();//get frequency
  bool dataFull();
  void resetWave();
  unsigned long printTime(int i);
  double printData(int i);
  unsigned long printPeaks(int i);
  waveform(int numPts);
  int _numPts;
  double _RMS = NULL;
  std::vector<double> _datapoints;
  std::vector<double> _filterDatapoints;
  std::vector<unsigned long> _peakVect;
  size_t _filtKernalSize = 11;

  double getAverage();
  void movingAvgFilter();
  double printFilterData(int i);

protected:
  int _posPointer;
  double _maxVal = -5;
  double _minVal = 5;
  double _amplitude = NULL;
  double _frequency = NULL;
  double _average = NULL;
  std::vector<unsigned long> _timeStamp;
  /*
  src: http://www.drdobbs.com/cpp/a-simple-and-efficient-fft-
  implementatio/199500857?pgno=1
  classical implementation of the Cooley-Tukey algorithm from
  Numerical Recipes in C++ [5], p.513.

  The initial signal is stored in the array data of length 2*nn,
  where each even element corresponds to the real part and each
  odd element to the imaginary part of a complex number.
  */
  // void FFT(double* data, unsigned long nn);
  // in case we need more filtering
};

#endif
