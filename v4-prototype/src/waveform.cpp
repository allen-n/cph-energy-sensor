#include "waveform.h"
// FIXME:
// #include "Particle.h"
/**
  initialize the wave object with the number of samples it will take
  as numPts and the size of its moving average filter kernal as
  filtKernalSize
  @param int numPts, specif
  @return
**/
waveform::waveform(int numPts, size_t filtKernalSize)
{
  this->_numPts = numPts;
  this->_datapoints.clear();
  this->_timeStamp.clear();
  this->_peakVect.clear();
  this->_datapoints.resize(numPts);
  this->_timeStamp.resize(numPts);
  this->_peakVect.resize(numPts);
  this->_posPointer = 0;
  this->_filtKernalSize = filtKernalSize + 1;
  this->_isFiltered = false;
}

void waveform::resetWave()
// reset pointer variable (_posPointer) back to 0 to write a new wave in
{
  this->_posPointer = 0;
  this->_maxVal = -5;
  this->_minVal = 5;
  this->_amplitude = NULL;
  this->_frequency = NULL;
  this->_average = NULL;
  this->_RMS = NULL;
}

unsigned long waveform::printTime(int i)
//print time value from vector at position i
{
  int vsize = this->_timeStamp.size();
  if(i <= vsize) return this->_timeStamp[i];
  else return -1;
}

double waveform::printData(int i)
//print data value from vector at position i
{
  int vsize = this->_datapoints.size();
  if(i <= vsize) return this->_datapoints[i];
  else return 0;
}

// double waveform::printFilterData(int i)
// //print data value from vector at position i
// {
//   int vsize = this->_filterDatapoints.size();
//   if(i <= vsize) return this->_filterDatapoints[i];
//   else return 0;
// }

unsigned long waveform::printPeaks(int i)
//print data value from vector at position i
//FIXME: peaks correct but not lining up with datapoints vector
{
  int vsize = this->_peakVect.size();
  if(i <= vsize) return this->_peakVect[i];
  else return -5;
}

void waveform::addData(unsigned long currentTime, double data)
// add a datapoint to the waveform in the for addData(time, data)
// and increment the position pointer in the waveform vector,
// will not overflow, implements circular buffering
{
  int pos = this->_posPointer;
  int numPts = this->_numPts;
  _datapoints[pos%numPts] = data;
  _timeStamp[pos%numPts] = currentTime;
  this->_posPointer++;
}

//FIXME: WRITE AVG FILTER FUNCTION http:
void waveform::movingAvgFilter()
{
  /*_filterDatapoints
  _datapoints*/
  this->_isFiltered = true;
  size_t len = this->_numPts;
  int kernSize = this->_filtKernalSize;
  int p = (kernSize-1)/2;
  int q = p+1;
  std::vector<double> y;
  std::vector<double>& x = this->_datapoints;
  y.resize(len);

  double acc = 0;
  for(size_t i = 0; i < kernSize; i++)
  {
    acc+=x[i];
  }

  y[p] = acc/kernSize;

  for(size_t i = p+1; i < len - p; i++)
  {
    acc = acc + x[i+p] - x[i-q];
    y[i] = acc / kernSize;
  }
  x = y;

}

double waveform::getRMS(double offset)
//returns RMS value of the waveform
{
  if(this->_RMS == NULL)
  {
    if(this->_amplitude == NULL) this->getAverage();
    double sum = 0;
    int numPts = this->_numPts;
    std::vector<double>& x = this->_datapoints;
    size_t index = (this->_filtKernalSize-1)/2;
    // if(this->_isFiltered) x = this->_filterDatapoints;
    //ignoring first and last point because they end up noisy
    for(int i = index; i < numPts - index; ++i)
    {
      double toAdd = (x[i] - this->_average);
      x[i] = toAdd; //NOTE: DC offset happens here
      sum+= toAdd*toAdd;
      if(toAdd > this->_maxVal) this->_maxVal = toAdd;
      if(toAdd < this->_minVal) this->_minVal = toAdd;
      // Serial.println(x[i]); //FIXME:
    }
    // Serial.println(); //FIXME:
    double temp = (sum/((double)numPts - index*2));
    this->_RMS = sqrt(temp);

    // NOTE: We are getting the amplitude of THE OTHER WAVEFORM
    double peak = (this->_maxVal)/2;
    for(int i = index + 1; i < numPts - index - 1; ++i)
    {
      if((x[i-1] <= x[i]) && (x[i+1] < x[i]) && (x[i] > peak)){
        this->_peakVect[i] = 1;
      } else {
        this->_peakVect[i] = 0;
      }
    }

  }
  return this->_RMS;
}

double waveform::getAverage()
//returns peak to peak value of the waveform
{
  if(this->_average == NULL)
  {
    double sum = 0;
    int numPts = this->_numPts;
    std::vector<double>& x = this->_datapoints;
    size_t index = (this->_filtKernalSize-1)/2;
    // if(x.size() == this->_filterDatapoints.size()) x = this->_filterDatapoints;
    for(int i = index; i < numPts - index; ++i)
    {
      double data = x[i];
      sum += data;
    }
    this->_average = sum/((double)numPts - index*2);
  }
  return this->_average;
}

/**
  Depreciated, don't use this.
**/
double waveform::getFrequency()
{

  if(this->_frequency == NULL)
  {
    if(this->_average == NULL) this->getAverage();

    int numPts = this->_numPts;
    size_t index = (this->_filtKernalSize-1)/2;
    std::vector<double>& x = this->_datapoints;
    // if(x.size() == this->_filterDatapoints.size()) x = this->_filterDatapoints;
    for(int i = index + 1; i < numPts - index - 2; ++i)
    {
      double prev = x[i-1];
      double current = x[i];
      double next = x[i+1];
      double next2 = x[i+2];
      double avg = this->_average;
      double peak = this->_average + this->getAverage()/2;
      if(prev > avg && next2 < avg && next <= avg && current >= avg) //TODO: keep fixing peak finder
      {
        this->_peakVect[i] = (this->_timeStamp[i] + this->_timeStamp[i+1])/2;
      }
      else
      {
        this->_peakVect[i] = 0;
      }
    }

    int counter = 0;
    double totalTime = 0;
    double nowTime = 0;
    double maxTime = 0;
    double prevTime = 0;
    /*unsigned long maxTime = 35791394;*/
    for(int i = 1; i < numPts - 1; ++i)
    {
      nowTime = this->_peakVect[i];
      if(nowTime > 0)
      {
        if(prevTime == 0)
        {
          prevTime = nowTime;
          continue;
        }
        else
        {
            maxTime = nowTime;
            counter++;
        }
      }
    }
    maxTime = (maxTime) / 1000000.0;
    prevTime = (prevTime) / 1000000.0;
    this->_frequency = (double)counter/((maxTime) - (prevTime));
  }

  return this->_frequency;
}

bool waveform::dataFull()
{
  if(this->_posPointer > _numPts) return true;
  else return false;
}
