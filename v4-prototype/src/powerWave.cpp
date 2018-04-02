#include "powerWave.h"

#define PI 3.14159265
/*powerWave::powerWave(waveform& vWave, waveform& iWave) :
waveform::waveform(vWave._numPts)*/
powerWave::powerWave(int numPts, int SAMPLE_RATE)
//initialize the wave object with the number of samples it will take
// as numPts
{
  this->_pData.clear();
  /*this->_pDataSize = (numPts - 2);
  this->_pData.resize(this->_pDataSize);*/
  /*this->_vWave = & vWave;
  this->_iWave = & iWave;*/
  this->_vWave = NULL;
  this->_iWave = NULL;
  this->_SAMPLE_RATE = SAMPLE_RATE;
}

void powerWave::clearWave()
{
  /*this->_pData.clear();*/
  this->_apparentP = NULL;
  this->_realP = NULL;
  this->_reactiveP = NULL;
  this->_PF = NULL;
  for(size_t i = 0; i < this->_pData.size(); i++)
  {
    this->_pData[i] = 0;
    this->_pDataAngle[i] = 0;
  }
  /*this->_vWave = NULL;
  this->_iWave = NULL;*/
}

void powerWave::addComponents(waveform& vWave, waveform& iWave)
{
  if(this->_vWave == NULL) this->_vWave = & vWave;
  if(this->_iWave == NULL) this->_iWave = & iWave;
  this->_pDataSize = (iWave._numPts - (iWave._filtKernalSize-1))*2;
  this->_pData.resize(this->_pDataSize);
  this->_pDataAngle.resize(this->_pDataSize);
}

double powerWave::getApparentP()
{
  return this->_apparentP;
}
double powerWave::getRealP()
{
  return this->_realP;
}
double powerWave::getReactiveP()
{
  return this->_reactiveP;
}
double powerWave::getPF()
{
  return this->_PF;
}

double powerWave::getSpectrum(int i)
{
  int vsize = this->_pData.size();
  if(i <= vsize) return this->_pData[i];
  else return -1;
}

float powerWave::getHarmonic(int i)
{
  if(i <= this->_numHarmonics) return this->_harmonics[i];
  else return -1;
}

float powerWave::getHarmonicAngle(int i)
{
  if(i <= this->_numHarmonics) return this->_harmonicsAngle[i];
  else return -1;
}

size_t powerWave::getNumHarmoics()
{
  return this->_numHarmonics;
}

int powerWave::getPDataSize()
{
  return this->_pData.size();
}

void powerWave::calcP()
{
  if(this->_apparentP == NULL)
  {
    double vRMS = this->_vWave->_RMS;
    double iRMS = this->_iWave->_RMS;
    this-> _apparentP = vRMS * iRMS;

    double size = this->_vWave->_numPts;
    double sum = 0;
    double v = 0;
    double i = 0;
    size_t index = (this->_iWave->_filtKernalSize-1)/2;
    for(int m = index; m < size - index; ++m)
    {
      v = this->_vWave->_filterDatapoints[m] - this->_vWave->getAverage();
      i = this->_iWave->_filterDatapoints[m] - this->_iWave->getAverage();
      // this->_pData[m-index] = i*v;
      this->_pData[m-index] = i; //FFT on just current
      sum += v*i;
    }
    this->_realP = sum/(size-(2.0*index));
  }

  this->_PF = (this->_realP)/(this->_apparentP);
  this->_reactiveP = sin(acos(this->_PF)) * this->_apparentP;

  size_t len = this->_pData.size()/2;
  float pi = 3.1415;
  double buff[len];

  trimData(this->_pData, this->_iWave->_peakVect);

  for(size_t i=0; i<len; i++)
  {
    //Blackman Window Function from http://www.dspguide.com/ch16/1.htm
    // buff[i] = this->_pData[i]*(0.42 - .5*cos(2*pi*(i)/len) + 0.08*cos(4*pi*(i)/len));
    buff[i] = this->_pData[i];
  }
  for(size_t i=0; i<len*2; i++)
  {
    if(i%2 == 0) this->_pData[i] = buff[i/2];
    else this->_pData[i] = 0;
  }
}

void powerWave::trimData(std::vector<float>& data, std::vector<unsigned long>& peaks)
{
  std::vector<float> temp = data;
  int size = data.size();
  size_t start = 0;
  size_t stop = 0;
  size_t i = 0;
  while(peaks[i] == 0 && i < peaks.size())
  {
    temp[i] = 0;
    i++;
  }
  start = i;

  i = peaks.size()-1;
  while(peaks[i] == 0 && i > 0)
  {
    temp[i] = 0;
    i--;
  }
  stop = i;
  data.clear();
  data.resize(size, 0);

  i = start;
  size_t k = 0;
  while( i <= stop)
  {
    data[k] = temp[i];
    i++;
    k++;
  }
}

//test with pure sine wave
void powerWave::computeFFT()
{
  size_t len = this->_pData.size();
  float data[len];
  for(size_t i=0; i<len; i++)
  {
    data[i] = this->_pData[i];
  }

  this->FFT(data, len/2, 1);

  //second half of FFT is mirror with no useful information
  float denom = 0;
  for(size_t i=0; i<len; i++)
  {
    if(i < len/4)
    {
      // Sending <RE, IM> for each frequency bucket, where
      // _pData[i] = RE and _pDataAngle[i] = IM
      this->_pData[i] = data[i*2];
      this->_pDataAngle[i] = data[(i*2)+1];
      // commented old code below:
      // float t1 = data[i*2];
      // float t2 = data[(i*2)+1];
      // this->_pData[i] = sqrt(t1*t1+t2*t2)*2/len; //no sqrt, this cooresponds to energy
      // this->_pDataAngle[i] = atan2(t2,t1) * 180/PI;
      // denom += (t1*t1+t2*t2);
    } else {
      this->_pData[i] = 0;
      this->_pDataAngle[i] = 0;
    }
  }

  // Taking harmonic power as percentage over total spectrum
  // for(size_t i=0; i<len/4; i++)
  // {
  //     this->_pData[i] = roundf(10000*this->_pData[i]/denom)/100;
  // }

  float denom2 = 0;
  this->_errVal = this->_pDataSize;
  int resolution = this->_SAMPLE_RATE * 2 / len; //len of our fft is actually len/2, hence 2 in numerator
  for(size_t i=0; i<this->_numHarmonics; i++)
  {
    double intpart;
    int index = i*60;
    double fracpart = (float)index/(float)resolution;
    float frac = modf(fracpart, &intpart);
    index = intpart;
    // this->_harmonics[i] = index+frac;
    this->_harmonics[i] = (this->_pData[index]*(1-frac) + this->_pData[index+1]*frac);
    this->_harmonicsAngle[i] = (this->_pDataAngle[index]*(1-frac) + this->_pDataAngle[index+1]*frac);
    // denom2 += this->_harmonics[i];
  }
  // taking harmonic power over only harmonic frequencies
  for(size_t i=0; i<this->_numHarmonics; i++)
  {
      // this->_harmonics[i] = roundf(10000*this->_harmonics[i]/denom2)/100;
      this->_harmonics[i] = roundf(1000*this->_harmonics[i])/1000;
      this->_harmonicsAngle[i] = roundf(1000*this->_harmonicsAngle[i])/1000;
  }
}

#define SWAP(a,b)tempr=(a);(a)=(b);(b)=tempr
//tempr is a variable from our FFT function
// source: https://www.codeproject.com/Articles/9388/How-to-implement-the-FFT-algorithm
//data -> float array that represent the array of complex samples
//number_of_complex_samples -> number of samples (N^2 order number)
//isign -> 1 to calculate FFT and -1 to calculate Reverse FFT
float powerWave::FFT (float data[], unsigned long number_of_complex_samples, int isign)
{
    //variables for trigonometric recurrences
    unsigned long n,mmax,m,j,istep,i;
    double wtemp,wr,wpr,wpi,wi,theta,tempr,tempi;
    float pi = 3.1415;

    // the complex array is real+complex so the array
    //as a size n = 2* number of complex samples
    // real part is the data[index] and
    //the complex part is the data[index+1]
    n=number_of_complex_samples * 2;

    //binary inversion (note that the indexes
    //start from 0 witch means that the
    //real part of the complex is on the even-indexes
    //and the complex part is on the odd-indexes
    j=0;
    for (i=0;i<n/2;i+=2) {
        if (j > i) {
            //swap the real part
            SWAP(data[j],data[i]);
            //swap the complex part
            SWAP(data[j+1],data[i+1]);
            // checks if the changes occurs in the first half
            // and use the mirrored effect on the second half
            if((j/2)<(n/4)){
                //swap the real part
                SWAP(data[(n-(i+2))],data[(n-(j+2))]);
                //swap the complex part
                SWAP(data[(n-(i+2))+1],data[(n-(j+2))+1]);
            }
        }
        m=n/2;
        while (m >= 2 && j >= m) {
            j -= m;
            m = m/2;
        }
        j += m;
    }

        //Danielson-Lanzcos routine
      mmax=2;
      //external loop
      while (n > mmax)
      {
          istep = mmax<<  1;
          theta=-1*(2*pi/mmax);
          wtemp=sin(0.5*theta);
          wpr = -2.0*wtemp*wtemp;
          wpi=sin(theta);
          wr=1.0;
          wi=0.0;
          //internal loops
          for (m=1;m<mmax;m+=2) {
              for (i= m;i<=n;i+=istep) {
                  j=i+mmax;
                  tempr=wr*data[j-1]-wi*data[j];
                  tempi=wr*data[j]+wi*data[j-1];
                  data[j-1]=data[i-1]-tempr;
                  data[j]=data[i]-tempi;
                  data[i-1] += tempr;
                  data[i] += tempi;
              }
              wr=(wtemp=wr)*wpr-wi*wpi+wr;
              wi=wi*wpr+wtemp*wpi+wi;
          }
          mmax=istep;
      }
}
