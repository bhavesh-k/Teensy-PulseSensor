#include <CircularBuffer.h>
#define UseSerial

volatile long prevMs[3] = { 0 };
volatile long currentMs[3] = { 0 };

const int sensorPins[3] = {22, 15, 14};
const int ledPins[3] = {3, 6, 9};
int thresh[3] = { 0 }; // pulse detect threshold initial value
int drop[3] = { 0 }; // pulse drop thresh initial value
bool trig[3] = {false, false, false}; // used to track whether pulse has crossed thresh (for Schmitt triggering purposes)
int pulses[3] = { 0 }; // count number of pulses detected on each sensor
const int noiseFloor = 0; // approx. lowest ADC value during noise measurement
const int BUFSZ = 100; // circular buffer size for moving average
const int CUMBUFSZ = 2000; // circular buffer size for long term trending
const int BPMBUF = 4; // // circular buffer size for BPM avg

int adcVal[3] = { 0 }; // current ADC val for each pulse sensor
int maxVal[3] = { 0 }; // max ADC val from last CUMBUFSZ samples for each sensor
int minVal[3] = { 0 }; // min ADC val from last CUMBUFSZ samples for each sensor
int range[3] = { 0 }; // range (max - min) of ADC val from last CUMBUFSZ samples for each sensor
bool replaceMax = false; // used to flag when the circ buffer's current max value has exited/popped
bool replaceMin = false; // used to flag when the circ buffer's current min value has exited/popped
float avgs[3] = { 0 }; // avg ADC val from last BUFSZ samples for each sensor
CircularBuffer<int,BUFSZ> adcStore[3] = { }; // store latest BUFSZ ADC values for each sensor
//float cumAvgs[3] = { 0 }; // avg ADC val from last CUMBUFSZ samples for each sensor
CircularBuffer<int,CUMBUFSZ> adcCumStore[3] = { }; // store CUMBUFSZ ADC values for each sensor for long term trending
bool bufReady = false; // wait until buffer is full before doing moving avg and max on it
int serialCount = 0; // only write data to serial output 1 in N times, using this counter

CircularBuffer<int, BPMBUF> timeDiffs[3] = { }; // store latest 5 time differences
float avgBPM[3] = { 0 };  // calculated BPM from each sensor
bool bpmReady[3] = {false, false, false}; // indicates whether BPM estimate is ready for each sensor

void setup() {
  // put your setup code here, to run once:
  #ifdef UseSerial
    Serial.begin(2000000);
  #endif
  
  for(int i = 0; i < 3; ++i)
  {
    pinMode(ledPins[i], OUTPUT);
  }
}

void loop() {

  delay(1);

  // read all 3 pulse sensors
  for(int i = 0; i < 3; ++i)
  {
    adcVal[i] = analogRead(sensorPins[i]);
    if(bufReady)
    {
      avgs[i] += (adcVal[i]/float(BUFSZ)) - (adcStore[i].last()/float(BUFSZ));
//      cumAvgs[i] += (adcVal[i]/float(CUMBUFSZ)) - (adcCumStore[i].last()/float(CUMBUFSZ));
      
      if(maxVal[i] <= adcCumStore[i].last())
        replaceMax = true;
      else
        maxVal[i] = max(maxVal[i], avgs[i]);
      if(minVal[i] == adcCumStore[i].last())
        replaceMin = true;
      else
        minVal[i] = min(minVal[i], avgs[i]);
    }
    adcStore[i].unshift(adcVal[i]); // store the new sensor value in the moving avg circular buffer
    adcCumStore[i].unshift(avgs[i]); // store the new avg in the cumulative circular buffer

    if(replaceMax)
    {
      replaceMax = false;
      int maximum = 0;
      for(int j = 0; j < CUMBUFSZ; ++j)
      {
        maximum = max(maximum, adcCumStore[i][j]);
      }
      maxVal[i] = maximum;
    }
    if(replaceMin)
    {
      replaceMin = false;
      int minimum = 9999;
      for(int j = 0; j < CUMBUFSZ; ++j)
      {
        minimum = min(minimum, adcCumStore[i][j]);
      }
      minVal[i] = minimum;
    }
    
    range[i] = maxVal[i] - minVal[i];
  }

  // write only 1 in 1000 measurements to serial output
  if(serialCount == 10)
  {
    #ifdef UseSerial
        Serial.printf("%.02f,%.02f,%.02f\n", avgBPM[0],avgBPM[1],avgBPM[2]);
//      Serial.printf("%.02f,%.d,%d,%d,%d\n", avgs[1],maxVal[1],minVal[1],thresh[1],drop[1]);
//      Serial.printf("%.02f,%.02f,%.02f\n", avgs[0],avgs[1],avgs[2]);
    #endif
    serialCount = 0;
    if(!bufReady)
    {
      bufReady = true;
      for(int i = 0; i < 3; ++i)
      {
        int total = 0;
        for(int j = 0; j < BUFSZ; ++j)
        {
          total += adcStore[i][j];
        }
        avgs[i] = total / float(BUFSZ);

//        int cumTotal = 0;
        int maximum = 0;
        int minimum = 9999;
        for(int j = 0; j < CUMBUFSZ; ++j)
        {
          maximum = max(maximum, adcCumStore[i][j]);
          minimum = min(minimum, adcCumStore[i][j]);
//          cumTotal += adcCumStore[i][j];
        }
//        cumAvgs[i] = cumTotal / float(CUMBUFSZ);
        maxVal[i] = maximum;
        minVal[i] = minimum;
        range[i] = maximum - minimum;
      }
    }
  }
  else
  {
    serialCount++;
  }

  // blink relevant LEDs with the person's pulse
  for(int i = 0; i < 3; ++i)
  {
    thresh[i] = int(0.7 * range[i] + minVal[i]);
    if(avgs[i] > thresh[i])
    {
      if(range[i] > 100)
      {
        if(!trig[i])
        {
          trig[i] = true;
          pulses[i]++;
          prevMs[i] = currentMs[i];
          currentMs[i] = millis();

          // if more than 2 seconds have passed since a peak, assume that
          // the person has left and reset the BPM estimation
          long timePassed = currentMs[i] - prevMs[i];
          if(timePassed > 2000)
          {
            bpmReady[i] = false;
            avgBPM[i] = 0;
            timeDiffs[i].clear();
          }
          // otherwise calculate the BPM
          else
          {
            timeDiffs[i].unshift(float(timePassed));
            if(timeDiffs[i].isFull())
            {
              bpmReady[i] = true;
              int total = 0;
              for(int j =  0; j < BPMBUF; j++)
              {
                total += timeDiffs[i][j];
              }
              avgBPM[i] = 60000.0 / (total / float(BPMBUF));
              digitalWrite(ledPins[i], HIGH);
            }
          }
        }
      }
    }
    else
    {
      if(trig[i])
      {
        drop[i] = int(minVal[i] + 0.3 * range[i]);
        if(avgs[i] < drop[i])
        {
          trig[i] = false;
        }
      }
      digitalWrite(ledPins[i], LOW);
    }

  }
}
