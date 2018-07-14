#include <CircularBuffer.h>
#define UseSerial

volatile long prevMs[3] = { 0 };
volatile long currentMs[3] = { 0 };
const int sensorPins[3] = {16, 19, 14};
const int ledPins[3][3] = {
                           {23, 22, 21}, // RGB of LED 1
                           {3,4,5}, // LED 2
                           {6,9,10} // LED 3
                           };
int thresh[3] = { 0 }; // pulse detect threshold initial value
int drop[3] = { 0 }; // pulse drop thresh initial value
bool trig[3] = {false, false, false}; // used to track whether pulse has crossed thresh (for Schmitt triggering purposes)
int pulses[3] = { 0 }; // count number of pulses detected on each sensor
const int noiseFloor = 0; // approx. lowest ADC value during noise measurement
const int BUFSZ = 100; // circular buffer size for moving average
const int CUMBUFSZ = 1000; // circular buffer size for long term trending
const int BPMBUF = 5; // // circular buffer size for BPM avg

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
int avgBPM[3] = {80, 80, 80};  // calculated BPM from each sensor
bool bpmReady[3] = {false, false, false}; // indicates whether BPM estimate is ready for each sensor

void setup() {
  // put your setup code here, to run once:
  #ifdef UseSerial
    Serial.begin(2000000);
  #endif
  
  for(int i = 0; i < 3; ++i)
  {
    for(int j = 0; j < 3; ++j)
    {
      pinMode(ledPins[i][j], OUTPUT);
    }
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
        Serial.printf("%.02f,%d,%d,%d,%d\n", avgs[0],maxVal[0],minVal[0],thresh[0],drop[0]);
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
    thresh[i] = int(0.52 * range[i] + minVal[i]);
    if(avgs[i] > thresh[i])
    {
      if(range[i] > 0)
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
            //timeDiffs[i].clear();
            for(int j = 0; j < 3; ++j)
            {
              analogWrite(ledPins[i][j], 0);
            }
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
              avgBPM[i] = int(60000.0 / (total / float(BPMBUF)));

              // and blink/PWM the LEDS
              if(avgBPM[i] > 140)
              {
                // same color as BPM of 140
                
                // red
                analogWrite(ledPins[i][0], 255);
                // green
                analogWrite(ledPins[i][1], 0);
                // blue
                analogWrite(ledPins[i][2], 0);
              }
              else if(avgBPM[i] >= 107 && avgBPM[i] <= 140)
              {
                // red
                analogWrite(ledPins[i][0], int((255.0/33)*(avgBPM[i] - 107)));
                // green
                analogWrite(ledPins[i][1], 255 - int((255.0/33)*(avgBPM[i] - 107)));
                // blue
                analogWrite(ledPins[i][2], 0);
              }
              else if(avgBPM[i] >= 74)
              {
                // red
                analogWrite(ledPins[i][0], 0);
                // green
                analogWrite(ledPins[i][1], int((255.0/33)*(avgBPM[i] - 74)));
                // blue
                analogWrite(ledPins[i][2], 255 - int((255.0/33)*(avgBPM[i] - 74)));
              }
              else if(avgBPM[i] >= 41)
              {
                // red
                analogWrite(ledPins[i][0], 200 - int((200.0/33)*(avgBPM[i] - 41)));
                // green
                analogWrite(ledPins[i][1], 0);
                // blue
                analogWrite(ledPins[i][2], 55 + int((200.0/33)*(avgBPM[i] - 41)));
              }
              else
              {
                // same colour as BPM of 41

                // red
                analogWrite(ledPins[i][0], 200);
                // green
                analogWrite(ledPins[i][1], 0);
                // blue
                analogWrite(ledPins[i][2], 55);
              }
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
      for(int j = 0; j < 3; ++j)
      {
        analogWrite(ledPins[i][j], LOW);
      }
    }

  }
}
