#include <Arduino.h>
#include "ads.h"
#include <Bounce2.h>

//################# PIN DEFINITIONS ########################

#define ADS_RESET_PIN (3)     // Pin number attached to ads reset line.
#define ADS_INTERRUPT_PIN (4) // Pin number attached to the ads data ready line.
const byte mode_sel = 9;

//############################### VARIABLES ###########################################
float iFinger = 0.0;
int Index = 0;
String Data_In = "";
String mode = "s";

unsigned long sysClock;
int waitTime = 1000;
int clicks = 0;

//########### FUNCTION DECLARATIONS ############
void mode_set();
void ads_data_callback(float *sample);
void deadzone_filter(float *sample);
void signal_filter(float *sample);
void parse_com_port(void);

/* Receives new samples from the ADS library */
void ads_data_callback(float *sample, uint8_t sample_type)
{
  if (sample_type == ADS_SAMPLE)
  {
    // Low pass IIR filter
    signal_filter(sample);

    // Deadzone filter
    deadzone_filter(sample);

    //Serial.println(sample[0]);
    iFinger = sample[0];
    Index = (int)iFinger;
  }
}
// INSTANTIATE A Bounce OBJECT.
Bounce bounce = Bounce();

void setup()
{
  bounce.attach(mode_sel, INPUT_PULLUP);
  // DEBOUNCE INTERVAL IN MILLISECONDS
  bounce.interval(20); // interval in ms

  Serial.begin(115200); // DEBUG

  Serial1.begin(115200); //Data Output
  Serial1.setTimeout(50);

  Serial2.begin(115200); //Index Finger Input
  Serial2.setTimeout(100);

  Serial1.println("Initializing Sensor Glove...");

  ads_init_t init; // One Axis ADS initialization structure

  init.sps = ADS_100_HZ;                         // Set sample rate to 100 Hz
  init.ads_sample_callback = &ads_data_callback; // Provide callback for new data
  init.reset_pin = ADS_RESET_PIN;                // Pin connected to ADS reset line
  init.datardy_pin = ADS_INTERRUPT_PIN;          // Pin connected to ADS data ready interrupt
  init.addr = 0;                                 // Update value if non default I2C address is assinged to sensor

  // Initialize ADS hardware abstraction layer, and set the sample rate
  int ret_val = ads_init(&init);

  if (ret_val != ADS_OK)
  {
    Serial.print("Sensor Glove failed with reason: ");
    Serial.println(ret_val);
  }
  else
  {
    Serial.println("Sensor Glove Online...");
  }

  // Start reading data in interrupt mode
  ads_run(true);
}

void loop()
{
  // sysClock = millis();

  // Update the Bounce instance (YOU MUST DO THIS EVERY LOOP)
  bounce.update();
  if (bounce.changed())
  {
    int deboucedInput = bounce.read();
    if (deboucedInput == LOW && clicks == 0)
    {
      sysClock = millis();
      clicks = clicks + 1;
    }
    else if (clicks > 0 && sysClock + waitTime > millis())
    {
      clicks = clicks + 1;
    }
  }

  if (clicks == 6 && sysClock + waitTime < millis())
  {
    mode_set();
    Serial.print("clicks = ");
    Serial.println(clicks);
    sysClock = 0;
    clicks = 0;
  }

  else if (clicks > 0 && sysClock + waitTime < millis())
  {
    Serial.print("clicks = ");
    Serial.println(clicks);
    sysClock = 0;
    clicks = 0;
  }
  else if (sysClock + waitTime < millis())
  {
    sysClock = 0;
    clicks = 0;
  }

  if (Index > 120 && mode == "s")
  {
    Serial.println("mode#");
    while (Index > 118)
    {
      delay(50);
    }
  }
}

void mode_set()
{
  if (mode == "s")
  {
    mode = "a";
  }
  else if (mode == "a")
  {
    mode = "s";
  }
  Serial.print(mode);
  Serial.println('#');
}

void serialEvent()
{ //Data Request
  while (Serial.available())
  {
    // Returns data
    Data_In = Serial.readStringUntil('#');
    if (Data_In == "gfd")
    {
      Serial.print(Index);
      Serial.println('#');
      Data_In = "";
    }
  }
}

void serialEvent1()
{ //Data Request
  while (Serial1.available())
  {

    // Returns data
    Data_In = Serial1.readStringUntil('#');
    if (Data_In == "gfd")
    {
      Serial.print(Index);
      Serial.print('#');
      Data_In = "";
    }
  }
}
void serialEvent2()
{ //Data Request
  while (Serial2.available())
  {
    // Returns data
    Index = Serial2.readStringUntil('#').toInt();
  }
}

/* 
 *  Second order Infinite impulse response low pass filter. Sample freqency 100 Hz.
 *  Cutoff freqency 20 Hz. 
 */
void signal_filter(float *sample)
{
  static float filter_samples[2][6];

  for (uint8_t i = 0; i < 2; i++)
  {
    filter_samples[i][5] = filter_samples[i][4];
    filter_samples[i][4] = filter_samples[i][3];
    filter_samples[i][3] = (float)sample[i];
    filter_samples[i][2] = filter_samples[i][1];
    filter_samples[i][1] = filter_samples[i][0];

    // 20 Hz cutoff frequency @ 100 Hz Sample Rate
    filter_samples[i][0] = filter_samples[i][1] * (0.36952737735124147f) - 0.19581571265583314f * filter_samples[i][2] +
                           0.20657208382614792f * (filter_samples[i][3] + 2 * filter_samples[i][4] + filter_samples[i][5]);

    sample[i] = filter_samples[i][0];
  }
}

/* 
 *  If the current sample is less that 0.5 degrees different from the previous sample
 *  the function returns the previous sample. Removes jitter from the signal. 
 */
void deadzone_filter(float *sample)
{
  static float prev_sample[2];
  float dead_zone = 0.75f;

  for (uint8_t i = 0; i < 2; i++)
  {
    if (fabs(sample[i] - prev_sample[i]) > dead_zone)
      prev_sample[i] = sample[i];
    else
      sample[i] = prev_sample[i];
  }
}