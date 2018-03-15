
//  Created by bobolink
//  twitter: @wm6h
//  rev: 20180304

/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleWrite.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

/*
   Two core ESP-32 Arduino real-time audio demo.
   Tested on Espressif ESP32 Dev board
   Rev. 1 silicon
   Real-Time Samples at 8Ksps for voice audio range (< 4KHz).
   Not for music.
   Compatible with Arduino IDE

   Generates a sine wave
*/

#include <Arduino.h>
#include <driver/adc.h>


#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

int LED_BUILTIN = 5;

const uint16_t  N = 1024; // should be a power of 2 for FFTs
// Determines how long the Application Processor can run for 
// real-time applications
// or how long the Application has to write the samples to
// a ring buffer for non-real time applications.
// Latency, input to output, is a function of N.


// we create complex numbers here for convenience
// could be done in frame processing
volatile double realPing[N];
volatile double imagPing[N];
volatile double realPong[N];
volatile double imagPong[N];

// Do frame processing in floats for better dynamic range.
// At last stage, scale floats to range -1.0 to +1.0
// then convert to unsigned char. Mult by 127 and
// adding 128 to map +1.0 -- -1.0 to 0-255 for
// output to 8-bit unsigned DAC

// we create analytic IQ input samples
// but we only output real numbers
volatile unsigned char outRealPing[N];
volatile unsigned char outRealPong[N];

const float Pi = 3.14159;
float theta  = 0.0;
float sampleRateHz = 8000.0;
float testFreqHz = 1000.0;
//const float theta_increment = 2.0 * Pi * (testFreqHz / sampleRateHz);
float theta_increment;


float volume = 0.5;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// this is Core 1's variable.
// Core 0 will operate on the other buffer
volatile boolean pingCore1 = true;

volatile boolean sampling = false;
volatile boolean outputEnable = true;


TaskHandle_t Task1;
SemaphoreHandle_t newFrame;

esp_err_t result;

void IRAM_ATTR onTimer()
{

  portENTER_CRITICAL_ISR(&timerMux);
  sampling = true;
  portEXIT_CRITICAL_ISR(&timerMux);

}

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();

// not very safe code
       char tempc[4];
 

      if (value.length() > 0) {
        Serial.println("*********");
        Serial.print("New value: ");
        for (int i = 0; i < value.length(); i++)
        {
          //Serial.print(value[i]);
          tempc[i] = value[i];
        }

        //float f = (float) atof(tempc);
        float f = strtod(tempc,NULL);
        if( (f>30) && (f<=3000) )
            testFreqHz = f;
            
        Serial.println(f);
        Serial.println("*********");
      }
    }
};


void setup()
{

  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);

  newFrame = xSemaphoreCreateMutex();


  xTaskCreatePinnedToCore(
    frameProcessing,
    "FrameProcessing",
    1000,
    NULL,
    1,
    &Task1,
    0);

  delay(500);  // needed to start-up task1
               // from Mr. Spiess' video

  // zero the DAC buffers
  for (int i = 0; i < N; ++i)
  {
    outRealPing[i] = 0;
    outRealPong[i] = 0;

  }
  
  // this almost matches the output resolution
  // all channels  GPIOs 32-39
  result = adc1_config_width(ADC_WIDTH_9Bit);
  // complete with Apple type error message
  if (result != ESP_OK)
  {
    Serial.println("Error, an unknown error occurred");
  }

  // this might allow 3.3VDC on the mic
  // pin 34
  result = adc1_config_channel_atten( ADC1_CHANNEL_6, ADC_ATTEN_11db);
  if (result != ESP_OK)
  {
    Serial.println("Error, an unknown error occurred");
  }




  BLEDevice::init("MyESP32");
  BLEServer *pServer = BLEDevice::createServer();

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World");
  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();


  

  Serial.print("Setup: Executing on core ");
  Serial.println(xPortGetCoreID());

  hw_timer_t* timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 125, true); // sampling frequency 8kHz
  timerAlarmEnable(timer);

}

// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1
//                   This Task runs on Core: 1
// Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1Core1
// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

// ||||||||||||||||||
//  Sample Service
// ||||||||||||||||||
void loop()
{
  for (int i = 0; i < N; ++i)
  {
    while (!sampling);

    portENTER_CRITICAL(&timerMux);
    sampling = false;
    portEXIT_CRITICAL(&timerMux);

    //digitalWrite(LED_BUILTIN, HIGH);
    if (pingCore1 == true)
    {
      // configured for 9 bits with an 11 dB pad
      // drop one bit to give 8 to match the output
      realPing[i] = (float) (((byte)((adc1_get_voltage(ADC1_CHANNEL_6) >> 1) & 0xFF)) - 128.0);
      imagPing[i] = 0.0;
      // DAC output is 8-bit unsigned. Maximum (255) corresponds
      // for VDD_A 3.3V, to 2.59V
      if (outputEnable == true)
      { // GPIO pin 25 of the ESP32.
        // This is the output of DAC1
        dacWrite(25, outRealPing[i]);
      }
    }
    else
    {
      // drop one bit to give 8 to match the output
      realPong[i] = (float) (((byte)((adc1_get_voltage(ADC1_CHANNEL_6) >> 1) & 0xFF)) - 128.0);
      imagPong[i] = 0.0;
      // DAC output is 8-bit unsigned. Maximum (255) corresponds
      // for VDD_A 3.3V, to 2.59V
      if (outputEnable == true)
      { // for VDD_A 3.3V, to 2.59V
        dacWrite(25, outRealPong[i]);
      }
    } // end single sample processing
     //digitalWrite(LED_BUILTIN, LOW);
  } // end N samples processing

  // swap working buffer
  pingCore1 = !pingCore1;
  // give the old buffer to frame processing
  xSemaphoreGive(newFrame);
} // end sample service task

// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// 1111111111111111111111111111111111111111111111111111111111111
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/



// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0
//                    This Task runs on Core: 0
// Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0Core0
// |||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
// Frame processing
// Runs every N samples

void frameProcessing( void* parameter )
{
  Serial.print("Frame Processing: Executing on core ");
  Serial.println(xPortGetCoreID());
  
  for (;;) // this is required, normally put in by
  {        // Arduino IDE

    float outVal;

    outputEnable = true;

    // "Arf! Arf!"
    // pet the watchdog.
    vTaskDelay(10);
    
    // wait for the ping-pong buffer swap
    // indicating frame processing should begin
    // Core 1 has the timer resolution and is in charge of buffers
    xSemaphoreTake(newFrame, portMAX_DELAY);
    
    digitalWrite(LED_BUILTIN, HIGH);
    // don't need to give it back
    //xSemaphoreGive(newFrame);
    theta_increment = 2.0 * Pi * (testFreqHz * 0.000125);

    // frame processing takes place here:
    for (int i = 0; i < N; ++i)
    {
      outVal = sinf(theta) * volume;
      theta += theta_increment;
      if (theta > 2.0 * Pi)
      {
        theta -= 2.0 * Pi;
      }
      // NOTE:
      // this variable belongs to core 1 -- use the other buffer
      // If core 1 is ping, we are pong
      if (pingCore1 == true)
      {
        outRealPong[i] = (unsigned char) (outVal * 127.0 + 128.0);
      }
      else
      {
        outRealPing[i] = (unsigned char) (outVal * 127.0 + 128.0);
      }

    }// end frame processing of N samples
    
    digitalWrite(LED_BUILTIN, LOW); 
    // this processing takes 2.24 msec.
    // out of 128 msec. available to the Frame
    // 125 usec * 1024 = 128 msec.
    // 1.75% loaded 
    //
  } // end Arduino task loop
}
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/
// 0000000000000000000000000000000000000000000000000000000000000
// /\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/




