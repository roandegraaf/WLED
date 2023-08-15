#pragma once

#include "wled.h"
#include "palettes.h"

//v2 usermod that allows to change brightness and color using a rotary encoder, 
//change between modes by pressing a button (many encoders have one included)
class RotaryEncoderBrightnessColor : public Usermod
{
private:
  //Private class members. You can declare variables and functions only accessible to your usermod here
  unsigned long lastTime = 0;
  unsigned long currentTime;
  unsigned long loopTime;

  unsigned char select_state = 0; // 0 = brightness 1 = color
  unsigned char button_state = HIGH;
  unsigned char prev_button_state = HIGH;
  CRGB fastled_col;
  CHSV prim_hsv;
  int16_t new_val;

  unsigned char Enc_A;
  unsigned char Enc_B;
  unsigned char Enc_A_prev = 0;

  // private class memebers configurable by Usermod Settings (defaults set inside readFromConfig())
  int8_t pins[3]; // pins[0] = DT from encoder, pins[1] = CLK from encoder, pins[2] = CLK from encoder (optional)
  int fadeAmountBri; 
  int fadeAmountCol;
  int fadeAmountTemp;

  int kelvinValue = 2500; // default value for temperature mode

public:
  //Functions called by WLED

  /*
   * setup() is called once at boot. WiFi is not yet connected at this point.
   * You can use it to initialize variables, sensors or similar.
   */
  void setup()
  {
    //Serial.println("Hello from my usermod!");
    pinMode(pins[0], INPUT_PULLUP);
    pinMode(pins[1], INPUT_PULLUP);
    if(pins[2] >= 0) pinMode(pins[2], INPUT_PULLUP);
    currentTime = millis();
    loopTime = currentTime;
  }

  /*
   * loop() is called continuously. Here you can check for events, read sensors, etc.
   * 
   * Tips:
   * 1. You can use "if (WLED_CONNECTED)" to check for a successful network connection.
   *    Additionally, "if (WLED_MQTT_CONNECTED)" is available to check for a connection to an MQTT broker.
   * 
   * 2. Try to avoid using the delay() function. NEVER use delays longer than 10 milliseconds.
   *    Instead, use a timer check as shown here.
   */
  void loop()
{
  currentTime = millis(); // get the current elapsed time

  if (currentTime >= (loopTime + 1)) // 1ms since last check of encoder = 1000Hz
  {
    loopTime = currentTime; // Updates loopTime
    
    int encoderState = digitalRead(pins[0]) << 1 | digitalRead(pins[1]);
    if (encoderState != Enc_A_prev)
    {
      if ((encoderState == 0b01 && Enc_A_prev == 0b00) ||
          (encoderState == 0b10 && Enc_A_prev == 0b11))
      {
        // Clockwise rotation
        handleEncoderClockwise();
      }
      else if ((encoderState == 0b10 && Enc_A_prev == 0b00) ||
               (encoderState == 0b01 && Enc_A_prev == 0b11))
      {
        // Counter-clockwise rotation
        handleEncoderCounterClockwise();
      }
    }
    Enc_A_prev = encoderState;
  }

  // Check for button press to switch between brightness and color modes
  if (digitalRead(pins[2]) == LOW && prev_button_state == HIGH)
  {
    if (select_state == 0)
    {
      select_state = 1; // Switch to color mode
    }
    else if (select_state == 1)
    {
      select_state = 2; // Switch to temperature mode
    }
    else
    {
      select_state = 0; // Switch back to brightness mode
    }
  }
  prev_button_state = digitalRead(pins[2]);
}

void handleEncoderClockwise()
{
  if (select_state == 0)
  {
    if (bri + fadeAmountBri <= 255)
      bri += fadeAmountBri; // increase the brightness, don't go over 255
    else
      bri = 255; // Set brightness to max if it goes beyond 255
  }
  else if (select_state == 1)
  {
    // Handle color change for clockwise rotation
    fastled_col.red = col[0];
    fastled_col.green = col[1];
    fastled_col.blue = col[2];
    prim_hsv = rgb2hsv_approximate(fastled_col);
    new_val = (int16_t)prim_hsv.h + fadeAmountCol;
    if (new_val > 255)
      new_val -= 255; // roll-over if bigger than 255
    if (new_val < 0)
      new_val += 255; // roll-over if smaller than 0
    prim_hsv.h = (byte)new_val;
    hsv2rgb_rainbow(prim_hsv, fastled_col);
    col[0] = fastled_col.red;
    col[1] = fastled_col.green;
    col[2] = fastled_col.blue;
  }
  else if (select_state == 2)
  {
    // Handle temperature change for clockwise rotation 
    if (kelvinValue + fadeAmountTemp <= 10091)
      kelvinValue += fadeAmountTemp; // increase the temperature, don't go over 255
    else
      kelvinValue = 10091; // Set temperature to max if it goes beyond 255
    byte rgbw[] = {0, 0, 0, 0};
    colorKtoRGB2(kelvinValue, rgbw);
  }
  // Call colorUpdated and updateInterfaces
  colorUpdated(CALL_MODE_BUTTON);
  updateInterfaces(CALL_MODE_BUTTON);
}

void handleEncoderCounterClockwise()
{
  if (select_state == 0)
  {
    if (bri - fadeAmountBri >= 5) // Adjusted minimum value to 5
      bri -= fadeAmountBri; // decrease the brightness, don't go below 5
    else
      bri = 5; // Set brightness to min if it goes below 5
  }
  else if (select_state == 1)
  {
    // Handle color change for counter-clockwise rotation
    fastled_col.red = col[0];
    fastled_col.green = col[1];
    fastled_col.blue = col[2];
    prim_hsv = rgb2hsv_approximate(fastled_col);
    new_val = (int16_t)prim_hsv.h - fadeAmountCol;
    if (new_val > 255)
      new_val -= 255; // roll-over if bigger than 255
    if (new_val < 0)
      new_val += 255; // roll-over if smaller than 0
    prim_hsv.h = (byte)new_val;
    hsv2rgb_rainbow(prim_hsv, fastled_col);
    col[0] = fastled_col.red;
    col[1] = fastled_col.green;
    col[2] = fastled_col.blue;
  }
  else if (select_state == 2)
  {
    // Handle temperature change for counter-clockwise rotation
    if (kelvinValue - fadeAmountTemp >= 1900)
      kelvinValue -= fadeAmountTemp; // decrease the temperature, don't go below 0
    else
      kelvinValue = 1900; // Set temperature to min if it goes below 0
    byte rgbw[] = {0, 0, 0, 0};
    colorKtoRGB2(kelvinValue, rgbw);
  }

  // Call colorUpdated and updateInterfaces
  colorUpdated(CALL_MODE_BUTTON);
  updateInterfaces(CALL_MODE_BUTTON);
}

void colorKtoRGB2(uint16_t kelvin, byte* rgb) //white spectrum to rgb, calc
{
  int r = 0, g = 0, b = 0;
  float temp = kelvin / 100.0f;
  if (temp <= 66.0f) {
    r = 255;
    g = roundf(99.4708025861f * logf(temp) - 161.1195681661f);
    if (temp <= 19.0f) {
      b = 0;
    } else {
      b = roundf(138.5177312231f * logf((temp - 10.0f)) - 305.0447927307f);
    }
  } else {
    r = roundf(329.698727446f * powf((temp - 60.0f), -0.1332047592f));
    g = roundf(288.1221695283f * powf((temp - 60.0f), -0.0755148492f));
    b = 255;
  }
  //g += 12; //mod by Aircoookie, a bit less accurate but visibly less pinkish
  rgb[0] = (uint8_t) constrain(r, 0, 255);
  rgb[1] = (uint8_t) constrain(g, 0, 255);
  rgb[2] = (uint8_t) constrain(b, 0, 255);
  rgb[3] = 0;

  col[0] = rgb[0];
  col[1] = rgb[1];
  col[2] = rgb[2];
}

  void addToConfig(JsonObject& root)
  {
    JsonObject top = root.createNestedObject("rotEncBrightness");
    top["fadeAmountBri"] = fadeAmountBri;
    top["fadeAmountCol"] = fadeAmountCol;
    top["fadeAmountTemp"] = fadeAmountTemp;
    JsonArray pinArray = top.createNestedArray("pin");
    pinArray.add(pins[0]);
    pinArray.add(pins[1]); 
    pinArray.add(pins[2]); 
  }

  /* 
   * This example uses a more robust method of checking for missing values in the config, and setting back to defaults:
   * - The getJsonValue() function copies the value to the variable only if the key requested is present, returning false with no copy if the value isn't present
   * - configComplete is used to return false if any value is missing, not just if the main object is missing
   * - The defaults are loaded every time readFromConfig() is run, not just once after boot
   * 
   * This ensures that missing values are added to the config, with their default values, in the rare but plauible cases of:
   * - a single value being missing at boot, e.g. if the Usermod was upgraded and a new setting was added
   * - a single value being missing after boot (e.g. if the cfg.json was manually edited and a value was removed)
   * 
   * If configComplete is false, the default values are already set, and by returning false, WLED now knows it needs to save the defaults by calling addToConfig()
   */
  bool readFromConfig(JsonObject& root)
  {
    // set defaults here, they will be set before setup() is called, and if any values parsed from ArduinoJson below are missing, the default will be used instead
    fadeAmountBri = 5;
    fadeAmountCol = 2;
    fadeAmountTemp = 100;
    pins[0] = -1;
    pins[1] = -1;
    pins[2] = -1;

    JsonObject top = root["rotEncBrightness"];

    bool configComplete = !top.isNull();
    configComplete &= getJsonValue(top["fadeAmountBri"], fadeAmountBri);
    configComplete &= getJsonValue(top["fadeAmountCol"], fadeAmountCol);
    configComplete &= getJsonValue(top["fadeAmountTemp"], fadeAmountTemp);
    configComplete &= getJsonValue(top["pin"][0], pins[0]);
    configComplete &= getJsonValue(top["pin"][1], pins[1]);
    configComplete &= getJsonValue(top["pin"][2], pins[2]);

    return configComplete;
  }
};
