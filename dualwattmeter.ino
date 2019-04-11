#include <Adafruit_NeoPixel.h>
#define VERSIONSTR "dualwattmeter standalone box"
#define BAUDRATE 57600
#define PRINTHEADER "millis()\t\tlastPrintDisplay\twattage0\t\twattage1" // a legend to the printed header

#define PRINTDISPLAYRATE 1000 // how many milliseconds between running printDisplay
#define UPDATEDISPLAYRATE 1000 // how many milliseconds between running updateDisplays

#define VOLTPIN A0 // Voltage Sensor Pin
#define AMPSPIN0 A3 // Current Sensor Pin
#define AMPSPIN1 A2 // Current Sensor Pin
#define NOISYZERO 0.5  // assume any smaller measurement should be 0
#define OVERSAMPLING 25.0 // analog oversampling
#define VOLTCOEFF 13.36  // larger number interprets as lower voltage
#define AMPCOEFF 13.05  // PLUSOUT = OUTPUT, PLUSRAIL = PEDAL INPUT
#define AMPOFFSET 118.0 // when current sensor is at 0 amps this is the ADC value
float voltage, current0, current1, wattage0, wattage1;

unsigned long lastPrintDisplay = 0; // when's the last time we did printDisplay
unsigned long lastUpdateDisplays = 0; // when's the last time we did updateDisplays
unsigned long backupTimer = 0;

#define POWER_STRIP_PIN         7
#define POWER_STRIP_PIXELS      40 // number of pixels in whatwatt power column
#define DISPLAY0_PIN    4
#define DISPLAY1_PIN    5
#define DISPLAY_PIXELS (8*28) // actually 27 wide but leftmost doesn't exist
// bottom right is first pixel, goes up 8, left 1, down 8, left 1...
// https://www.aliexpress.com/item/8-32-Pixel/32225275406.html
#include "font1.h"
uint32_t fontColor = Adafruit_NeoPixel::Color(64,64,25);
uint32_t backgroundColor = Adafruit_NeoPixel::Color(0,0,1);
Adafruit_NeoPixel display0 = Adafruit_NeoPixel(DISPLAY_PIXELS, DISPLAY0_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel display1 = Adafruit_NeoPixel(DISPLAY_PIXELS, DISPLAY1_PIN, NEO_GRB + NEO_KHZ800);

// scale the logarithmic displays
#define MIN_POWER 1
#define MAX_POWER 50000

void setup() {
  Serial.begin(BAUDRATE);
  Serial.println(VERSIONSTR);
  Serial.println(PRINTHEADER);
  display0.begin();
  display0.show();
  display1.begin();
  display1.show();
}

void loop() {
  updateDisplays();
  printDisplay();
  getVoltages();
  // wattage = (millis()/100)%10000; // for testing powerStrip
  // updatePowerStrip();
}

void getVoltages(){
  voltage = average(analogRead(VOLTPIN) / VOLTCOEFF, voltage);

  uint32_t ampsRaw = 0; // reset adder
  for(int j = 0; j < OVERSAMPLING; j++) ampsRaw += analogRead(AMPSPIN0) - AMPOFFSET;
  current0 = ((float)ampsRaw / OVERSAMPLING) / AMPCOEFF;  // PLUSOUT = OUTPUT, PLUSRAIL = PEDAL INPUT
  if( current0 < NOISYZERO ) current0 = 0; // we assume anything near or below zero is a reading error
  wattage0 = voltage * current0;

  ampsRaw = 0; // reset adder
  for(int j = 0; j < OVERSAMPLING; j++) ampsRaw += analogRead(AMPSPIN1) - AMPOFFSET;
  current1 = ((float)ampsRaw / OVERSAMPLING) / AMPCOEFF;  // PLUSOUT = OUTPUT, PLUSRAIL = PEDAL INPUT
  if( current1 < NOISYZERO ) current1 = 0; // we assume anything near or below zero is a reading error
  wattage1 = voltage * current1;
}

float average(float val, float avg){
  if(avg == 0) avg = val;
  return (val + (avg * (OVERSAMPLING - 1))) / OVERSAMPLING;
}

void printDisplay() {
  if (millis() - lastPrintDisplay > PRINTDISPLAYRATE) {
    Serial.print(millis());
    Serial.print("\t\t\t");
    Serial.print(millis() - lastPrintDisplay);
    Serial.print("\t\t\t");
    Serial.print(wattage0);
    Serial.print("\t\t\t");
    Serial.println(wattage1);
    lastPrintDisplay = millis();
  }
}

void updatePowerStrip(const Adafruit_NeoPixel& strip, int wattage){
  float ledstolight;
  ledstolight = logPowerRamp(wattage);
  if( ledstolight > POWER_STRIP_PIXELS ) ledstolight=POWER_STRIP_PIXELS;
  unsigned char hue = ledstolight/POWER_STRIP_PIXELS * 170.0;
  uint32_t color = Wheel(strip, hue<1?1:hue);
  static const uint32_t dark = Adafruit_NeoPixel::Color(0,0,0);
  doFractionalRamp(strip, 0, POWER_STRIP_PIXELS, ledstolight, color, dark);
  strip.show();
}

float logPowerRamp( float p ) {
  float l = log(p/MIN_POWER)*POWER_STRIP_PIXELS/log(MAX_POWER/MIN_POWER);
  return l<0 ? 0 : l;
}

// Input a value 0 to 255 to get a color value. The colours transition rgb back to r.
uint32_t Wheel(const Adafruit_NeoPixel& strip, byte WheelPos) {
  if (WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, WheelPos * 3, 0);
  } else if (WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, 255 - WheelPos * 3, WheelPos * 3);
  } else {
    WheelPos -= 170;
    return strip.Color(WheelPos * 3, 0, 255 - WheelPos * 3);
  }
}

uint32_t dim(uint32_t c){  // a full RGB color
#if ENABLE_DIMMING
  return Adafruit_NeoPixel::Color(
  dim( (uint8_t)( (c>>16) & 0xff ) ),
  dim( (uint8_t)( (c>>8 ) & 0xff ) ),
  dim( (uint8_t)( (c>>0 ) & 0xff ) ) );
#else
  return c;
#endif
}

void doFractionalRamp(const Adafruit_NeoPixel& strip, uint8_t offset, uint8_t num_pixels, float ledstolight, uint32_t firstColor, uint32_t secondColor){
  for( int i=0,pixel=offset; i<=num_pixels; i++,pixel++ ){
    uint32_t color;
    if( i<(int)ledstolight )  // definitely firstColor
        color = firstColor;
    else if( i>(int)ledstolight )  // definitely secondColor
        color = secondColor;
    else  // mix the two proportionally
        color = weighted_average_of_colors( firstColor, secondColor, ledstolight-(int)ledstolight);
    strip.setPixelColor(pixel, dim(color));
  }
}

uint32_t weighted_average_of_colors( uint32_t colorA, uint32_t colorB, float fraction ){
  // TODO:  weight brightness to look more linear to the human eye
  uint8_t RA = (colorA>>16) & 0xff;
  uint8_t GA = (colorA>>8 ) & 0xff;
  uint8_t BA = (colorA>>0 ) & 0xff;
  uint8_t RB = (colorB>>16) & 0xff;
  uint8_t GB = (colorB>>8 ) & 0xff;
  uint8_t BB = (colorB>>0 ) & 0xff;
  return Adafruit_NeoPixel::Color(
    RA*fraction + RB*(1-fraction),
    GA*fraction + GB*(1-fraction),
    BA*fraction + BB*(1-fraction) );
}

void updateDisplays() {
  if (millis() - lastUpdateDisplays > UPDATEDISPLAYRATE) {
    char buf[]="    "; // stores the number we're going to display
    sprintf(buf,"%4d",millis()/100);// for testing display
    //sprintf(buf,"%4d",(int)(wattHours / 29));
    writeDisplay(display0, buf);
    writeDisplay(display1, buf);
    lastUpdateDisplays = millis();
  }
}

void writeDisplay(const Adafruit_NeoPixel& strip, char* text) {
#define DISPLAY_CHARS   4 // number of characters in display
#define FONT_W 7 // width of font
#define FONT_H 8 // height of font
  for (int textIndex=0; textIndex<DISPLAY_CHARS; textIndex++) {
    char buffer[FONT_H][FONT_W]; // array of horizontal lines, top to bottom, left to right
    for(int fontIndex=0; fontIndex<sizeof(charList); fontIndex++){ // charList is in font1.h
      if(charList[fontIndex] == text[textIndex]){ // if fontIndex is the index of the desired letter
        int pos = fontIndex*FONT_H; // index into CHL where the character starts
        for(int row=0;row<FONT_H;row++){ // for each horizontal row of pixels
          memcpy_P(buffer[row], (PGM_P)pgm_read_word(&(CHL[pos+row])), FONT_W); // copy to buffer from flash
        }
      }
    }
    for (int fontXIndex=0; fontXIndex<FONT_W; fontXIndex++) {
      for (int fontYIndex=0; fontYIndex<FONT_H; fontYIndex++) {
        uint32_t pixelColor = buffer[fontYIndex][FONT_W-1-fontXIndex]=='0' ? fontColor : backgroundColor; // here is where the magic happens
        if ((FONT_W*(DISPLAY_CHARS-1-textIndex) + fontXIndex) & 1) { // odd columns are top-to-bottom
          strip.setPixelColor((FONT_H*FONT_W)*(DISPLAY_CHARS-1-textIndex) + fontXIndex*FONT_H +           fontYIndex ,pixelColor);
        } else { // even columns are bottom-to-top
          strip.setPixelColor((FONT_H*FONT_W)*(DISPLAY_CHARS-1-textIndex) + fontXIndex*FONT_H + (FONT_H-1-fontYIndex),pixelColor);
        }
      }
    }
  }
  strip.setPixelColor((FONT_W-1)*FONT_H+0,fontColor); // light up the decimal point
  strip.setPixelColor((FONT_W  )*FONT_H+7,backgroundColor); // keep decimal point visible
  strip.setPixelColor((FONT_W-2)*FONT_H+7,backgroundColor); // keep decimal point visible
  strip.show(); // send the update out to the LEDs
}
