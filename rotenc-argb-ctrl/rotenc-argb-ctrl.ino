/**************************************************************************
Rotary Encoder with ARGB ctrl
**************************************************************************/

#include <stdbool.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_TLC5947.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <Wire.h>           // Include the I2C library (required)
#include <SparkFunSX1509.h> // Include SX1509 library


/*** SX1509 IO Expander Config ***/
#define KEY_ROWS 3
#define KEY_COLS 3
#define IO_EXP_INT_PIN A3

// How many boards do you have chained?
#define NUM_TLC5974 1
#define data   11
#define clock  13
#define latch  12
#define oe     -1    // set to -1 to not use the enable pin (its optional)

// Type returned by millis() "syscall" (???)
#define ms_time_t unsigned long

#define N_ARGB 3
#define ARGB_PINS   { A0, A1, A2 }
#define ARGB_N_LEDS { 60, 8, 12 }

#define N_BTNS      3
#define BTN_PINS    { 3, 4, 5 }
#define BTN_ADDRS   { 0,  1,  2  }

#define N_ROTENC    3
#define ROTENC_PINS { \
    2, 3, 0,\
    4, 5, 1,\
    6, 7, 2}

#define MAX_LED_VAL         255
#define N_ENCODER_STEPS     24
#define LED_INC             (MAX_LED_VAL / N_ENCODER_STEPS / 2)
#define ON_MSG              "---ARGBCTRL-ON---"
#define CW_CHAR             '>'
#define CCW_CHAR            '<'
#define SW_CHAR             'v'
#define SW_DEBOUNCE_MS      5

#define SERIAL_CMD_DELIM      ';'
#define SERIAL_CMD_MAXLEN     64
#define SERIAL_CMD_ARG_DELIM  ','
#define SERIAL_CMD_MAXARGS    4

// command format: c<stripIndex>,<red>,<green>,<blue>;
// set strip 0 to red: c0,255,0,0;
#define SERIAL_CMD_COLOR      'c'
// command format: b<btnIndex>,<red>,<green>,<blue>;
// set button 0 to red: b0,255,0,0;
#define SERIAL_CMD_BTN_COLOR  'b'

// Map of port ids to port values
#define PORTID_0            PIND
#define PORTID_1            PINB
#define PORTID_2            PINC

#define RED_ID    0
#define GREEN_ID  1
#define BLUE_ID   2
typedef uint8_t ColorId;

// RGBStrip struct for tracking color and lighting effects
typedef struct {
  uint8_t   red;              // red value
  uint8_t   green;            // green value
  uint8_t   blue;             // blue value
} RGBStrip;

// IO Switch struct
typedef struct {
  uint8_t   swPin;            // Switch pin
  uint8_t   swPort;           // Switch port register for fast read access
  ms_time_t swLastChangeMs;   // Millisecond timestamp of last switch, for debounce
  bool      swWasPressed;     // flag for switch press
  bool      activeHigh;
} IOSwitch;

// Rotary encoder struct
typedef struct  {
  // TODO: Define pins with less bits?
  uint8_t   aPin;             // A pin of rotary encoder
  uint8_t   aPort;            // A port register for fast read access
  uint8_t   bPin;             // B pin of rotary encoder
  uint8_t   bPort;            // B port register for fast read access
  uint8_t   state;            // Encoder state
  uint8_t   prevPos;          // last encoder position
  IOSwitch  btn;
} RotEnc;

// RGB Button struct
typedef struct {
  uint8_t   btnPin;           // button switch pin
  uint8_t   rgbAddr;          // address on the PWM controller
  RGBStrip  color;            // color of button
  IOSwitch  btn;
} RGBBtn;

static RGBBtn     gRGBBtns[N_BTNS]      = {0,};
static RotEnc     gRotEncs[N_ROTENC]    = {0,};

Adafruit_NeoPixel gARGBStrips[N_ARGB]     = {0,};
static ms_time_t  gLastARGBUpdate[N_ARGB] = {0,};
Adafruit_TLC5947  tlc                   = Adafruit_TLC5947(NUM_TLC5974, clock, data, latch);
static ms_time_t  gLastRGBBtnUpdate     = 0;
static uint8_t    gSelectedStrip        = 0;
static SX1509     io;


// see link for port info https://www.arduino.cc/en/Reference/PortManipulation
uint8_t calcPinPort(uint8_t pin)
{
  if(pin >= 0 && pin <= 7)
    return 0;
  else if(pin >= 8 && pin <= 13)
    return 1;
  else if(pin >= A0 && pin <= A5)
    return 2;

  // Invalid pin
  Serial.print("---BAD_PIN_PORT(");
  Serial.print(pin, DEC);
  Serial.println(")---");
  return 0;
}

uint8_t portFromId(uint8_t id)
{
  switch(id) {
    case 0:
      return PORTID_0;
    case 1:
      return PORTID_1;
    case 2:
      return PORTID_2;
    default:
      Serial.print("---BAD_PORT_ID(");
      Serial.print(id, DEC);
      Serial.println(")---");
      return 0;
  }
}

void setupEncoder()
{
  uint8_t rotEncPins[] = ROTENC_PINS;
  // init tracking structures
  // no deinit done, assuming these will never be destroyed
  for(int i=0; i < N_ROTENC; i++) {
    // init RotEnc struct
    gRotEncs[i].aPin   = rotEncPins[i*3];
    gRotEncs[i].aPort  = calcPinPort(gRotEncs[i].aPin);
    gRotEncs[i].bPin   = rotEncPins[i*3+1];
    gRotEncs[i].bPort  = calcPinPort(gRotEncs[i].bPin);
    gRotEncs[i].btn.swPin  = rotEncPins[i*3+2];

    pinMode(gRotEncs[i].aPin, INPUT_PULLUP);
    pinMode(gRotEncs[i].bPin, INPUT_PULLUP);
    io.pinMode(gRotEncs[i].btn.swPin, INPUT_PULLUP);

    // get an initial reading on the encoder pins
    if (digitalRead(gRotEncs[i].aPin) == LOW)
      gRotEncs[i].prevPos |= (1 << 0);
    if (digitalRead(gRotEncs[i].bPin) == LOW)
      gRotEncs[i].prevPos |= (1 << 1);


    Serial.print("---RotEnc[");
    Serial.print(i, DEC);
    Serial.print("]={aPin=");
    Serial.print(gRotEncs[i].aPin);
    Serial.print(",bPin=");
    Serial.print(gRotEncs[i].bPin);
    Serial.print(",swPin=");
    Serial.print(gRotEncs[i].btn.swPin);
    Serial.println("}");
  }

  Serial.print("---Setup ");
  Serial.print(N_ROTENC, DEC);
  Serial.println(" RotEncs---");
}

void setupARGB()
{
  uint8_t stripPins[]  = ARGB_PINS;
  uint8_t stripNLeds[] = ARGB_N_LEDS;;
  for(int i=0; i < N_ARGB; i++) {
    gARGBStrips[i] = Adafruit_NeoPixel(stripNLeds[i], stripPins[i], NEO_GRB + NEO_KHZ800);
    gARGBStrips[i].begin();
    gARGBStrips[i].show(); // Initialize all pixels to 'off'
  }
}

void setupRGBBtns()
{
  uint8_t btnPins[]   = BTN_PINS;
  uint8_t btnAddrs[]  = BTN_ADDRS;

  // init tracking structures
  // no deinit done, assuming these will never be destroyed
  for(int i=0; i < N_BTNS; i++) {
    // init RGBBtn structs
    gRGBBtns[i].btn.swPin       = btnPins[i];
    gRGBBtns[i].btn.activeHigh  = true;
    gRGBBtns[i].rgbAddr         = btnAddrs[i];
    gRGBBtns[i].color.red       = 0;
    gRGBBtns[i].color.green     = 0;
    gRGBBtns[i].color.blue      = 0;
    io.pinMode(gRotEncs[i].btn.swPin, INPUT);


    Serial.print("---RGBBtn[");
    Serial.print(i, DEC);
    Serial.print("],rgbAddr=");
    Serial.print(gRGBBtns[i].rgbAddr);
    Serial.print(",swPin=");
    Serial.print(gRGBBtns[i].btn.swPin);
    Serial.println("}");

  }

  Serial.print("---Setup ");
  Serial.print(N_BTNS, DEC);
  Serial.println(" RGBBtns---");
}

void setup()
{
  Serial.begin(9600);
  Serial.println(ON_MSG);

  if (!io.begin(0x3E))
  {
    Serial.println("IO expander setup failed");
    // If we failed to communicate, turn the pin 13 LED on
    int on = HIGH;
    while (1) {
      digitalWrite(LED_BUILTIN, on);
      if(on == HIGH)
        on = LOW;
      else
        on = HIGH;
      delay(200);
    }
  }

  setupEncoder();
  setupARGB();
  setupRGBBtns();
  tlc.begin();


}

int8_t getEncoderAction(RotEnc* rotEnc)
{
  int8_t enc_action = 0; // 1 or -1 if moved, sign is direction
  uint8_t aPort = portFromId(rotEnc->aPort);
  uint8_t bPort = portFromId(rotEnc->bPort);

  // note: for better performance, the code will use
  // direct port access techniques
  // http://www.arduino.cc/en/Reference/PortManipulation
  uint8_t enc_cur_pos = 0;
  // read in the encoder state first
  if (bit_is_clear(aPort, rotEnc->aPin % 8))
    enc_cur_pos |= (1 << 0);
  if (bit_is_clear(bPort, rotEnc->bPin % 8))
    enc_cur_pos |= (1 << 1);

  // if any rotation at all
  if (enc_cur_pos != rotEnc->prevPos) {
    if (rotEnc->prevPos == 0x00) {
      // this is the first edge
      if (enc_cur_pos == 0x01)
        rotEnc->state |= (1 << 0);
      else if (enc_cur_pos == 0x02)
        rotEnc->state |= (1 << 1);
    }

      // this is when the encoder is in the middle of a "step"
    if (enc_cur_pos == 0x03) {
      rotEnc->state |= (1 << 4);
    } else if (enc_cur_pos == 0x00) {
      // this is the final edge
      if (rotEnc->prevPos == 0x02) {
        rotEnc->state |= (1 << 2);
      }
      else if (rotEnc->prevPos == 0x01) {
        rotEnc->state |= (1 << 3);
      }

      // check the first and last edge
      // or maybe one edge is missing, if missing then require the middle state
      // this will reject bounces and false movements
      if (bit_is_set(rotEnc->state, 0) && (bit_is_set(rotEnc->state, 2) || bit_is_set(rotEnc->state, 4)))
        enc_action = 1;
      else if (bit_is_set(rotEnc->state, 2) && (bit_is_set(rotEnc->state, 0) || bit_is_set(rotEnc->state, 4)))
        enc_action = 1;
      else if (bit_is_set(rotEnc->state, 1) && (bit_is_set(rotEnc->state, 3) || bit_is_set(rotEnc->state, 4)))
        enc_action = -1;
      else if (bit_is_set(rotEnc->state, 3) && (bit_is_set(rotEnc->state, 1) || bit_is_set(rotEnc->state, 4)))
        enc_action = -1;

      rotEnc->state = 0; // reset for next time
    }
  }

  rotEnc->prevPos = enc_cur_pos;
  return enc_action;
}

void updateStrip(RGBStrip* rgb, ColorId id, bool inc, bool dec, bool press)
{
  uint8_t* color = NULL;
  
  switch(id) {
    case RED_ID:
      color = &rgb->red;
      break;
    case GREEN_ID:
      color = &rgb->green;
      break;
    case BLUE_ID:
      color = &rgb->blue;
      break;
    default:
      Serial.println("---BAD_COLOR_ID---");
      return;
  }

  if(inc) {
    if(*color < MAX_LED_VAL)
      *color = *color+1;
  } else if(dec) {
      if(*color > 0)
        *color = *color-1;
  } else if(press) {
    if(*color == 0)
      *color = MAX_LED_VAL / 4;
    else if(*color == (MAX_LED_VAL / 4))
      *color = MAX_LED_VAL;
    else
      *color = 0;
  }
}

bool ioSwitchActivated(IOSwitch* sw)
{
  ms_time_t now = millis();
  bool wasActivated = false;

  // handle millis() rollover after 50 days https://www.arduino.cc/reference/en/language/functions/time/millis/
  if(now < sw->swLastChangeMs)
    sw->swLastChangeMs = 0;

  // only handle changes once we are out of the debounce delay
  if((now - sw->swLastChangeMs) >= SW_DEBOUNCE_MS) {
    bool isHigh = io.digitalRead(sw->swPin) == HIGH;
    bool active = (isHigh && sw->activeHigh) ||  (!isHigh && !sw->activeHigh);
    if(active) {
      // only fire on the first press
      if(!sw->swWasPressed) {
        wasActivated = true;
        sw->swLastChangeMs = now;
        sw->swWasPressed = true;
      }
    } else {
      // debounce check
      sw->swWasPressed = false;
      sw->swLastChangeMs = now;
    }
  }
  return wasActivated;
}

void handleRotEncUpdates()
{
  // loop through encoders
  for(uint8_t i=0; i < N_ROTENC; i++) {
    // handle rotation updates
    int8_t encAction    = getEncoderAction(&gRotEncs[i]);
    bool   wasActivated = ioSwitchActivated(&gRotEncs[i].btn);
    updateStrip(&gRGBBtns[gSelectedStrip].color, i, encAction > 0, encAction < 0, wasActivated);
  }
}

void handlePixUpdate()
{
  ms_time_t now = millis();
  for(uint8_t i=0; i < N_ARGB; i++) {
    if((now - gLastARGBUpdate[i]) > 50) {
      RGBStrip* color = &gRGBBtns[i].color;
      for(uint16_t j=0; j < gARGBStrips[i].numPixels(); j++) {
        gARGBStrips[i].setPixelColor(j, gARGBStrips[i].Color(
          color->red, color->green, color->blue));
      }
      gARGBStrips[i].show();
      gLastARGBUpdate[i] = now;
    }
  }

  if((now - gLastRGBBtnUpdate) > 50) {
    for(uint8_t i=0; i<N_BTNS; i++) {
      RGBStrip* color = &gRGBBtns[i].color;
      tlc.setLED(i, color->red*8, color->green*8, color->blue*8);
    }
    tlc.write();
    gLastRGBBtnUpdate = now;
  }
}

void handleSerialCmds()
{
  Serial.setTimeout(100);
  // handle serial cmds
  while(Serial.available()) {
    // MAXLEN + null term
    char cmdBuf[SERIAL_CMD_MAXLEN+1] = {0,};
    size_t cmdLen = Serial.readBytesUntil(SERIAL_CMD_DELIM, cmdBuf, SERIAL_CMD_MAXLEN);

    // 0 len cmds are boring
    if(cmdLen == 0)
      return;

    Serial.print("DEBUG: recv cmd: ");
    Serial.write(cmdBuf, cmdLen);
    Serial.println();

    char cmd = cmdBuf[0];
    char *argv[SERIAL_CMD_MAXARGS] = {0,};
    uint8_t argc = 0;
    uint8_t lastArgStart = 1;

    // parse args
    for(int i=1; i < cmdLen; i++) {
      // find delim, set ptr
      if(cmdBuf[i] == SERIAL_CMD_ARG_DELIM) {
        argv[argc++] = &cmdBuf[lastArgStart];
        lastArgStart = i+1;
        // replace delim with null term
        cmdBuf[i] = '\0';
      }
    }

    // last arg will not have a delim after it
    if(lastArgStart < cmdLen) {
      argv[argc++] = &cmdBuf[lastArgStart];
      cmdBuf[cmdLen] = '\0';
    }

    switch(cmd) {
      case SERIAL_CMD_COLOR:
        Serial.print("rgb=(");
        Serial.print(atoi(argv[1]));
        Serial.print(",");
        Serial.print(atoi(argv[2]));
        Serial.print(",");
        Serial.print(atoi(argv[3]));
        Serial.println(")");
        /* gRGB[0] = (uint8_t)atoi(argv[1]); */
        /* gRGB[1] = (uint8_t)atoi(argv[2]); */
        /* gRGB[2] = (uint8_t)atoi(argv[3]); */
        break;
      case SERIAL_CMD_BTN_COLOR:
        Serial.println("---BTN_COLOR_UPDATE---");
        break;

      default:
        Serial.println("---INVALID_CMD---");
    }
  }
}

void updateSelectedStrip()
{
  for(uint8_t i=0; i < N_BTNS; i++) {
    if(ioSwitchActivated(&gRGBBtns[i].btn)) {
      gSelectedStrip = i;
      Serial.print("select:");
      Serial.println(i);
    }
  }
}

void loop()
{
  updateSelectedStrip();
  handleRotEncUpdates();
  handleSerialCmds();
  handlePixUpdate();
}
