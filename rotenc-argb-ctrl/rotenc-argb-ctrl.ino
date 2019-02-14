/**************************************************************************
Rotary Encoder with ARGB ctrl
**************************************************************************/

#include <stdbool.h>
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define ARGB_PIN   A0
#define NUM_LEDS   8
#define BRIGHTNESS 50

#define N_ROTENC 3
#define ROTENC_PINS { \
    2, 3, 4, \
    5, 6, 7, \
    8, 9, 10 }

#define MAX_LED_VAL         4095
#define N_ENCODER_STEPS     24
#define LED_INC             (MAX_LED_VAL / N_ENCODER_STEPS / 2)
#define ON_MSG              "---ARGBCTRL-ON---"
#define CW_CHAR             '>'
#define CCW_CHAR            '<'
#define SW_CHAR             'v'
#define SW_DEBOUNCE_MS      5

#define PORTID_0 PIND
#define PORTID_1 PINB
#define PORTID_2 PINC

// Rotary encoder struct
typedef struct  {
// TODO: Define pins with less bits?
  uint8_t aPin;         // A pin of rotary encoder
  uint8_t aPort;        // A port register for fast read access
  uint8_t bPin;         // B pin of rotary encoder
  uint8_t bPort;        // B port register for fast read access
  uint8_t swPin;        // Switch pin
  uint8_t swPort;       // Switch port register for fast read access
  uint8_t state;        // Encoder state
  uint8_t prevPos;      // last encoder position
  unsigned long swLastChangeMs; // Millisecond timestamp of switch change
  bool swWasPressed;    // flag for switch press
} RotEnc;

static RotEnc   gRotEncs[N_ROTENC] = {0,};
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, ARGB_PIN, NEO_GRB + NEO_KHZ800);
static unsigned long lastPixUpdate = 0;
static uint8_t gRGB[3] = {0,};

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
  Serial.println("---BAD_PIN_PORT---");
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
      Serial.println("---BAD_PORT_ID---");
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
    gRotEncs[i].swPin  = rotEncPins[i*3+2];
    gRotEncs[i].swPort = calcPinPort(gRotEncs[i].swPin);

    pinMode(gRotEncs[i].aPin, INPUT_PULLUP);
    pinMode(gRotEncs[i].bPin, INPUT_PULLUP);
    pinMode(gRotEncs[i].swPin, INPUT_PULLUP);

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
    Serial.print(gRotEncs[i].swPin);
    Serial.println("}");
  }

  Serial.print("---Setup ");
  Serial.print(N_ROTENC, DEC);
  Serial.println(" RotEncs---");
}

void setupARGB()
{
  strip.setBrightness(BRIGHTNESS);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
}

void setup()
{
  Serial.begin(9600);
  Serial.println(ON_MSG);
  setupEncoder();
  setupARGB();
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
  if (enc_cur_pos != rotEnc->prevPos)
  {
    if (rotEnc->prevPos == 0x00)
    {
      // this is the first edge
      if (enc_cur_pos == 0x01) {
        rotEnc->state |= (1 << 0);
      }
      else if (enc_cur_pos == 0x02) {
        rotEnc->state |= (1 << 1);
      }
    }

    if (enc_cur_pos == 0x03)
    {
      // this is when the encoder is in the middle of a "step"
      rotEnc->state |= (1 << 4);
    }
    else if (enc_cur_pos == 0x00)
    {
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
      if (bit_is_set(rotEnc->state, 0) && (bit_is_set(rotEnc->state, 2) || bit_is_set(rotEnc->state, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(rotEnc->state, 2) && (bit_is_set(rotEnc->state, 0) || bit_is_set(rotEnc->state, 4))) {
        enc_action = 1;
      }
      else if (bit_is_set(rotEnc->state, 1) && (bit_is_set(rotEnc->state, 3) || bit_is_set(rotEnc->state, 4))) {
        enc_action = -1;
      }
      else if (bit_is_set(rotEnc->state, 3) && (bit_is_set(rotEnc->state, 1) || bit_is_set(rotEnc->state, 4))) {
        enc_action = -1;
      }

      rotEnc->state = 0; // reset for next time
    }
  }

  rotEnc->prevPos = enc_cur_pos;
  return enc_action;
}

void handleRotEncUpdates()
{
  unsigned long now = millis();

  // loop through encoders
  for(uint8_t i=0; i < N_ROTENC; i++) {
    // handle rotation updates
    int8_t enc_action = getEncoderAction(&gRotEncs[i]);
    if (enc_action > 0) { // Clockwise
      if(gRGB[i] < 255)
        gRGB[i]++;
      Serial.print(i, DEC);
      Serial.println(CW_CHAR);
    } else if (enc_action < 0) { // Counter-clockwise
      if(gRGB > 0)
        gRGB[i]--;
      Serial.print(i, DEC);
      Serial.println(CCW_CHAR);
    }

    // handle rotation updates
    uint8_t swPort = portFromId(gRotEncs[i].swPort);
    // handle millis() rollover after 50 days https://www.arduino.cc/reference/en/language/functions/time/millis/
    if(now < gRotEncs[i].swLastChangeMs)
       gRotEncs[i].swLastChangeMs = 0;
    // only handle changes once we are out of the debounce delay
    if((now - gRotEncs[i].swLastChangeMs) >= SW_DEBOUNCE_MS) {
      if(bit_is_clear(swPort, gRotEncs[i].swPin % 8)) {
        // only fire on the first press
        if(!gRotEncs[i].swWasPressed) {
          gRGB[i] = 0;
          Serial.print(i, DEC);
          Serial.println(SW_CHAR);
          gRotEncs[i].swLastChangeMs = now;
          gRotEncs[i].swWasPressed = true;
        }
      } else {
        // debounce check
        gRotEncs[i].swWasPressed = false;
        gRotEncs[i].swLastChangeMs = now;
      }
    }
  }
}

void loop()
{
  unsigned long now = millis();
  handleRotEncUpdates();

  if((now - lastPixUpdate) > 10) {
    for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, strip.Color(gRGB[0], gRGB[1], gRGB[2]));
    }
    strip.show();
    lastPixUpdate = now;
  }
}
