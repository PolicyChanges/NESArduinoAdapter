#pragma GCC optimize("O3")
#pragma GCC push_options

#include <Keyboard.h>
#include <KeyboardLayout.h>

//Connector (Connect also GND and 5V):  CLOCK, LATCH,     DATA
constexpr u8 inputPinsPort1[] = { 2, 3, 4 };  //change these as necessary

#define CLOCK1 inputPinsPort1[0]
#define LATCH1 inputPinsPort1[1]
#define DATA1 inputPinsPort1[2]

void setupJoysticks()
{
  pinMode(LATCH1, OUTPUT);
  pinMode(CLOCK1, OUTPUT);
  pinMode(DATA1, INPUT_PULLUP);
}

#define latch_low digitalWrite(LATCH1, LOW)
#define latch_high digitalWrite(LATCH1, HIGH)
#define clock_low digitalWrite(CLOCK1, LOW)
#define clock_high digitalWrite(CLOCK1, HIGH)
#define wait delayMicroseconds(12)
#define waitfullread delayMicroseconds(240)
#define waitread delayMicroseconds(36)

static u8 previousState = 0;
static u8 currentState = 0;

/* Reference
#define NES_A       B00000001
#define NES_B       B00000010
#define NES_SELECT  B00000100
#define NES_START   B00001000
#define NES_UP      B00010000
#define NES_DOWN    B00100000
#define NES_LEFT    B01000000
#define NES_RIGHT   B10000000
*/

#define currentTime micros()

#define KEY_W 0x77
#define KEY_A 0x97
#define KEY_S 0x73
#define KEY_D 0x64

#define KEY_F 0x66
#define KEY_J 0x6A

constexpr u8 keyMapKeys[8]
{
  KEY_LEFT_ARROW,           // NES Controller A Button
  KEY_RIGHT_ARROW,           // NES Controller B Button
  KEY_F,         // NES Controller Select Button
  KEY_J,           // NES Controller Enter Button
  KEY_W,    // NES Controller Up Button
  KEY_S,  // NES Controller Down Button
  KEY_A,  // NES Controller Left Button
  KEY_D  // NES Controller Right Button
};

void setup() 
{
  Keyboard.begin();
  setupJoysticks();
}

static u32 APressedTimeStamp = micros();
static u32 BPressedTimeStamp = micros();

constexpr u32 clampInterval = 32000;

void processInput(u8 currentStates, u8 changedStates) __attribute((always_inline));
void processInput(u8 currentStates, u8 changedStates) 
{
  for (int i = 0; i < 8; i++) {
    if ((changedStates >> i) & 0b00000001) {
      if (((currentStates >> i) & 0b00000001)) {
        if (i == 0) {
          if (currentTime - APressedTimeStamp > clampInterval) {
            Keyboard.press(keyMapKeys[0]);
            APressedTimeStamp = micros();
          }
        } else if (i == 1) {
          if (currentTime - BPressedTimeStamp > clampInterval) {
            Keyboard.press(keyMapKeys[1]);
            BPressedTimeStamp = micros();
          }
        } else 
          Keyboard.press(keyMapKeys[i]);
      } 
      else {
        if (i == 0) {
          Keyboard.release(keyMapKeys[0]);
          APressedTimeStamp = micros();
        }
        else if (i == 1) {
          Keyboard.release(keyMapKeys[1]);
          BPressedTimeStamp = micros();
        }
        else {
          Keyboard.release(keyMapKeys[i]);
        }
      }
    }
  }
}

void readController(u8 &state) __attribute((always_inline));
void readController(u8 &state) 
{
  latch_low;
  clock_low;
  latch_high;
  wait;
  latch_low;

  for (int i = 0; i < 8; i++) {
    state |= (!digitalRead(DATA1) << i);
    clock_high;
    wait;
    clock_low;
    wait;
  }
}

static unsigned long previousTime = micros();

constexpr unsigned long pollInterval = 2000;     // 2000 microseconds

void loop() 
{
  currentState = 0;

  if (currentTime - previousTime > pollInterval) {
   
    readController(currentState);

    u8 changedButtonStates = currentState ^ previousState;

    processInput(currentState, previousState);

    previousState = currentState;
    previousTime = currentTime;
  }
}

#pragma GCC pop_options