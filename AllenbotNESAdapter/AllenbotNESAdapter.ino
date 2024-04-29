#pragma GCC optimize("O3")
#pragma GCC push_options

#include <Keyboard.h>
#include <KeyboardLayout.h>

//Connector (Connect also GND and 5V):  CLOCK, LATCH,     DATA
const uint8_t inputPinsPort1[] = { 2, 3, 4 };  //change these as necessary

#define CLOCK1 inputPinsPort1[0]
#define LATCH1 inputPinsPort1[1]
#define DATA1 inputPinsPort1[2]

void setupJoysticks() {
  pinMode(LATCH1, OUTPUT);
  pinMode(CLOCK1, OUTPUT);
  pinMode(DATA1, INPUT_PULLUP);
}

#define latch_low digitalWrite(LATCH1, LOW)
#define latch_high digitalWrite(LATCH1, HIGH)
#define clock_low digitalWrite(CLOCK1, LOW)
#define clock_high digitalWrite(CLOCK1, HIGH)
#define wait delayMicroseconds(12)
// read + rising & falling edge + aduino us error + propagation padding
#define waitfullread delayMicroseconds(240)
// rising & falling edge + aduino us error + propagation padding
#define waitread delayMicroseconds(36)

static uint8_t previousState = 0;
static uint8_t lastKnownGoodState = 0;
volatile uint8_t currentState = 0;

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

#define KEY_X 0x78  // Rotate clockwise
#define KEY_Z 0x7A  // Rotate counter-clockwise

constexpr uint8_t keyMapKeys[8]{
  KEY_X,           // NES Controller A Button
  KEY_Z,           // NES Controller B Button
  KEY_ESC,         // NES Controller Select Button
  KEY_RETURN,      // NES Controller Enter Button
  KEY_UP_ARROW,    // NES Controller Up Button
  KEY_DOWN_ARROW,  // NES Controller Down Button
  KEY_LEFT_ARROW,  // NES Controller Left Button
  KEY_RIGHT_ARROW  // NES Controller Right Button
};

void setup() {
  Keyboard.begin();
  setupJoysticks();
}

static unsigned long apress = 0;
static unsigned long arelease = millis();
static unsigned long bpress = 0;
static unsigned long brelease = millis();

void processInput(uint8_t currentStates, uint8_t changedStates) __attribute((always_inline));
void processInput(uint8_t currentStates, uint8_t changedStates) {
  for (int i = 0; i < 8; i++) {
    if ((changedStates >> i) & 0b00000001) {
      if (((currentStates >> i) & 0b00000001)) {
        if (i == 0) {
          if (apress == 0) {
            apress = millis();
          } else {
            arelease = millis();

            if ((arelease - apress) > 32) {
              Keyboard.press(keyMapKeys[0]);
              apress = millis();
            }
          }
        } else if (i == 1) {
          if (apress == 0) {
            bpress = millis();
          } else {
            brelease = millis();
            if ((brelease - bpress) > 32) {
              Keyboard.press(keyMapKeys[1]);
              bpress = millis();
            }
          }
        } else {
          Keyboard.press(keyMapKeys[i]);
        }
      } 
      else {
        if (i == 0) {
          apress = millis();
          Keyboard.release(keyMapKeys[0]);
        }
        else if (i == 1) {
          bpress = millis();
          Keyboard.release(keyMapKeys[1]);
        }
        else {
          Keyboard.release(keyMapKeys[i]);
        }
      }
    }
  }
}

void readController(uint8_t *state) __attribute((always_inline));
void readController(uint8_t *state) {
  latch_low;
  clock_low;
  latch_high;
  wait;
  latch_low;

  for (int i = 0; i < 8; i++) {
    *state |= (!digitalRead(DATA1) << i);
    clock_high;
    wait;
    clock_low;
    wait;
  }
}

static unsigned long previousTime = micros();
static unsigned long currentTime = micros();

static constexpr unsigned long pollInterval = 4000;     // 4000 microseconds
static constexpr unsigned long badStateInterval = 999;  // 999 microseconds

void loop() 
{
  currentState = 0;

  if (currentTime - previousTime > pollInterval) {
   
    readController(&currentState);

    uint8_t changedButtonStates = currentState ^ previousState;

    uint8_t output[2];
    memcpy(&output[0], &changedButtonStates, 1);
    memcpy(&output[1], &currentState, 1);

    if (currentTime - micros() > badStateInterval)
      processInput(output[1], output[0]);

    previousState = currentState;
    previousTime = currentTime;
  }
  currentTime = micros();
}

#pragma GCC pop_options