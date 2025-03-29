#pragma GCC optimize("Ofast")
#pragma GCC push_options

#include <Keyboard.h>

#define TEC_DEFAULT

//Connector (Connect also GND and 5V):  CLOCK, LATCH, DATA
static constexpr u8 inputPinsPort1[] = { 2, 3, 4 };  //change these as necessary

#define CLOCK inputPinsPort1[0]
#define LATCH inputPinsPort1[1]
#define DATA inputPinsPort1[2]

static void setupJoysticks() {
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DATA, INPUT_PULLUP);
}

#define currentTime micros()

#define bounceIntervalMicros 54000

// Current state of buttons stored as boolean bits
//static u8 currentState = 0;
// State of the controller as it was on previous interval.
static u8 previousState = 0;
// Timestamp of previous interval
static unsigned long previousTime = currentTime;
// pollInterval is the interval between reading controller. loop() runs at 16MHz
static constexpr unsigned long pollInterval = 0;
static constexpr char* nameIdx[8] = { "A", "B", "Select", "Start", "Up", "Down", "Left", "Right" };

#define NES_A B00000001
#define NES_B B00000010
#define NES_SELECT B00000100
#define NES_START B00001000
#define NES_UP B00010000
#define NES_DOWN B00100000
#define NES_LEFT B01000000
#define NES_RIGHT B10000000



#define latch_low digitalWrite(LATCH, LOW)
#define latch_high digitalWrite(LATCH, HIGH)
#define clock_low digitalWrite(CLOCK, LOW)
#define clock_high digitalWrite(CLOCK, HIGH)

#define wait delayMicroseconds(12)

// Emulator Keys
#define KEY_W 0x77
#define KEY_A 0x97
#define KEY_S 0x73
#define KEY_D 0x64

#define KEY_F 0x66
#define KEY_J 0x6A

// TEC Keys
#define KEY_X 0x78
#define KEY_Z 0x7A

#define KEY_1 0x31
#define KEY_W 0x77
#define KEY_S 0x73

#ifdef TEC_DEFAULT
inline bool isHandlingTECInput();

static constexpr u8 keyMapKeys[8]{
  KEY_X,           // NES Controller A Button
  KEY_Z,           // NES Controller B Button
  0x0,             // NES Controller Select Button
  KEY_RETURN,      // NES Controller Enter Button
  KEY_UP_ARROW,    // NES Controller Up Button
  KEY_DOWN_ARROW,  // NES Controller Down Button
  KEY_LEFT_ARROW,  // NES Controller Left Button
  KEY_RIGHT_ARROW  // NES Controller Right Button
};
#else

constexpr bool isHandlingTECInput()
{
  return false;
}
static constexpr u8 keyMapKeys[8]
{
  KEY_LEFT_ARROW,   // NES Controller A Button
  KEY_RIGHT_ARROW,  // NES Controller B Button
  KEY_F,            // NES Controller Select Button
  KEY_J,            // NES Controller Enter Button
  KEY_W,            // NES Controller Up Button
  KEY_S,            // NES Controller Down Button
  KEY_A,            // NES Controller Left Button
  KEY_D             // NES Controller Right Button
};
#endif TEC_DEFAULT

static unsigned long long previousChangedStateTimestamp[8];

void readController() [[force_inline]];

void setup() {
#if defined(PROFILE) || defined(PROFILE_BUTTONS)
  Serial.begin(9600);
#endif

  Keyboard.begin();
  setupJoysticks();

  // Initialize input timestamps
  for (int i = 0; i < 8; i++)
    previousChangedStateTimestamp[i] = currentTime;

  attachInterrupt(digitalPinToInterrupt(LATCH), readControllerRaw, FALLING);
  attachInterrupt(digitalPinToInterrupt(CLOCK), readControllerRaw, FALLING);
}

void processInput(u8 currentState) 
{
  const unsigned long processInputCurrentTimestamp = currentTime;
  u8 changedStates = currentState ^ previousState;

  for (int i = 0; i < 8; i++) {
    // don't debounce left and right presses
    if ((i == NES_LEFT || i == NES_RIGHT) == false) {
      const unsigned long delta = processInputCurrentTimestamp - previousChangedStateTimestamp[i];
        // check for bounce unless checking
      if (((currentState >> i) & 0b00000001) && delta < bounceIntervalMicros) {
        Serial.println(String(nameIdx[i]) + " button cancelled. Last state change " + String(delta) + " microseconds ago.");
        changedStates &= ~(1 << i);
        previousChangedStateTimestamp[i] = processInputCurrentTimestamp;
      }
    }
    if ((changedStates >> i) & 0b00000001) {
      if (((currentState >> i) & 0b00000001)) {
        Keyboard.press(keyMapKeys[i]);
        previousChangedStateTimestamp[i] = processInputCurrentTimestamp;
      } else 
            Keyboard.release(keyMapKeys[i]);
    }
  }
}


volatile unsigned long long buttonEventIDRaw = 0;
volatile u8 buttonStateRaw = 0;

inline void readControllerRaw() 
{
  buttonStateRaw |= (!digitalRead(DATA) << (buttonEventIDRaw % 8));
}

inline void signalReadControllerInterrupt() 
{
  buttonStateRaw = 0;
  // Read A button
  latch_high;
  wait;
  latch_low;
  buttonEventIDRaw++;
  //Read remaining buttons
  for (int i = 1; i < 8; i++) {
    clock_high;
    wait;
    clock_low;
    wait;
    buttonEventIDRaw++;
  }
}

void loop() 
{
  const unsigned long currentLoopTimestamp = currentTime;
  const unsigned long delta = currentLoopTimestamp - previousTime;
  if (delta >= pollInterval) {
    signalReadControllerInterrupt();
    u8 changedStates = buttonStateRaw ^ previousState;

    if (changedStates && isHandlingTECInput(buttonStateRaw) == false)
      processInput(buttonStateRaw);

    previousState = buttonStateRaw;
    previousTime = currentLoopTimestamp;
  }
}

#ifdef TEC_DEFAULT
bool isHandlingTECInput(u8 state) 
{
  bool isHandlingInput = false;

  if ((previousState & NES_SELECT) && !(state & NES_SELECT))  // if select and another button was pressed in the previous input, release all
    Keyboard.releaseAll();
  else if (state & NES_SELECT) {
    isHandlingInput = true;

    // Emotes
    if (state & NES_UP && !(previousState & NES_UP)) {
      Keyboard.press(KEY_1);
      Keyboard.press(KEY_UP_ARROW);
      Keyboard.release(KEY_1);
      Keyboard.release(KEY_UP_ARROW);
    }
    if (state & NES_DOWN && !(previousState & NES_DOWN)) {
      Keyboard.press(KEY_1);
      Keyboard.press(KEY_DOWN_ARROW);
      Keyboard.release(KEY_1);
      Keyboard.release(KEY_DOWN_ARROW);
    }

    if (state & NES_LEFT && !(previousState & NES_LEFT)) {
      Keyboard.press(KEY_1);
      Keyboard.press(KEY_LEFT_ARROW);
      Keyboard.release(KEY_1);
      Keyboard.release(KEY_LEFT_ARROW);
    }

    if (state & NES_RIGHT && !(previousState & NES_RIGHT)) {
      Keyboard.press(KEY_1);
      Keyboard.press(KEY_RIGHT_ARROW);
      Keyboard.release(KEY_1);
      Keyboard.release(KEY_RIGHT_ARROW);
    }

    // Zoom in
    if (!(previousState & NES_A) && (state & NES_A))
      Keyboard.press(KEY_W);
    else if ((previousState & NES_A) && !(state & NES_A))
      Keyboard.release(KEY_W);

    // Zoom out
    if (!(previousState & NES_B) && (state & NES_B))
      Keyboard.press(KEY_S);
    else if ((previousState & NES_B) && !(state & NES_B))
      Keyboard.release(KEY_S);

    // Escape key/back button
    if (!(previousState & NES_START) && (state & NES_START))
      Keyboard.press(KEY_ESC);
    else if ((previousState & NES_START) && !(state & NES_START))
      Keyboard.release(KEY_ESC);
  }
  return isHandlingInput;
}
#else
constexpr bool isHandlingTECInput(u8 state) 
{
  return false;
}
#endif TEC_DEFAULT

#pragma GCC pop_options