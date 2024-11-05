#pragma GCC optimize("Ofast")
#pragma GCC push_options
#include <XInput.h>

//Connector (Connect also GND and 5V):  CLOCK, LATCH,     DATA
constexpr u8 inputPinsPort1[] = { 2, 3, 4 };  //change these as necessary

#define CLOCK inputPinsPort1[0]
#define LATCH inputPinsPort1[1]
#define DATA inputPinsPort1[2]

void setupJoysticks()
{
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);
  pinMode(DATA, INPUT_PULLUP);
}

#define latch_low digitalWrite(LATCH, LOW)
#define latch_high digitalWrite(LATCH, HIGH)
#define clock_low digitalWrite(CLOCK, LOW)
#define clock_high digitalWrite(CLOCK, HIGH)

#define wait delayMicroseconds(12)


static u8 previousState = 0;
static u8 currentState = 0;

#define NES_A       B00000001
#define NES_B       B00000010
#define NES_SELECT  B00000100
#define NES_START   B00001000
#define NES_UP      B00010000
#define NES_DOWN    B00100000
#define NES_LEFT    B01000000
#define NES_RIGHT   B10000000

#define currentTime micros()

static constexpr u8 xinputMapKeys[8]{
  BUTTON_B,
  BUTTON_A,
  BUTTON_BACK,
  BUTTON_START,
  DPAD_UP,
  DPAD_DOWN,
  DPAD_LEFT,
  DPAD_RIGHT
};

struct debounceButton
{
  u32 buttonPressedInterval[8]  = {31992, 31992, 31992, 31992, 31992, 31992, 31992, 31992};
  u32 buttonReleasedInterval[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  u32 buttonPressedTimeStamp[8];
  u32 buttonReleasedTimeStamp[8];
} debounce;

void setup() 
{
  //Serial.begin(1000000);
  XInput.begin();
  setupJoysticks();

  for(int i = 0; i < 8; i++) {
    debounce.buttonPressedTimeStamp[i] = currentTime;
    debounce.buttonReleasedTimeStamp[i] = currentTime;
  }
}

void processInput(u8 currentStates, u8 changedStates) 
{
  const unsigned long inputCurrentTime = currentTime;
  for (int i = 0; i < 8; i++) {
    if ((changedStates >> i) & 0b00000001) {
      if (((currentStates >> i) & 0b00000001)) {
        if (inputCurrentTime - debounce.buttonPressedTimeStamp[i] > debounce.buttonPressedInterval[i]) {
          XInput.press(xinputMapKeys[i]);
          debounce.buttonPressedTimeStamp[i] = currentTime;
        }
      }
      else if (inputCurrentTime - debounce.buttonReleasedTimeStamp[i] > debounce.buttonReleasedInterval[i]) {
        XInput.release(xinputMapKeys[i]);
        debounce.buttonReleasedTimeStamp[i] = currentTime;
      }
    }
  } 
}

void readController(u8 &state) 
{
  latch_low;
  clock_low;
  latch_high;
  wait;
  latch_low;

  for (int i = 0; i < 8; i++) {
    state |= (!digitalRead(DATA) << i);
    clock_high;
    wait;
    clock_low;
    wait;
  }
}

static unsigned long previousTime = currentTime;

constexpr unsigned long pollInterval = 2000;     // 2000 microseconds

void loop() 
{
  currentState = 0;
  
  if (currentTime - previousTime > pollInterval) {
   
    readController(currentState);

    u8 changedButtonStates = currentState ^ previousState;

    if(handleTECInput() == false)
      processInput(currentState, previousState);

    //if(previousState != currentState)
    //  Serial.println("State: "  + String(currentState));
    previousState = currentState;
    previousTime = currentTime;
  }
}

bool handleTECInput() 
{
  bool isHandlingInput = false;
  
    // release all if start or select were released
  if(((previousState & NES_SELECT) && !(currentState & NES_SELECT)) || (previousState & NES_START) && !(currentState & NES_START))
    XInput.releaseAll();
  else if(currentState & NES_SELECT)
  {
    isHandlingInput = true;

    // send button x or y if select and start are pressed with a/b
    if(currentState & NES_START) {
      if(currentState & NES_A && !(previousState & NES_A))
        XInput.press(XInputControl::BUTTON_X);
      else if(!(currentState & NES_A) && previousState & NES_A)
        XInput.release(XInputControl::BUTTON_X);
      if(currentState & NES_B && !(previousState & NES_B))
        XInput.press(XInputControl::BUTTON_Y);
      else if(!(currentState & NES_B) && previousState & NES_B)
        XInput.release(XInputControl::BUTTON_Y);
    }else{
      // playfield zoom -- select + a/b 
      XInput.setJoystick(XInputControl::JOY_LEFT, currentState & NES_B, currentState & NES_A, false, false, false);  
      // emotes -- select + directional pad
      XInput.setJoystick(XInputControl::JOY_RIGHT,
                              currentState & NES_UP && !(previousState & NES_UP), 
                              currentState & NES_DOWN && !(previousState & NES_DOWN),                   
                              currentState & NES_LEFT && !(previousState & NES_LEFT), 
                              currentState & NES_RIGHT && !(previousState & NES_RIGHT),
                              false);
    }
  }   
  return isHandlingInput;
}

#pragma GCC pop_options