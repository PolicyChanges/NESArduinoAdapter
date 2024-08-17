#pragma GCC optimize("O3")
#pragma GCC push_options

#include <XInput.h>

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


#define NES_A       B00000001
#define NES_B       B00000010
#define NES_SELECT  B00000100
#define NES_START   B00001000
#define NES_UP      B00010000
#define NES_DOWN    B00100000
#define NES_LEFT    B01000000
#define NES_RIGHT   B10000000


#define currentTime micros()

#define KEY_W 0x77
#define KEY_A 0x97
#define KEY_S 0x73
#define KEY_D 0x64

#define KEY_F 0x66
#define KEY_J 0x6A

#define KEY_X 0x78
#define KEY_Z 0x7A

#define LINUX
#define TEC_DEFAULT

#ifdef TEC_DEFAULT
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
#else
constexpr u8 keyMapKeys[8]
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
#endif

struct debounceButton
{
#ifdef LINUX  // Note: cdc_acm - HZ/CLK_TICK = 100
  u32 buttonPressedInterval[8]  = {16000, 16000, 16000, 16000, 16000, 16000, 16000, 16000};
  u32 buttonReleasedInterval[8] = {16000, 16000, 16000, 16000, 16000, 16000, 16000, 16000};
#else
  u32 buttonPressedInterval[8]  = {16000, 16000, 0, 0, 0, 0, 0, 0};
  u32 buttonReleasedInterval[8] = {16000, 16000, 0, 0, 0, 0, 0, 0};
#endif

  u32 buttonPressedTimeStamp[8];
  u32 buttonReleasedTimeStamp[8];
} debounce;

void setup() 
{
  XInput.begin();
  setupJoysticks();
  
  for(int i = 0; i < 8; i++) {
    debounce.buttonPressedTimeStamp[i] = micros();
    debounce.buttonReleasedTimeStamp[i] = micros();
  }
}

void processInput(u8 currentStates, u8 changedStates) 
{
  for (int i = 0; i < 8; i++) {
    if ((changedStates >> i) & 0b00000001) {
      if (((currentStates >> i) & 0b00000001)) {
        if (currentTime - debounce.buttonPressedTimeStamp[i] > debounce.buttonPressedInterval[i]) {
          XInput.press(xinputMapKeys[i]);
          debounce.buttonPressedTimeStamp[i] = micros();
        }
      }
      else if (currentTime - debounce.buttonReleasedTimeStamp[i] > debounce.buttonReleasedInterval[i]) {
        XInput.release(xinputMapKeys[i]);
        debounce.buttonReleasedTimeStamp[i] = micros();
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
    state |= (!digitalRead(DATA1) << i);
    clock_high;
    wait;
    clock_low;
    wait;
  }
}

static unsigned long previousTime = micros();

constexpr unsigned long pollInterval = 2000;     // 2000 microseconds

#ifdef TEC_DEFAULT
bool handleTECInput();
#endif

void loop() 
{
  currentState = 0;

  if (currentTime - previousTime > pollInterval) {
   
    readController(currentState);

    u8 changedButtonStates = currentState ^ previousState;
#ifdef TEC_DEFAULT
  //if(handleTECInput() == false)
#endif
    processInput(currentState, previousState);

    previousState = currentState;
    previousTime = currentTime;
  }
}

#ifdef asdf //TEC_DEFAULT
bool handleTECInput() 
{
  bool isHandlingInput = false;
  
  if((previousState & NES_SELECT) && !(currentState & NES_SELECT))   // if select and another button was pressed in the previous input, release all
    Keyboard.releaseAll();
  else if(currentState & NES_SELECT)
  {
    isHandlingInput = true;

#define KEY_1 0x31

    if(currentState & NES_UP && !(previousState & NES_UP)) {
      Keyboard.press(KEY_1);
      Keyboard.press(KEY_UP_ARROW);
      Keyboard.release(KEY_1);
      Keyboard.release(KEY_UP_ARROW);
    }
    if (currentState & NES_DOWN && !(previousState & NES_DOWN)) {
      Keyboard.press(KEY_1);
      Keyboard.press(KEY_DOWN_ARROW);
      Keyboard.release(KEY_1);
      Keyboard.release(KEY_DOWN_ARROW);
    } 
    
    if (currentState & NES_LEFT && !(previousState & NES_LEFT)) {
      Keyboard.press(KEY_1);
      Keyboard.press(KEY_LEFT_ARROW);
      Keyboard.release(KEY_1);
      Keyboard.release(KEY_LEFT_ARROW);
    } 
    
    if (currentState & NES_RIGHT && !(previousState & NES_RIGHT)){
      Keyboard.press(KEY_1);
      Keyboard.press(KEY_RIGHT_ARROW);
      Keyboard.release(KEY_1);
      Keyboard.release(KEY_RIGHT_ARROW);
    }

    if((previousState & NES_A) && !(currentState & NES_A))
      Keyboard.release(0x77);
    else if(currentState & NES_A) 
      Keyboard.press(0x77); // W key
    
    if((previousState & NES_B) && !(currentState & NES_B))
      Keyboard.release(0x73);
    else if(currentState & NES_B)
      Keyboard.press(0x73); // S key 
    
    
    if((previousState & NES_START) && !(currentState & NES_START))
      Keyboard.release(KEY_ESC);
    else if(currentState & NES_START)
      Keyboard.press(KEY_ESC);
  }   

  return isHandlingInput;
}
#endif

#pragma GCC pop_options