//add to board manager
//https://raw.githubusercontent.com/dmadison/ArduinoXInput_Boards/master/package_dmadison_xinput_index.json

/* 
Startup Modes:
a+b = 50hz debouncing
select+right = goofy foot controller
select+down  = emulator friendly keyboard bindings
select+start = loop function select -- testing purposes

See Readme.md for more information
*/

// Comment out below and switch board to xinput(from url above) to act as an xbox controller


//#define KEYBOARD

#pragma GCC optimize("O3")
#pragma GCC push_options

// Reference
#define NES_A       B00000001
#define NES_B       B00000010
#define NES_SELECT  B00000100
#define NES_START   B00001000
#define NES_UP      B00010000
#define NES_DOWN    B00100000
#define NES_LEFT    B01000000
#define NES_RIGHT   B10000000

//Connector (Connect also GND and 5V):  CLOCK, LATCH,     DATA
constexpr const u8 inputPinsPort1[] = { 2, 3, 4 };  //change these as necessary

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
static u8 updateState = 0;
static uint8_t SELECT_START_A;
static uint8_t SELECT_START_B;

void readController(u8 &state) __attribute((always_inline));

#ifdef KEYBOARD
  #include <Keyboard.h>
#else
  #include <XInput.h>
#endif

#define KEY_X 0x78
#define KEY_Z 0x7A
#define KEY_W 0x77
#define KEY_S 0x73
#define KEY_Q 0x71
#define KEY_R 0x72
#define KEY_I 0x69
#define KEY_J 0x6A

#ifdef KEYBOARD

enum XInputControl : uint8_t
{
  BUTTON_B = KEY_X,
  BUTTON_A = KEY_Z,
  BUTTON_BACK = KEY_LEFT_SHIFT,
  BUTTON_START = KEY_RETURN,
  DPAD_UP = KEY_UP_ARROW,
  DPAD_DOWN = KEY_DOWN_ARROW,
  DPAD_LEFT = KEY_LEFT_ARROW,
  DPAD_RIGHT = KEY_RIGHT_ARROW,
	BUTTON_X = KEY_ESC,
	BUTTON_Y = KEY_LEFT_CTRL, 
  TRIGGER_LEFT,
	TRIGGER_RIGHT, 
  JOY_LEFT,
	JOY_RIGHT
};

#define KEY_1 0x31

class Controller_ : public Keyboard_
{
public:
  void setAutoSend(bool send){return 0;}
  void setJoystick(XInputControl joy, boolean up, boolean down, boolean left, boolean right, boolean useSOCD=true) {
 
    if(joy == XInputControl::JOY_RIGHT) {
      uint8_t dir[4] {DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT};
      bool states[4] {up, down, left, right};
      for(int i = 0; i < 4; i++){
        if(states[i]){
          press(KEY_1);
          this->press(dir[i]);
        }
        else{
          release(KEY_1);
          release(dir[i]);
        }
      }
    } else if(joy == XInputControl::JOY_LEFT) {
      if(up)
        press(BUTTON_X);
      else
        release(BUTTON_X);
      if(down)
        press(BUTTON_Y);
      else
        release(BUTTON_Y);
    } 
  }
  uint8_t SELECT_START_A = KEY_W;
  uint8_t SELECT_START_B = KEY_S;
  String id = "Keyboard";
};

Controller_ *Controller = dynamic_cast<Keyboard_*>(&Keyboard); 

#else
class Controller_ : public XInputController
{
public:
  uint8_t SELECT_START_A = BUTTON_X;
  uint8_t SELECT_START_B = BUTTON_Y; 
  
  String id = "XInput";
};

Controller_ *Controller = dynamic_cast<XInputController*>(&XInput);

uint8_t KEY_ESC = 0;
#endif

constexpr u8 xinputMapKeys[8] 
{
  BUTTON_B,
  BUTTON_A,
  BUTTON_BACK,
  BUTTON_START,
  DPAD_UP,
  DPAD_DOWN,
  DPAD_LEFT,
  DPAD_RIGHT
};

constexpr u8 goofyMapButtons[8] 
{
  BUTTON_A,
  BUTTON_B,
  BUTTON_START,
  BUTTON_BACK,
  DPAD_DOWN,
  DPAD_UP,
  DPAD_RIGHT,
  DPAD_LEFT
};

constexpr u8 emuMapButtons[8] 
{
  BUTTON_A,
  BUTTON_B,
  KEY_I,
  KEY_J,
  DPAD_DOWN,
  DPAD_UP,
  DPAD_RIGHT,
  DPAD_LEFT
};

static const bool isEmuFriendlyBinds = []() -> const bool
{
    u8 startupState = 0;

    setupJoysticks();
    readController(startupState);

    if(startupState & (NES_START | NES_DOWN))
      return true;

    return false;  
}();

static const u8 *buttonsMap = []() -> const u8*
{
  if(isEmuFriendlyBinds)
    return emuMapButtons;

  u8 startupState = 0;

  setupJoysticks();
  readController(startupState);

  if(startupState & (NES_SELECT | NES_RIGHT))
    return goofyMapButtons;

  return xinputMapKeys;  
}();

// in microseconds
static const unsigned long debounce_interval = []() -> const unsigned long
{
  u8 startupState = 0;
  double video_freq = 0;

  // note: 4*4 + 4! + 8 - 1 = 47 potential startup mode combinations
  setupJoysticks();
  readController(startupState);

  if(startupState & (NES_A | NES_B))
    video_freq = 50;
  else
    video_freq = 60;
  
  double padding  = 0; 
  double num_of_frames = 1;

  return ((unsigned long)((1 / video_freq) * 1000) * num_of_frames * 1000) - padding;
}();

static struct buttonClamp
{
// In microseconds
//#define NO_DEBOUNCE
#ifdef NO_DEBOUNCE
  static constexpr const unsigned long clampDownInterval[8] { 0, 0, 0, 0, 0, 0, 0, 0 };
  static constexpr const unsigned long clampUpInterval[8]   { 0, 0, 0, 0, 0, 0, 0, 0 };
#else
  const unsigned long clampDownInterval[8] { debounce_interval, debounce_interval, debounce_interval, debounce_interval, debounce_interval, debounce_interval, debounce_interval, debounce_interval };
  const unsigned long clampUpInterval[8]   { debounce_interval, debounce_interval, debounce_interval, debounce_interval, debounce_interval, debounce_interval, debounce_interval, debounce_interval };
#endif

  unsigned long onDownStateTimeStamp[8];
  unsigned long onUpStateTimeStamp[8];
  bool isPressed[8] { false, false, false, false, false, false, false, false };
} clamp;

// 204us
void readController(u8 &state) 
{
  state = 0;
  latch_low;
  clock_low;
  latch_high;
  wait;
  latch_low;

  for (u8 i = 0; i < 8; i++) {
    state |= (!digitalRead(DATA) << i);
    clock_high;
    wait;
    clock_low;
    wait;
  }
}

void (*loopMain)();
void loopTECFunc();
void loopBasicFunc();

void setup() 
{
//#define DEBUG_KEYBOARD
#ifdef DEBUG_KEYBOARD
  Serial.begin(9600);
#endif

  Controller->setAutoSend(true); 
  Controller->begin();

  for(u8 i = 0; i < 8; i++)
    clamp.onUpStateTimeStamp[i] = clamp.onDownStateTimeStamp[i] = micros();

  u8 startupState = 0;
  readController(startupState);

  if(isEmuFriendlyBinds)
    loopMain = &loopBasicFunc;
  else
    loopMain = &loopTECFunc;
}

void processInput(const u8 currentStates, u8 &updateStates) __attribute((always_inline));
void processInput(const u8 currentStates, u8 &updateStates) 
{ // TODO: catch if select is pressed and diabled releases
  const u8 changedStates = currentStates ^ previousState;
  const unsigned long processCurrentTime = micros();
  for (u8 i = 0; i < 8; i++) {
    if((changedStates >> i) & 0b00000001)
    {
      if ((currentStates >> i) & 0b00000001) {
        if(!clamp.isPressed[i] && processCurrentTime - clamp.onUpStateTimeStamp[i] >= clamp.clampUpInterval[i]){
          updateStates |= (0b00000001 << i);
          clamp.onDownStateTimeStamp[i] = processCurrentTime;
          clamp.isPressed[i] = true;
        }
      } 
      else {
        if(clamp.isPressed[i] && processCurrentTime - clamp.onDownStateTimeStamp[i] >= clamp.clampDownInterval[i]){
          updateStates &= 0b11111111 & ~(0b00000001 << i);
          clamp.onUpStateTimeStamp[i] = processCurrentTime;
          clamp.isPressed[i] = false;
        }
      }
    }
  }
}

void updateInput(const u8 updateStates) __attribute((always_inline));
void updateInput(const u8 updateStates) 
{
  const u8 changedStates = updateStates ^ previousState;
  for (u8 i = 0; i < 8; i++) {
    if((changedStates >> i) & 0b00000001)
      if (((updateStates >> i) & 0b00000001))
        Controller->press(buttonsMap[i]); 
      else  
        Controller->release(buttonsMap[i]); 
  }
}

void handleSelect(const u8 updateStates) __attribute((always_inline));
void handleSelect(const u8 updateStates)
{
  const u8 changedStates = updateStates ^ previousState;

  if(updateStates & NES_START) { 
      if(updateStates & NES_A) 
      Controller->press(SELECT_START_A); 
      else
        Controller->release(SELECT_START_A);
    
      if(updateStates & NES_B)
        Controller->press(SELECT_START_B); 
      else
        Controller->release(SELECT_START_B);
  }

  if(!(updateStates & NES_START)) {
    Controller->setJoystick(XInputControl::JOY_LEFT, updateStates & NES_B, updateStates & NES_A, false, false, false);  // menu navigation 
    Controller->setJoystick(XInputControl::JOY_RIGHT, updateStates & NES_UP, updateStates & NES_DOWN,                   // emotes             
                            updateStates & NES_LEFT, updateStates & NES_RIGHT, false); 
  }
}

#define UNBOUNDED_POLL
#ifdef UNBOUNDED_POLL
static constexpr const unsigned long pollInterval =  0;     // microseconds
#else
static constexpr const unsigned long pollInterval =  2000;     // microseconds
#endif

static unsigned long previousTime = micros();
static unsigned long currentTime = micros();

void loopBasicFunc()
{
    if (currentTime - previousTime > pollInterval) {
    
    u8 currentState = 0;

    readController(currentState);

    if(currentState != previousState) {

      processInput(currentState, updateState);

      if(updateState != previousState) {
          updateInput(updateState);
          previousState = updateState;
          previousTime = currentTime;
      }
    }
  }
  currentTime = micros();
}

void loopTECFunc() 
{
  if (currentTime - previousTime > pollInterval) {
    
    u8 currentState = 0;

    readController(currentState);

    if(currentState != previousState) {
        
      processInput(currentState, updateState);

      if(updateState != previousState) {
        // Release select
        if((previousState & NES_SELECT) && (~updateState & NES_SELECT)) [[unlikely]] {
          Controller->setJoystick(XInputControl::JOY_RIGHT, false, false, false, false, false);  // XInput library handles state breaking well. leave it.
          Controller->setJoystick(XInputControl::JOY_LEFT, false, false, false, false, false);
          Controller->releaseAll();

          previousState = updateState = 0;
          memset((void*)(clamp.isPressed), 0, sizeof(bool) * 8);
        }

        if((updateState & NES_SELECT) == false) [[likely]] {
          updateInput(updateState); 
        }
        else
          handleSelect(updateState);

        previousState = updateState;
        previousTime = currentTime;
      }
    }
  }
  currentTime = micros();
}



void loop() 
{

#if 0
static uint_least64_t sum_of_difference = 0;
static uint_least64_t start = 0;
static uint_least64_t end = 0;
static uint_least64_t n_samples = 0;
 // start = (double)micros();
#endif

  loopMain();

#if 0
  end = (uint_least64_t)micros();
  uint_least64_t delta = end - start;
  sum_of_difference += delta;
  n_samples++;
  
  //if(sum_of_difference % (uint_least64_t)30 == 0) 
  Serial.println(String("Average loop duration: ") + (double)sum_of_difference / (double)n_samples + String(" microseconds"));
#endif

}

#pragma GCC pop_options