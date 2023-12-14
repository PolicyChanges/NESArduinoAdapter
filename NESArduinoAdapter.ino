

/* 
Startup Modes:
pressing these keys while plugging in the controller usb will set various modes and settings.
a = 50hz debouncing
b = goofy foot controller
start = emulator friendly(non-tec) mode
select + up,down,left,right = 1,2,4,8 ms controller polling respectively (2000 microsecond polling is default)

Emulator Keyboard Bindings:
A                 = X
B                 = Z
start             = I
select            = J
up                = up
down              = down
left              = left
right             = right

TEC Mode Keyboard Bindings:
A                 = X
B                 = Z
start             = enter/return
select+B          = escape
select+A          = ctrl/control
select+A+B        = right shift/shift
select+start+A    = W (zoom in)
select+start+B    = Z (zoom out)
select+start+A+B  = F1
up                = up
down              = down
left              = left
right             = right

XInput/XBox Controller Default Bindings:
default xbox controls

Xinput/XBox Controller TEC Bindings:
A                 = Button B
B                 = Button A
Start             = Start
Select            = Back
Select+A          = Right Stick Up (zoom in)
Select+B          = Right Stick Down (zoom out)
Select+A+B        = unused
Select+Start+A    = Button X
Select+Start+B    = Button Y
Select+Start+A+B  = R3
up                = up
down              = down
left              = left
right             = right
*/

/*
For xinput support add to board manager
https://raw.githubusercontent.com/dmadison/ArduinoXInput_Boards/master/package_dmadison_xinput_index.json
for teensy, sparkfun, etc you may have to download extra board managers
Select AVR board with xinput and appropriate processor
Make sure to remove the "#define USE_KEYBOARD" line

To program the microcontroller with the xinput bootloader, you must ground reset pin twice then select port when available.  Now that the port is selected, ground the reset pin 
twice again, before pressing the upload button.  Once the Arudino IDE displays uploading in the bottom right, ground the pin twice once more, and the sketch will
upload to the microcontroller.  Putting a button in between the pins makes this much easier.
*/

// Comment out below and switch board to xinput(from url above) to act as an xbox controller/xinput device

#define USE_KEYBOARD

#ifndef USE_KEYBOARD
#define USE_XINPUT
#endif

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

#define PAL_DEBOUNCING  NES_A
#define GOOFY           NES_B
#define EMU_MODE        NES_START
#define POLL_RATE(x) (x >> 4)

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

#define latchLow digitalWrite(LATCH, LOW)
#define latchHigh digitalWrite(LATCH, HIGH)
#define clockLow digitalWrite(CLOCK, LOW)
#define clockHigh digitalWrite(CLOCK, HIGH)
#define wait delayMicroseconds(12)

static u8 previousState = 0;
static u8 updateState = 0;

void readController(u8 &state) __attribute((always_inline));

#ifdef USE_KEYBOARD
  #include <Keyboard.h>
  #include <KeyboardLayout.h>
#elif defined(USE_XINPUT)
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
#define KEY_1 0x31
#define VK_SHIFT 0x10

#ifdef USE_KEYBOARD

enum XInputControl : uint8_t
{
  BUTTON_B      = KEY_X,
  BUTTON_A      = KEY_Z,
  BUTTON_BACK   = 0x0,
  BUTTON_START  = KEY_RETURN,
  DPAD_UP       = KEY_UP_ARROW,
  DPAD_DOWN     = KEY_DOWN_ARROW,
  DPAD_LEFT     = KEY_LEFT_ARROW,
  DPAD_RIGHT    = KEY_RIGHT_ARROW,
  BUTTON_X      = KEY_ESC,
  BUTTON_Y      = KEY_RIGHT_CTRL, 
  //BUTTON_LOGO   = KEY_F1,
  BUTTON_LB     = KEY_I,
  BUTTON_RB     = KEY_J,
  BUTTON_R3     = KEY_RIGHT_SHIFT,
 /* BUTTON_L3 = KEY_W,
  BUTTON_R3 = KEY_S,
  TRIGGER_LEFT = 15,
  TRIGGER_RIGHT = 16,*/
  JOY_LEFT,
  JOY_RIGHT
};

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
      // up is button b; down is button a
      if(updateState & NES_B && updateState & NES_A)
        press(BUTTON_R3);
      else
        release(BUTTON_R3);

      if((previousState & NES_B) && !up)
        press(BUTTON_X);
      else
        release(BUTTON_X);

      if((previousState & NES_A) && !down)
        press(BUTTON_Y);
      else
        release(BUTTON_Y);
    } 
  }
  uint8_t getSelectStartA() {
    return KEY_W;
  }
  uint8_t getSelectStartB() {
    return KEY_S;
  }
  uint8_t getSelectStartAB() {
    return KEY_F1;
  }
    void handleSelectStart() {
    if(updateState & NES_B && updateState & NES_A)
      this->press(this->getSelectStartAB());
    else
      this->release(this->getSelectStartAB());

    if (~updateState & NES_B){
      if(updateState & NES_A)
        this->press(this->getSelectStartA()); 
      else if(!(updateState & NES_B))
        this->release(this->getSelectStartA());
    }
    if(~updateState & NES_A) {
      if(updateState & NES_B) 
        this->press(this->getSelectStartB()); 
      else if(!(updateState & NES_A))
        this->release(this->getSelectStartB());
    }
  }
  String getDeviceType() {
    return "Keyboard";
  }
};

Controller_ *controller = dynamic_cast<Keyboard_*>(&Keyboard); 

#elif defined(USE_XINPUT)
class Controller_ : public XInputController
{
public:
  uint8_t getSelectStartA() {
    return XInputControl::BUTTON_Y;
  }
  uint8_t getSelectStartB() {
    return XInputControl::BUTTON_X;
  }
  uint8_t getSelectStartAB() {
    return XInputControl::BUTTON_R3;
  }
  void handleSelectStart() {
    if(updateState & NES_A) 
      this->press(this->getSelectStartA()); 
    else
      this->release(this->getSelectStartA());
  
    if(updateState & NES_B)
      this->press(this->getSelectStartB()); 
    else
      this->release(this->getSelectStartB());
  }
  String GetDeviceType() {
    return "XInput";
  }
};

uint8_t KEY_ESC = 0;

Controller_ *controller = dynamic_cast<XInputController*>(&XInput);
#endif

static constexpr u8 xinputMapKeys[8] 
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

static constexpr u8 goofyMapButtons[8] 
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

static constexpr u8 emuMapButtons[8] 
{
  BUTTON_B,
  BUTTON_A,
  BUTTON_LB,
  BUTTON_RB,
  DPAD_UP,
  DPAD_DOWN,
  DPAD_LEFT,
  DPAD_RIGHT
};

static const bool isEmuFriendlyBinds = []() -> const bool
{
  u8 startupState = 0;

  setupJoysticks();
  readController(startupState);

  if(startupState & EMU_MODE)
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

  if(startupState & GOOFY)
    return goofyMapButtons;

  return xinputMapKeys;  
}();

// in microseconds
static const unsigned long debounceInterval = []() -> const unsigned long
{
  u8 startupState = 0;
  double videoFreq = 0;

  // note: 4*4 + 4! + 8 - 1 = 47 potential startup mode combinations
  setupJoysticks();
  readController(startupState);

  if(startupState & PAL_DEBOUNCING)
    videoFreq = 50;
  else
    videoFreq = 60;
  
  double padding  = 0; 
  double numOfFrames = 1.5;

  return ((unsigned long)((1 / videoFreq) * 1000) * numOfFrames * 1000) - padding;
}();

static const unsigned long pollInterval = []() -> const unsigned long
{
  u8 startupState = 0;

  setupJoysticks();
  readController(startupState);

  if(startupState | NES_SELECT)
    return (POLL_RATE(startupState) * 1000);

  return 4000;
}();

static struct buttonClamp
{
// In microseconds
//#define NO_DEBOUNCE
#ifdef NO_DEBOUNCE
  const unsigned long clampDownInterval[8] { 0, 0, 0, 0, 0, 0, 0, 0 };
  const unsigned long clampUpInterval[8]   { 0, 0, 0, 0, 0, 0, 0, 0 };
#else
  const unsigned long debounceIntervalDown = 2000; // reduce over shifting
  const unsigned long clampDownInterval[8] { debounceIntervalDown, debounceIntervalDown, debounceIntervalDown, debounceIntervalDown, debounceIntervalDown, debounceIntervalDown, debounceIntervalDown, debounceIntervalDown };
  const unsigned long clampUpInterval[8]   { debounceInterval, debounceInterval, debounceInterval, debounceInterval, debounceInterval, debounceInterval, debounceInterval, debounceInterval };
#endif

  unsigned long onDownStateTimeStamp[8];
  unsigned long onUpStateTimeStamp[8];
  bool isPressed[8] { false, false, false, false, false, false, false, false };
} clamp;

// 204us
void readController(u8 &state) 
{
  state = 0;
  latchLow;
  clockLow;
  latchHigh;
  wait;
  latchLow;

  for (u8 i = 0; i < 8; i++) {
    state |= (!digitalRead(DATA) << i);
    clockHigh;
    wait;
    clockLow;
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

  controller->setAutoSend(true); 
  controller->begin();

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
        controller->press(buttonsMap[i]); 
      else  
        controller->release(buttonsMap[i]); 
  }
}

void handleSelect(const u8 updateStates) __attribute((always_inline));
void handleSelect(const u8 updateStates)
{
  // TEC Menu Navigation
  if(updateStates & NES_START)
    controller->handleSelectStart();
       
  if(!(updateStates & NES_START)) {
    controller->setJoystick(XInputControl::JOY_LEFT, updateStates & NES_B, updateStates & NES_A, false, false, false);  // menu navigation 
    controller->setJoystick(XInputControl::JOY_RIGHT, updateStates & NES_UP, updateStates & NES_DOWN,                   // emotes             
                            updateStates & NES_LEFT, updateStates & NES_RIGHT, false); 
  }
}

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
          controller->setJoystick(XInputControl::JOY_RIGHT, false, false, false, false, false);  // XInput library handles state breaking well. leave it.
          controller->setJoystick(XInputControl::JOY_LEFT, false, false, false, false, false);
          controller->releaseAll();

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