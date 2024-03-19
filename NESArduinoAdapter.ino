/* 
Startup Modes:
pressing these keys while plugging in the controller usb will set various modes and settings.
a = 50hz debouncing
b = goofy foot controller
start = emulator friendly(non-tec) mode
select + up,down,left,right = 1,2,4,8 ms controller polling respectively (4000 microsecond polling is default)

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
A                 = Button BX
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
//#define USE_KEYBOARD

#ifndef USE_KEYBOARD
#define USE_XINPUT
#endif

#pragma GCC optimize("O3")
#pragma GCC push_options

// Reference
#define NES_A B00000001
#define NES_B B00000010
#define NES_SELECT B00000100
#define NES_START B00001000
#define NES_UP B00010000
#define NES_DOWN B00100000
#define NES_LEFT B01000000
#define NES_RIGHT B10000000

#define PAL_DEBOUNCING NES_A
#define GOOFY NES_B
#define EMU_MODE NES_START
// Shift right for d-pad bits
#define POLL_RATE(x) (x >> 4)

//Connector (Connect also GND and 5V):  CLOCK, LATCH,     DATA
constexpr const u8 inputPinsPort1[] = { 2, 3, 4 };  //change these as necessary

#define CLOCK inputPinsPort1[0]
#define LATCH inputPinsPort1[1]
#define DATA inputPinsPort1[2]

void setupJoysticks() {
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

// Keyboard inherets XInput properties
enum XInputControl : uint8_t {
  BUTTON_B = KEY_X,
  BUTTON_A = KEY_Z,
  BUTTON_BACK = 0x0,
  BUTTON_START = KEY_RETURN,
  DPAD_UP = KEY_UP_ARROW,
  DPAD_DOWN = KEY_DOWN_ARROW,
  DPAD_LEFT = KEY_LEFT_ARROW,
  DPAD_RIGHT = KEY_RIGHT_ARROW,
  BUTTON_X = KEY_ESC,
  BUTTON_Y = KEY_RIGHT_CTRL,
  //BUTTON_LOGO   = KEY_F1,
  BUTTON_LB = KEY_I,
  BUTTON_RB = KEY_J,
  BUTTON_R3 = KEY_RIGHT_SHIFT,
  /* BUTTON_L3 = KEY_W,
  BUTTON_R3 = KEY_S,
  TRIGGER_LEFT = 15,
  TRIGGER_RIGHT = 16,*/
  JOY_LEFT,
  JOY_RIGHT
};

//*********************************//
//**** Controller Interface *******//
//*********************************//
class Controller_ : public Keyboard_ {
public:
  void setAutoSend(bool send) {
    return 0;
  }
  void setJoystick(XInputControl joy, boolean up, boolean down, boolean left, boolean right, boolean useSOCD = true) {

    if (joy == XInputControl::JOY_RIGHT) {
      uint8_t dir[4]{ DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT };
      bool states[4]{ up, down, left, right };
      for (int i = 0; i < 4; i++) {
        if (states[i]) {
          press(KEY_1);
          this->press(dir[i]);
        } else {
          release(KEY_1);
          release(dir[i]);
        }
      }
    } else if (joy == XInputControl::JOY_LEFT) {
      // up is button b; down is button a
      if (updateState & NES_B && updateState & NES_A)
        press(BUTTON_R3);
      else
        release(BUTTON_R3);

      if ((previousState & NES_B) && !up)
        press(BUTTON_X);
      else
        release(BUTTON_X);

      if ((previousState & NES_A) && !down)
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
    if (updateState & NES_B && updateState & NES_A)
      this->press(this->getSelectStartAB());
    else
      this->release(this->getSelectStartAB());

    if (~updateState & NES_B) {
      if (updateState & NES_A)
        this->press(this->getSelectStartA());
      else if (!(updateState & NES_B))
        this->release(this->getSelectStartA());
    }
    if (~updateState & NES_A) {
      if (updateState & NES_B)
        this->press(this->getSelectStartB());
      else if (!(updateState & NES_A))
        this->release(this->getSelectStartB());
    }
  }
  String getDeviceType() {
    return "Keyboard";
  }
};

Controller_ *controller = dynamic_cast<Keyboard_ *>(&Keyboard);

#elif defined(USE_XINPUT)
class Controller_ : public XInputController {
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
    if (updateState & NES_A)
      this->press(this->getSelectStartA());
    else
      this->release(this->getSelectStartA());

    if (updateState & NES_B)
      this->press(this->getSelectStartB());
    else
      this->release(this->getSelectStartB());
  }
  String GetDeviceType() {
    return "XInput";
  }
};

uint8_t KEY_ESC = 0;

Controller_ *controller = dynamic_cast<XInputController *>(&XInput);
#endif
//*********************************//


//*********************************//
//**** Button mappings ************//
//*********************************//
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

static constexpr u8 goofyMapButtons[8]{
  BUTTON_A,
  BUTTON_B,
  BUTTON_START,
  BUTTON_BACK,
  DPAD_DOWN,
  DPAD_UP,
  DPAD_RIGHT,
  DPAD_LEFT
};

static constexpr u8 emuMapButtons[8]{
  BUTTON_B,
  BUTTON_A,
  BUTTON_LB,
  BUTTON_RB,
  DPAD_UP,
  DPAD_DOWN,
  DPAD_LEFT,
  DPAD_RIGHT
};
//*********************************//


//*********************************//
//**** Variables set on plugin ****//
//*********************************//
static const bool isEmuFriendlyBinds = []() -> const bool {
  u8 startupState = 0;
  setupJoysticks();
  readController(startupState);
  if (startupState & EMU_MODE)
    return true;

  return false;
}();

static const u8 *buttonsMap = []() -> const u8 * {
  u8 startupState = 0;
  setupJoysticks();
  readController(startupState);
  if (isEmuFriendlyBinds)
    return emuMapButtons;

  if (startupState & GOOFY)
    return goofyMapButtons;

  return xinputMapKeys;
}();

constexpr double numOfFrames = 1.4;

// in microseconds
static const u32 debounceIntervalPress = []() -> const u32 {
  u8 startupState = 0;
  setupJoysticks();
  readController(startupState);
  double videoFreq = 0;

  if (startupState & PAL_DEBOUNCING)
    videoFreq = 50;
  else
    videoFreq = 60;

  return ((u32)((1 / videoFreq) * 1000) * numOfFrames * 1000);
}();

static const u32 pollInterval = []() -> const u32 {
  u8 startupState = 0;
  setupJoysticks();
  readController(startupState);
  if (startupState | NES_SELECT)
    return (POLL_RATE(startupState) * 1000);

  // poll interval notes:
  // micros() skews min +4. mode is +12us ~70%
  // double flips on linux <= 1000
  return 2000-12;  
}();

void (*loopMain)();
void loopTECFunc();
void loopBasicFunc();

//whatever devilry. no performance impact.
/*static const void (*loopMain)() = +[]() { 
 if (isEmuFriendlyBinds)
    return &loopBasicFunc;
    
  return &loopTECFunc;
 }();*/
 
//*********************************//



//*********************************//
//**** Debouncing *****************//
//*********************************//
static struct buttonClamp {
// In microseconds
//#define NO_DEBOUNCE
#ifdef NO_DEBOUNCE
  const u32 clampDownInterval[8]{ 0, 0, 0, 0, 0, 0, 0, 0 };
  const u32 clampUpInterval[8]  { 0, 0, 0, 0, 0, 0, 0, 0 };
#else
  const u32 debounceIntervalRelease = 2 - numOfFrames;  

  const u32 clampPressInterval[8]    { debounceIntervalRelease, debounceIntervalRelease, debounceIntervalRelease, debounceIntervalRelease, debounceIntervalRelease, debounceIntervalRelease, debounceIntervalRelease, debounceIntervalRelease };
  const u32 clampReleaseInterval[8]  { debounceIntervalPress, debounceIntervalPress, debounceIntervalPress, debounceIntervalPress, debounceIntervalPress, debounceIntervalPress, debounceIntervalPress, debounceIntervalPress };
#endif

  u32 onPressStateTimeStamp[8];
  u32 onReleaseStateTimeStamp[8];
  bool isPressed[8]{ false, false, false, false, false, false, false, false };
} clamp;

// 204us
void readController(u8 &state) {
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
//*********************************//

void setup() {
//#define DEBUG_KEYBOARD
#ifdef DEBUG_KEYBOARD
  Serial.begin(9600);
#endif

  controller->setAutoSend(true);
  controller->begin();

  for (u8 i = 0; i < 8; i++)
    clamp.onReleaseStateTimeStamp[i] = clamp.onPressStateTimeStamp[i] = micros();
 if (isEmuFriendlyBinds)
    loopMain = &loopBasicFunc;
    
  loopMain = &loopTECFunc;
}

void processInput(const u8 currentStates, u8 &processUpdateState) __attribute((always_inline));
void processInput(const u8 processCurrentState, u8 &processUpdateState) {  // TODO: catch if select is pressed and diabled releases
  const u8 changedState = processCurrentState ^ previousState;
  const u32 processCurrentTime = micros();
  for (u8 i = 0; i < 8; i++) {
    if ((changedState >> i) & 0b00000001) {
      if ((processCurrentState >> i) & 0b00000001) {
        if (!clamp.isPressed[i] && processCurrentTime - clamp.onReleaseStateTimeStamp[i] >= clamp.clampReleaseInterval[i]) {
          processUpdateState |= (0b00000001 << i);
          clamp.onPressStateTimeStamp[i] = processCurrentTime;
          clamp.isPressed[i] = true;
        }
      } else {
        if (clamp.isPressed[i] && processCurrentTime - clamp.onPressStateTimeStamp[i] >= clamp.clampPressInterval[i]) {
          processUpdateState &= 0b11111111 & ~(0b00000001 << i);
          clamp.onReleaseStateTimeStamp[i] = processCurrentTime;
          clamp.isPressed[i] = false;
        }
      }
    }
  }
}

void updateInput(const u8 updateStates) __attribute((always_inline));
void updateInput(const u8 updateState) {
  const u8 changedStates = updateState ^ previousState;
  for (u8 i = 0; i < 8; i++) {
    if ((changedStates >> i) & 0b00000001)
      if (((updateState >> i) & 0b00000001))
        controller->press(buttonsMap[i]);
      else
        controller->release(buttonsMap[i]);
  }
}

void handleSelect(const u8 updateStates) __attribute((always_inline));
void handleSelect(const u8 updateStates) {
  // TEC Menu Navigation
  if (updateStates & NES_START)
    controller->handleSelectStart();

  if (!(updateStates & NES_START)) {
    controller->setJoystick(XInputControl::JOY_LEFT, updateStates & NES_B, updateStates & NES_A, false, false, false);  // menu navigation
    controller->setJoystick(XInputControl::JOY_RIGHT, updateStates & NES_UP, updateStates & NES_DOWN,                   // emotes
                            updateStates & NES_LEFT, updateStates & NES_RIGHT, false);
  }
}

static u32 previousTime = micros();
static u32 currentTime = micros();

void loopBasicFunc() {
  u8 currentState = 0;

  readController(currentState);

  if (currentState != previousState) {

    processInput(currentState, updateState);

    if (updateState != previousState) {
      updateInput(updateState);
      previousState = updateState;
    }
  }
}

void loopTECFunc() {
  u8 currentState = 0;

  readController(currentState);

  if (currentState != previousState) {

    processInput(currentState, updateState);

    if (updateState != previousState) {
      // Release select
      if ((previousState & NES_SELECT) && (~updateState & NES_SELECT)) [[unlikely]] {
        controller->setJoystick(XInputControl::JOY_RIGHT, false, false, false, false, false);  // XInput library handles state breaking well. leave it.
        controller->setJoystick(XInputControl::JOY_LEFT, false, false, false, false, false);
        controller->releaseAll();

        previousState = updateState = 0;
        memset((void *)(clamp.isPressed), 0, sizeof(bool) * 8);
      }

      if ((updateState & NES_SELECT) == false) [[likely]] {
        updateInput(updateState);
      } else
        handleSelect(updateState);

      previousState = updateState;
    }
  }
}

//#define Profile
#ifdef Profile
u32 printInterval = 1000;
u32 previousPrint = millis();
u32 min_d = 1000000;
u32 max_d = 0;
#endif

void loop() {

  u32 delta = currentTime - previousTime;
  
  if (delta > pollInterval) {

#ifdef Profile
    min_d = min(min_d, delta);
    max_d = max(max_d, delta);
    u32 currentPrint = millis();
    if(currentPrint - previousPrint > 1000){
      Serial.println(String("wait duration to function: ") + (delta)+" Min/Max: " + min_d + " " + max_d);
      previousPrint = millis();
      min_d = 1000000;
      max_d = 0;
    }
#endif Profile

    loopMain();
    previousTime = currentTime;

  }
  currentTime = micros();

}

#pragma GCC pop_options