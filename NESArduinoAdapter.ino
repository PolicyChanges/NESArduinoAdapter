#pragma GCC optimize("Ofast")
#pragma GCC push_options

#include <Keyboard.h>

#define PROFILE
#define TEC_DEFAULT
//#define PROFILE_BUTTONS

//Connector (Connect also GND and 5V):  CLOCK, LATCH,     DATA
static constexpr u8 inputPinsPort1[] = { 2, 3, 4 };  //change these as necessary

#define CLOCK1 inputPinsPort1[0]
#define LATCH1 inputPinsPort1[1]
#define DATA1 inputPinsPort1[2]

static void setupJoysticks() {
  pinMode(LATCH1, OUTPUT);
  pinMode(CLOCK1, OUTPUT);
  pinMode(DATA1, INPUT_PULLUP);
}

#define currentTime micros()

#define NES_A       B00000001
#define NES_B       B00000010
#define NES_SELECT  B00000100
#define NES_START   B00001000
#define NES_UP      B00010000
#define NES_DOWN    B00100000
#define NES_LEFT    B01000000
#define NES_RIGHT   B10000000

// Button states (used for profiling)
static bool isButtonPressed[8] = {0,0,0,0,0,0,0,0};

#ifdef PROFILE
#define u64 unsigned long
static char buffer[100];
static u32 printInterval = 1000000;
static u32 PRPrintInterval = 10000;
static u32 controllerConnectTimestamp = currentTime;
static u32 previousPrint = currentTime;
static u32 min_d = 1000000000;
static u32 max_d = 0;
static u32 flush = 0;
static u32 intervalDelta = 0;
static u32 profileStartTimestamp = currentTime;
static u32 profileEndTimestamp = currentTime;

#define start_profile profileStartTimestamp = currentTime;
#define end_profile profileEndTimestamp = currentTime;
#define print_profile //todo
#define print_profile_active     if ((currentTime - previousPrint) > printInterval) [[unlikely]] {                                     \
                                   Serial.println("Profiler Delta: " + String(profileEndTimestamp - profileStartTimestamp) + "μs");  \
                                   previousPrint = currentTime;                                                                        \
                                 }
#else
#define start_profile
#define end_profile
#define print_profile
#define print_profile_active
#endif PROFILE

#ifdef PROFILE_BUTTONS
/*
#include <ArxContainer.h>
arx::stdx::map<String, int> buttonIdx = {
        {"a",       0},
        {"b",       1},
        {"select",  2},
        {"start",   3},
        {"up",      4},
        {"down",    5},
        {"left",    6},
        {"right",   7}
    };
*/
static constexpr String nameIdx[8] = {"A","B","Select","Start","Up","Down","Left","Right"};

static u32 previousButtonPrint = currentTime;
static u32 buttonStartTimestamp[8] = {0,0,0,0,0,0,0,0};
static u32 buttonStopTimestamp[8]  = {0,0,0,0,0,0,0,0};
static u32 buttonEventID[8]        = {0,0,0,0,0,0,0,0};
                 
#define profile_start(x) buttonStartTimestamp[x] = currentTime;
#define profile_stop(x)  buttonStopTimestamp[x]  = currentTime; buttonEventID[x]++;
#define profile_delta(x) ((buttonStopTimestamp[x] - buttonStartTimestamp[x])/(double)1000)  /* microseconds to milliseconds */
#define button_event_as_string(x) String(buttonEventID[x]-1)
#define profile_print(x)      Serial.println(button_event_as_string(x) +  " Press/Release Interval: " +             \
                              String(profile_delta(x)) + "ms" + "\t\t\tEventID: " +                          \
                              button_event_as_string(x) + "-" + String(nameIdx[x]));                        \
                              buttonStopTimestamp[x] = buttonStartTimestamp[x] = 0;                          \
                              if(profile_delta(x) <= 8) {if(profile_delta(x) <= 2) Serial.println("bounce EventID: " + button_event_as_string(x));} \
                              else Serial.println("Possible bounce EventID: " + button_event_as_string(x)); \
                              
#define profile_print_active(x) if(isButtonPressed[x]){                            \
                                  buttonStopTimestamp[x] = currentTime;             \
                                  Serial.println(profile_delta(x) + String("ms"));  \
                                }
#else
#define profile_start(x) 
#define profile_stop(x) 
#define profile_delta(x) 
#define button_event_as_string(x)
#define profile_print(x) 
#endif PROFILE_BUTTONS

#ifndef PROFILE
#define start_profile
#define end_profile
#define profile_delta
#define print_profile
#endif  NOTPROFILE

// Current state of buttons stored as boolean bits
static u8 currentState = 0;
// State of the controller as it was on previous interval.  Usually between 12 and 20 microseconds.
static u8 previousState = 0;
// Timestamp of previous interval
static unsigned long previousTime = currentTime;
// pollInterval is the interval between reading
static constexpr unsigned long pollInterval = 0;//196;  //1000*1000;     //microseconds

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
bool isHandlingTECInput() [[force_inline]];

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
bool isHandlingTECInput() [[force_inline]] { return false; }

static constexpr u8 keyMapKeys[8]{
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

// Debounce Interverals Per Button
static constexpr u32 buttonPressedInterval[8] = { 31960, 31960, 0, 0, 0, 0, 12000, 12000};
static constexpr u32 buttonReleasedInterval[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
// User Input Timestamps
static u32 buttonPressedTimestamp[8];
static u32 buttonReleasedTimestamp[8];

void setup() {
#ifdef PROFILE
  Serial.begin(115200);
#endif PROFILE

  Keyboard.begin();
  setupJoysticks();

  // Initialize input timestamps
  for (int i = 0; i < 8; i++) {
    buttonPressedTimestamp[i] = currentTime;
    buttonReleasedTimestamp[i] = currentTime;
  }
}

unsigned long processInput() [[force_inline]] {

  const unsigned long processInputCurrentTimestamp = currentTime;
  const u8 changedStates = currentState ^ previousState;
  
  for (int i = 0; i < 8; i++) {
    if ((changedStates >> i) & 0b00000001) {
      if (((currentState >> i) & 0b00000001)) {
        if (!isButtonPressed[i] && processInputCurrentTimestamp - buttonPressedTimestamp[i] > buttonPressedInterval[i]) {
          Keyboard.press(keyMapKeys[i]);
          buttonPressedTimestamp[i] = processInputCurrentTimestamp;
          isButtonPressed[i] = true;
          profile_start(i)
        }
      } else if (isButtonPressed[i] && processInputCurrentTimestamp - buttonReleasedTimestamp[i] > buttonReleasedInterval[i]) {
        Keyboard.release(keyMapKeys[i]);
        buttonReleasedTimestamp[i] = processInputCurrentTimestamp;
        isButtonPressed[i] = false;
        profile_stop(i) 
        profile_print(i)
      }
    }
  }
  return processInputCurrentTimestamp;
}

#define latch_low digitalWrite(LATCH1, LOW)
#define latch_high digitalWrite(LATCH1, HIGH)
#define clock_low digitalWrite(CLOCK1, LOW)
#define clock_high digitalWrite(CLOCK1, HIGH)
#define wait delayMicroseconds(10)

void readController(u8 &state) [[force_inline]] {
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

void loop() {

  currentState = 0;

 // if (currentTime - previousTime >= pollInterval) {
//profile_print_active(6)
print_profile_active
start_profile 
    readController(currentState);

    //const u8 changedButtonStates = currentState ^ previousState;

    if (isHandlingTECInput() == false) [[likely]] {
      previousTime = processInput();
    }
    previousState = currentState;
end_profile
//  }

}

#ifdef TEC_DEFAULT
bool isHandlingTECInput() [[force_inline]] {
  bool isHandlingInput = false;
  
  if((previousState & NES_SELECT) && !(currentState & NES_SELECT))   // if select and another button was pressed in the previous input, release all
    Keyboard.releaseAll();
  else if(currentState & NES_SELECT)
  {
    isHandlingInput = true;

    // Emotes
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

    // Zoom in
    if(!(previousState & NES_A) && (currentState & NES_A)) 
      Keyboard.press(KEY_W); 
    else if((previousState & NES_A) && !(currentState & NES_A))
      Keyboard.release(KEY_W);

    // Zoom out
    if(!(previousState & NES_B) && (currentState & NES_B))
      Keyboard.press(KEY_S);
    else if((previousState & NES_B) && !(currentState & NES_B))
      Keyboard.release(KEY_S);
    
    // Escape key/back button
    if(!(previousState & NES_START) && (currentState & NES_START))
      Keyboard.press(KEY_ESC);
    else if((previousState & NES_START) && !(currentState & NES_START))
      Keyboard.release(KEY_ESC);

  }   

  return isHandlingInput;
}
#endif TEC_DEFAULT

#pragma GCC pop_options