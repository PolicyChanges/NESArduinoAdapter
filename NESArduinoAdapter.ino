#pragma GCC optimize("Ofast")
#pragma GCC push_options

#include <Keyboard.h>

#define TEC_DEFAULT
#define PROFILE
//#define PROFILE_BUTTONS


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

// Current state of buttons stored as boolean bits
//static u8 currentState = 0;
// State of the controller as it was on previous interval.
static u8 previousState = 0;
// Timestamp of previous interval
static unsigned long previousTime = currentTime;
// pollInterval is the interval between reading controller. loop() runs at 16MHz
// so set to 500-4000 to minimize bit-bashing. 12000 to eliminate bit-bashing (in microseconds)
static constexpr unsigned long pollInterval = 800; 
static constexpr unsigned long debounceInterval = 54000;

constexpr unsigned long numberOfFramesDebounce = 3; 
constexpr unsigned long frameInteveralMicros = 16000;
constexpr unsigned long debounceIntervalMicros = numberOfFramesDebounce * frameInteveralMicros;

#define NES_A       B00000001
#define NES_B       B00000010
#define NES_SELECT  B00000100
#define NES_START   B00001000
#define NES_UP      B00010000
#define NES_DOWN    B00100000
#define NES_LEFT    B01000000
#define NES_RIGHT   B10000000

static constexpr char* nameIdx[8] = {"A","B","Select","Start","Up","Down","Left","Right"};

#ifdef PROFILE
  static unsigned long total_delta = 0;
  static u32 printInterval = 1000000;
  static u32 previousPrint = currentTime;
  static u32 min_d = 1000000000;
  static u32 max_d = 0;
  static u32 profileStartTimestamp = currentTime;
  static u32 profileEndTimestamp = currentTime;

  #define start_profile()       profileStartTimestamp = currentTime;
  #define end_profile()         profileEndTimestamp = currentTime;
  #define profile_delta_active  (profileEndTimestamp - profileStartTimestamp)
  #define print_delta_sum       Serial.println("Delta Sum: " + String(total_delta));
  #define print_profile_active(x)  if ((currentTime - previousPrint) > printInterval) [[unlikely]] {                                     \
                                Serial.println("Profiler Delta: " + String(profile_delta_active) + "μs Loop delta: " + String(x));                             \
                                previousPrint = currentTime;}//print_delta_sum}

#else
  #define start_profile()
  #define end_profile()
  #define print_profile_active(x)
#endif 

#ifdef PROFILE_BUTTONS

// Button states (used for profiling)
static bool isButtonPressed[8] = {0,0,0,0,0,0,0,0};

static u32 previousButtonPrint = currentTime;
static u32 buttonStartTimestamp[8] = {0,0,0,0,0,0,0,0};
static u32 buttonStopTimestamp[8]  = {0,0,0,0,0,0,0,0};
static u32 buttonEventID[8]        = {0,0,0,0,0,0,0,0};
static u32 nBounces[8]             = {0,0,0,0,0,0,0,0};

#define profile_start(x)  isButtonPressed[x] = true; buttonStartTimestamp[x] = currentTime;
#define profile_stop(x)  buttonStopTimestamp[x]  = currentTime; ++buttonEventID[x]-1; isButtonPressed[x] = false;
#define profile_delta(x) ((buttonStopTimestamp[x] - buttonStartTimestamp[x])/(double)1000)  /* microseconds to milliseconds */
#define mod_16(x) ((unsigned long)round(profile_delta(x))%16)
#define button_event_as_string(x) String(buttonEventID[x]-1)

#define print_bounce(x) if(profile_delta(x) <= (double)36){                                                          \
                        Serial.print("*****Bounce on EventID: " + button_event_as_string(x) +                            \
                        " Interval: "); Serial.print(profile_delta(x), 3);                                          \ 
                        Serial.println(String("ms") +                                                               \
                        " Total: " + String(++nBounces[x]) +                                                        \
                        " Bounces per press: " + String((double)nBounces[x] * 100.0 / (double)(buttonEventID[x]-1)) + "%" + \
                        " Button: " + String(nameIdx[x])+"******");}      


#define reset_button_profile(x) buttonStopTimestamp[x] = buttonStartTimestamp[x] = 0;

#define profile_print(x)  /*if(nameIdx[x][0] == 'L' || nameIdx[x][0] == 'R')*/                                      \ 
                          Serial.println(String(nameIdx[x]) +  " Press/Release Interval: " +                        \
                          String(profile_delta(x)) + "ms" + "\t\tEventID: " +                                       \
                          button_event_as_string(x) + "-" + String(nameIdx[x]) +                                    \
                          "   \tMod16: " + mod_16(x));                                                              \                                                        
                              
#define profile_print_active(x) if(isButtonPressed[x]){                                                             \
                                  buttonStopTimestamp[x] = currentTime;                                             \
                                  Serial.println(profile_delta(x) + "ms");}
#else
// Button states (used for profiling)
static bool isButtonPressed[8] = {0,0,0,0,0,0,0,0};

static u32 buttonStartTimestamp[8] = {0,0,0,0,0,0,0,0};
static u32 buttonStopTimestamp[8]  = {0,0,0,0,0,0,0,0};
static u32 buttonEventID[8]        = {0,0,0,0,0,0,0,0};
#define profile_start(x)  isButtonPressed[x] = true; buttonStartTimestamp[x] = currentTime;
#define profile_stop(x)  buttonStopTimestamp[x]  = currentTime; ++buttonEventID[x]-1; isButtonPressed[x] = false;
#define profile_delta(x) 
#define print_bounce(x)
#define button_event_as_string(x)
#define profile_print(x) 
#define reset_button_profile(x) buttonStopTimestamp[x] = buttonStartTimestamp[x] = 0;
#endif 


#define latch_low digitalWrite(LATCH, LOW)
#define latch_high digitalWrite(LATCH, HIGH)
#define clock_low digitalWrite(CLOCK, LOW)
#define clock_high digitalWrite(CLOCK, HIGH)

#define _delayNanoseconds(__ns)     __builtin_avr_delay_cycles( (double)(F_CPU)*((double)__ns)/1.0e9 )

//#define wait _delayNanoseconds(6000) // 6μs
#define USE_INTERRUPT 
#ifdef USE_INTERRUPT
#define wait delayMicroseconds(12)
#else
#define wait _delayNanoseconds(1)
#endif USE_INTERRUPT
//#define wait delayMicroseconds(12)

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

static bool isHandlingTECInput() [[force_inline]] { return false; }
static constexpr u8 keyMapKeys[8] {
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
static constexpr u32 buttonPressedInterval[8]  = { 130000, 130000, 16000, 16000, 16000, 16000, 16000, 16000 };
static constexpr u32 buttonReleasedInterval[8] = { 16000, 16000, 16000, 16000, 16000, 16000, 16000, 16000 };

static constexpr u32 initialPressedDebounceInterval[8]  = { 130000, 130000, 16000, 16000, 16000, 16000, 16000, 16000 };
static constexpr u32 subsequentPressedDebounceInterval[8]  = { 64000, 64000, 64000, 64000, 64000, 64000, 64000, 64000 };
static u32 numberOfKeyPressedBounces[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// User Input Timestamps
static unsigned long long buttonPressedTimestamp[8];
static unsigned long long buttonReleasedTimestamp[8];
static unsigned long long previousChangedStateTimestamp[8];


void readController() [[force_inline]];

void setup() {
#if defined(PROFILE) || defined(PROFILE_BUTTONS)
  Serial.begin(9600);
#endif

  Keyboard.begin();
  setupJoysticks();

  // Initialize input timestamps
  for (int i = 0; i < 8; i++) {
    buttonPressedTimestamp[i]  = currentTime;
    buttonReleasedTimestamp[i] = currentTime;
    previousChangedStateTimestamp[i] = currentTime;
  }
#ifdef USE_INTERRUPT
  attachInterrupt(digitalPinToInterrupt(LATCH), readControllerRaw, FALLING);
  attachInterrupt(digitalPinToInterrupt(CLOCK), readControllerRaw, FALLING);
#endif
}

#define MAX_SAMPLES 480
volatile int buttonStateVoltage[MAX_SAMPLES];

volatile unsigned long long buttonEventIDRaw = 0;
volatile u8 buttonStateRaw = 0;

void processInput(u8 currentState) {
  const unsigned long processInputCurrentTimestamp = currentTime;

  u8 changedStates = currentState ^ previousState;

  for (int i = 0; i < 8; i++) {
    unsigned long buttonPressedDelta = processInputCurrentTimestamp - buttonPressedTimestamp[i];
    if ((changedStates >> i) & 0b00000001) {
      if (((currentState >> i) & 0b00000001)) {
       if ((numberOfKeyPressedBounces[i] == 0 && buttonPressedDelta < initialPressedDebounceInterval[i]) 
        || (numberOfKeyPressedBounces[i] != 0 && buttonPressedDelta < subsequentPressedDebounceInterval[i])) {
          numberOfKeyPressedBounces[i]++;
          Serial.println("Button bounce " + String(nameIdx[i]) + " " + String(numberOfKeyPressedBounces[i]) + ((numberOfKeyPressedBounces[i]==1) ? " time." : " times.") + " Resetting timestamp. Delta: " + String(buttonPressedDelta));
          buttonPressedTimestamp[i] = processInputCurrentTimestamp;
        } else {
            Keyboard.press(keyMapKeys[i]);
            buttonPressedTimestamp[i] = processInputCurrentTimestamp;
            //previousChangedStateTimestamp[i] = processInputCurrentTimestamp;
            profile_start(i)
            numberOfKeyPressedBounces[i] = 0;
        }
      } 
      else { // if (processInputCurrentTimestamp - buttonReleasedTimestamp[i] > buttonReleasedInterval[i]) {
        //Serial.println("Bounce with interval " + String((double)(micros() - buttonPressedTimestamp[i])));
        profile_stop(i)        
        print_bounce(i) 
        Keyboard.release(keyMapKeys[i]);
        buttonReleasedTimestamp[i] = processInputCurrentTimestamp;
        profile_print(i)
        reset_button_profile(i)
      }
    }
  }
}

void readControllerRaw() {
  buttonStateRaw |= (!digitalRead(DATA) << (buttonEventIDRaw%8));      
}

void loop() {
  const unsigned long currentLoopTimestamp = currentTime;

  if (currentLoopTimestamp - previousTime >= pollInterval) {
    unsigned long t1 = currentTime;
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
    
    previousTime = currentLoopTimestamp;
    
    u8 changedStates = buttonStateRaw ^ previousState;
    
    if (changedStates && isHandlingTECInput(buttonStateRaw) == false) 
      processInput(buttonStateRaw);   
    
    previousState = buttonStateRaw;
    //Serial.println("function duration: "+String(currentTime-t1));
  }

}

#ifdef TEC_DEFAULT
bool isHandlingTECInput(u8 state) {
  bool isHandlingInput = false;
  
  if((previousState & NES_SELECT) && !(state & NES_SELECT))   // if select and another button was pressed in the previous input, release all
    Keyboard.releaseAll();
  else if(state & NES_SELECT)
  {
    isHandlingInput = true;

    // Emotes
    if(state & NES_UP && !(previousState & NES_UP)) {
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
    
    if (state & NES_RIGHT && !(previousState & NES_RIGHT)){
      Keyboard.press(KEY_1);
      Keyboard.press(KEY_RIGHT_ARROW);
      Keyboard.release(KEY_1);
      Keyboard.release(KEY_RIGHT_ARROW);
    }

    // Zoom in
    if(!(previousState & NES_A) && (state & NES_A)) 
      Keyboard.press(KEY_W); 
    else if((previousState & NES_A) && !(state & NES_A))
      Keyboard.release(KEY_W);

    // Zoom out
    if(!(previousState & NES_B) && (state & NES_B))
      Keyboard.press(KEY_S);
    else if((previousState & NES_B) && !(state & NES_B))
      Keyboard.release(KEY_S);
    
    // Escape key/back button
    if(!(previousState & NES_START) && (state & NES_START))
      Keyboard.press(KEY_ESC);
    else if((previousState & NES_START) && !(state & NES_START))
      Keyboard.release(KEY_ESC);

  }  
  return isHandlingInput;
}
#else
constexpr bool isHandlingTECInput(u8 state) { 
  return false;
}
#endif TEC_DEFAULT

#pragma GCC pop_options