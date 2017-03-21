// Copyright (C) 2017 Vladimir Tsymbal
// e-mail: vladimir.tsymbal@web.de
// v.0.4
// This software is furnished "as is", without technical support, and with no
// warranty, express or implied, as to its usefulness for any purpose.
//
// The Interrupt-based Rotary Encoder Menu Sketch
// by Simon Merrett, based on insight from Oleg Mazurov, Nick Gammon, rt and Steve Spence, and code from Nick Gammon

#include <TimerOneCTC.h>
#include <rgb_lcd.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <EEPROM.h>

// User defined parameters
//===================================================================================================//
#define LIN_SPEED 1 // vs. speed in steps if not defined (commented out)


#define STEPS_PER_ROTATION 200 // Number of motor steps per one rotation
#define PULLEY_DIA 30 // Diameter of pulley, mm

#ifdef LIN_SPEED
#define MAX_LIN_SPEED 10 // Maximum linear speed, m/min
#define MIN_LIN_SPEED 0 // Minimum linear speed, m/min
#define INC_LIN_SPEED 0.1 // Increment linear speed, m/min
#else
#define MAX_MOTOR_SPEED 1000 // Maximum speed, steps/s
#define MIN_MOTOR_SPEED 100 // Minimum speed, steps/s
#define INC_MOTOR_SPEED 1 // Minimum speed, steps/s
#endif // LIN_SPEED

#define MAX_ACCEL_TIME 5 // Maximum time for acceleration, s
#define MIN_ACCEL_TIME 1 // Minimum time for acceleration, s
#define INC_ACCEL_TIME 1 // Increment time for acceleration, s

#define MAX_FW_TIME 8 // Maximum forward feed time, s
#define MIN_FW_TIME 0 // Minimum forward feed time, s
#define INC_FW_TIME 0.1 // Increment forward feed time, s

#define MAX_BW_TIME 8 // Maximum backward feed time, s
#define MIN_BW_TIME 0 // Minimum backward feed time, s
#define INC_BW_TIME 0.1 // Increment backward feed time, s

#define MAX_PAUSE 1 // Maximum pause time, s
#define MIN_PAUSE 0 // Minimum pause time, s
#define INC_PAUSE 0.1 // Increment pause time, s

#define MIN_RUN_MODE 2 // 2 tackt start mode 
#define MAX_RUN_MODE 4 // 4 tackt start mode
#define INC_RUN_MODE 2 // increment tackt start mode

//==============================================================================
// Assigned board pins to buttons

const int EncPinA = 2; // Encoder channel A interrupt pin
const int EncPinB = 3; // Encoder channel B interrupt pin
const int buttonPinE = 4; // Encoder push button pin

const int buttonPinS = 5; // Start button pin
const int buttonPinP = 6; // Feed button pin

const int buttonPinA = 7; // Button A pin
const int buttonPinB = 8; // Button B pin
const int buttonPinC = 9; // Button C pin
const int buttonPinD = 10; // Button D pin

const int stepperDirPin = 11; // Stepper driver DIR pin
const int stepperStepPin = 12; // Stepper driver STEP (PUL) pin
const int stepperEnaPin = 13; // Stepper driver ENA pin

//==============================================================================

// Define a stepper and the pins it will use
//AccelStepper stepper(AccelStepper::HALF4WIRE, 2, 4, 3, 5); // Defaults to 4 pins on 2, 3, 4, 5
//AccelStepper stepper(AccelStepper::HALF4WIRE, 14, 16, 15, 17);

AccelStepper stepper(AccelStepper::DRIVER, stepperStepPin, stepperDirPin);

//==============================================================================
// Define an LCD screen

rgb_lcd lcd;
// Define LCD screen backlight color
const int colorR = 100;
const int colorG = 100;
const int colorB = 0;

//==============================================================================

// State of system
#define ST_SELECT 1 // Modes/parameters selection
#define ST_ONRUN 0 // Motor running

// Types of modes (can be added more combinations)
/*
  enum run_mode
  {
  rm_FWD, // Forward
  rm_BKW, // Backward
  rm_FWP, // Forward-Pause
  rm_FPB, // Forward-Pause-Backward
  RUN_MODES_NUM
  };
*/
// Types of actione within any mode
enum act_type
{
  ac_FW, // moving forward
  ac_BW, // moving backward
  ac_PS, // staying in pause
};
enum button
{
  bt_Non,
  bt_A,
  bt_B,
  bt_C,
  bt_D,
  bt_S,
  bt_P
};
enum _start_mode
{
  st_2T = 2,
  st_4T = 4,
};

enum param_id
{
  par_speed,
  par_fwtime,
  par_pause,
  par_bwtime,
  par_accel,
  par_stmode
};

typedef struct params
{
  float value;
  float inc;
  int max;
  int min;
  byte id;
  char name[8];
} Param;
#define NUM_PARAM 6
#define MAGIC_NUMBER 74

int _selected_param = 0; // int type in order to be able to decrement a param from 0
Param param[NUM_PARAM];
const Param* GetParam(byte id) {
  for (byte i = 0; i < NUM_PARAM; i++)  {
    if ( param[i].id == id )
      return &param[i];
  }
}
void StoreParams(){
  int eeAddress = 0;
  EEPROM.put(eeAddress, MAGIC_NUMBER);
  eeAddress++;
  
  for (byte i = 0; i < NUM_PARAM; i++)  {
    EEPROM.put(eeAddress, param[i]);
    eeAddress += sizeof(Param);
  }
}
void LoadParams(){
  int eeAddress = 0;
  byte number;
  EEPROM.get( eeAddress, number);
  eeAddress++;
  
  for (byte i = 0; i < NUM_PARAM; i++){
    EEPROM.get( eeAddress, param[i]);
    eeAddress += sizeof(Param);
  }
} 
void PrintParams()
{
  for (byte i = 0; i < NUM_PARAM; i++){
  Serial.println("-----");
  Serial.print("name: ");
  Serial.println(param[i].name);
  Serial.print("id: ");
  Serial.println(param[i].id);
  Serial.print("value: ");
  Serial.println(param[i].value);
  Serial.print("inc: ");
  Serial.println(param[i].inc);
  Serial.print("max: ");
  Serial.println(param[i].max);
  Serial.print("min: ");
  Serial.println(param[i].min);
  }
}

// Action types with their timing parameters
class Action
{
  public:
    Action(byte type, float dir) : act_type(type), dir(dir) {}
    byte act_type;
    long time_period; // Action time, ms
    float dir; // Direction of motor rotation: CW=1, CCW=-1, Non=0
    void (*func)(); // A function to run by action object
    void SetFunction( void(*func)() ) {
      this->func = func;
    };
    void CallFunction() {
      return func();
    };
};

// Action functions
void StepperRunSpeed() {
  stepper.runSpeed();
}
void StepperNoRun() {}

// According to the 3 action types:
Action fw_feed(ac_FW, 1);
Action bw_feed(ac_BW, -1);
Action ps_feed(ac_PS, 0);

// The Mode class combines one or several actions,
// being called one by one in a loop
// Actions are global objects, the calass only manages acccess
class Mode
{
  public:
    void AddAction(Action* p);
    Action* GetAction();
    void NextAction(); // Call to change for a next Action
    int GetActionsNumber();
    void Reset(); // Set indexes to 0
  protected:
    Action* list[8]; // Action pointers container
    int curr = 0; // Curent index of Action
    int N = 0;  // Number of Actions in a Mode
};
void Mode::AddAction(Action* p)
{
  list[curr] = p;
  curr++;
  N++;
}
Action* Mode::GetAction() {
  return list[curr];
}
int Mode::GetActionsNumber() {
  return N;
}
void Mode::NextAction() {
  curr++;
  if (curr == N)
    curr = 0;
}
void Mode::Reset() {
  curr = 0;
}

// Global mode
Mode g_mode;

// Global updatable motor settings
float _MotorSpeed; // steping motor speed, steps/sec
float _MotorAccelTime; //motor acceleration/decceleration, sec


// Global flags
bool _state = ST_SELECT;
bool _update_display = true;
bool reset_timer = true;
bool _is_newaction = true;
byte _start_mode = 0;

//==============================================================================
// Rotary encoder and Menue declarations
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte readingEnc = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
const byte MAX_ENC_POS = 255; // as the encoderPos type is byte

// Button reading, including debounce without delay function declarations
byte _buttonOldState_E = HIGH;  // assume switch open because of pull-up resistor
byte _buttonOldState_S = HIGH;  // assume switch open because of pull-up resistor
unsigned long _buttonPressTime_E;  // when the switch last changed state
unsigned long _buttonPressTime_S;  // when the switch last changed state
boolean _buttonPressed_E = 0; // global condition of switch
const unsigned long debounceTime = 10;  // milliseconds

#define MENU_TOP 0
#define MENU_BOT 1
byte _MenuLevel = MENU_TOP; // With two levels of menu it can be 0 or 1

//==============================================================================
// Interupt pin definitiones for direct port access
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_NANO)
#define PORT_ENC PIND
#define PORT_ENC_MSK 0xC
#define PIN23_MSK B00001100
#define PIN_2_MSK B00000100
#define PIN_3_MSK B00001000
#elif defined(ARDUINO_AVR_MEGA2560)
#define PORT_ENC PINE
#define PORT_ENC_MSK 0x30
#define PIN23_MSK B00110000
#define PIN_2_MSK B00010000
#define PIN_3_MSK B00100000
#elif defined(ARDUINO_SAM_DUE)
//Due specific code
#else
#error Unsupported hardware
#endif

//==============================================================================
//Rotary encoder interrupt service routine for one encoder pin
void EncPinA_ISR() {
  cli(); //stop interrupts happening before we read pin values
  readingEnc = PORT_ENC & PORT_ENC_MSK; // read all eight pin values then strip away all but pinA and pinB's values
  if (readingEnc == PIN23_MSK && aFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos --; //decrement the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (readingEnc == PIN_2_MSK) bFlag = 1; //signal that we're expecting pinB to signal the transition to detent from free rotation
  sei(); //restart interrupts
}

//Rotary encoder interrupt service routine for the other encoder pin
void EncPinB_ISR() {
  cli(); //stop interrupts happening before we read pin values
  readingEnc = PORT_ENC & PORT_ENC_MSK; //read all eight pin values then strip away all but pinA and pinB's values
  if (readingEnc == PIN23_MSK && bFlag) { //check that we have both pins at detent (HIGH) and that we are expecting detent on this pin's rising edge
    encoderPos ++; //increment the encoder's position count
    bFlag = 0; //reset flags for the next turn
    aFlag = 0; //reset flags for the next turn
  }
  else if (readingEnc == PIN_3_MSK) aFlag = 1; //signal that we're expecting pinA to signal the transition to detent from free rotation
  sei(); //restart interrupts
}


void setup()
{
  Serial.begin(9600);

  // Rotary encoder section of setup
  pinMode(EncPinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(EncPinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  attachInterrupt(digitalPinToInterrupt(EncPinA), EncPinA_ISR, RISING); // set an interrupt on EncPinA, looking for a rising edge signal and executing the "EncPinA_ISR" Interrupt Service Routine (below)
  attachInterrupt(digitalPinToInterrupt(EncPinB), EncPinB_ISR, RISING); // set an interrupt on EncPinB, looking for a rising edge signal and executing the "EncPinB_ISR" Interrupt Service Routine (below)
  // button section of setup
  pinMode (buttonPinE, INPUT_PULLUP); // setup the button pin

  //=================================
  pinMode(buttonPinA, INPUT_PULLUP);
  pinMode(buttonPinB, INPUT_PULLUP);
  pinMode(buttonPinC, INPUT_PULLUP);
  pinMode(buttonPinD, INPUT_PULLUP);
  pinMode(buttonPinS, INPUT_PULLUP);
  pinMode(buttonPinP, INPUT_PULLUP);
  //=================================
  pinMode(stepperDirPin, OUTPUT);
  pinMode(stepperStepPin, OUTPUT);
  pinMode(stepperEnaPin, OUTPUT);

  //=================================
  // LCD setup
  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, colorB);


  // Initial action parameters
  fw_feed.time_period = 1000; // ms
  bw_feed.time_period = 1000; // ms
  ps_feed.time_period = 0; // ms
  fw_feed.SetFunction(&StepperRunSpeed);
  bw_feed.SetFunction(&StepperRunSpeed);
  ps_feed.SetFunction(&StepperNoRun);

  // Setting parameters

  // Check wheather we have to load default parameters
  int eeAddress = 0;
  byte number;
  EEPROM.get( eeAddress, number);
  int state = digitalRead (buttonPinE);

  if(state == LOW || number != MAGIC_NUMBER)
  {
  #ifdef LIN_SPEED
    strcpy( param[par_speed].name, "Speed");
    param[par_speed].id = par_speed;
    param[par_speed].value = MAX_LIN_SPEED;
    param[par_speed].min = MIN_LIN_SPEED;
    param[par_speed].max = MAX_LIN_SPEED;
    param[par_speed].inc = INC_LIN_SPEED;
  #else
    strcpy( param[par_speed].name, "Speed");
    param[par_speed].id = par_speed;
    param[par_speed].value = MAX_MOTOR_SPEED;
    param[par_speed].min = MIN_MOTOR_SPEED;
    param[par_speed].max = MAX_MOTOR_SPEED;
    param[par_speed].inc = INC_MOTOR_SPEED;
  #endif //LIN_SPEED
  
    strcpy( param[par_fwtime].name, "FW time");
    param[par_fwtime].id = par_fwtime;
    param[par_fwtime].value = (float)fw_feed.time_period / 1000;
    param[par_fwtime].max = MAX_FW_TIME;
    param[par_fwtime].min = MIN_FW_TIME;
    param[par_fwtime].inc = INC_FW_TIME;
  
    strcpy( param[par_pause].name, "Pause");
    param[par_pause].id = par_pause;
    param[par_pause].value = (float)ps_feed.time_period / 1000;
    param[par_pause].max = MAX_PAUSE;
    param[par_pause].min = MIN_PAUSE;
    param[par_pause].inc = INC_PAUSE;
  
    strcpy( param[par_bwtime].name, "BW time");
    param[par_bwtime].id = par_bwtime;
    param[par_bwtime].value = (float)bw_feed.time_period / 1000;
    param[par_bwtime].max = MAX_BW_TIME;
    param[par_bwtime].min = MIN_BW_TIME;
    param[par_bwtime].inc = INC_BW_TIME;
  
    strcpy( param[par_accel].name, "Accel");
    param[par_accel].id = par_accel;
    param[par_accel].value = MIN_ACCEL_TIME;
    param[par_accel].max = MAX_ACCEL_TIME;
    param[par_accel].min = MIN_ACCEL_TIME;
    param[par_accel].inc = INC_ACCEL_TIME;
  
    strcpy( param[par_stmode].name, "Mode");
    param[par_stmode].value = MIN_RUN_MODE; // First mode
    param[par_stmode].id = par_stmode;
    param[par_stmode].min = MIN_RUN_MODE;
    param[par_stmode].max = MAX_RUN_MODE;
    param[par_stmode].inc = INC_RUN_MODE;
  }
  else
    LoadParams(); 
  PrintParams();// for DEBUGGING
    

  // Initializing Timer1, by default 1 sec
  Timer1.initialize(1000000);
  Timer1.stop();


  // Populating run mode with actions (in order of appearance in the cycle)
  g_mode.AddAction(&fw_feed);
  g_mode.AddAction(&bw_feed);
  g_mode.AddAction(&ps_feed);

  _state == ST_SELECT; // Set in select state

  UpdateGlobalSettings(); // Sync-up params
  _update_display = true;
}

float Lin2StepSpeed(float linear_speed) /// make it inlined
{
  // Linear motion speed is defined by the formula:
  // MotionSpeed = ( MotorSpeed / STEPS_PER_ROTATION ) * 60 * (PULLEY_DIA / 1000) * PI
  // MotorSpeed = ( MotionSpeed * STEPS_PER_ROTATION * 1000 ) / (PULLEY_DIA * PI * 60);

  return ( linear_speed * STEPS_PER_ROTATION * 1000 ) / (PULLEY_DIA * PI * 60);
}

// Call this funciton when changed some settings using buttons/encoder
// Parameters are copied to global settings
void UpdateGlobalSettings()
{
  // Updating selected values
  // Moroe speed depends on whether it set in linear or motor speed
  float maxspeed;
#ifdef LIN_SPEED
  _MotorSpeed = Lin2StepSpeed( param[par_speed].value );
  maxspeed = Lin2StepSpeed( MAX_LIN_SPEED );
#else
  _MotorSpeed = param[par_speed].value;
  maxspeed = MAX_MOTOR_SPEED;
#endif // LIN_SPEED
  stepper.setMaxSpeed(maxspeed);
  stepper.setSpeed(_MotorSpeed);
  _MotorAccelTime = param[par_accel].value;
  _start_mode = (byte)param[par_stmode].value;

  fw_feed.time_period = param[par_fwtime].value * 1000; // Convert FP sec into ms
  bw_feed.time_period = param[par_bwtime].value * 1000;
  ps_feed.time_period = param[par_pause].value * 1000;

  g_mode.Reset(); // Line-up actions in the mode

  Serial.println("UpdateGlobalSettings");
}

// Call this function once when starting a cycle
void UpdateStartSettings()
{
  //Timer1.stop(); // Need to stop timer as it might continue counting from previous cycle
  reset_timer = true; // Flag to reset timer when startina a cycle
  _is_newaction = true;
  g_mode.Reset(); // Make Actions in order for a new cycle
}


void MotorTest()
{
  //Serial.println(_MotorSpeed);
  if (_state == ST_ONRUN)
    stepper.runSpeed();
}

void Run_Modes()
{
  if (_state == ST_ONRUN)
  {
    // Get the ongoing action from the current mode
    Action* ac = g_mode.GetAction();

    if (_is_newaction) {
      Serial.println("New action");

      long timer = ac->time_period;
      if (timer == 0)
      {
        // Set next action and break
        // No need to stop timer, reset action - it's done in previous action
        g_mode.NextAction();
        return;
      }
      _is_newaction = false;
      // Action just started.
      UpdateMotorSpeed(ac->dir); // CW/CCW/0 dir
      if (reset_timer) // may be not needed???
      {
        // Set up the timer
        Serial.println("Timer start - before");
        Timer1.setPeriod(timer * 1000);
        Timer1.start();
        Serial.println("Timer start - after");
      }
    }
    if (Timer1.isTime() != true) { // Timer is not finished
      ac->CallFunction();
    }
    else
    { // Action time is over
      Timer1.stop();
      reset_timer = true; // reset timer next time
      Serial.println("Timer stop");
      // Toggle next action in the current mode
      g_mode.NextAction();
      _is_newaction = true;
    }
  }
}


void UpdateMotorSpeed(float dir) /// make it inlined
{
  stepper.setSpeed(_MotorSpeed * dir);
  Serial.print("Motor Speed: ");
  Serial.println(stepper.speed());
}


void Display()
{
  if (_update_display)
  {
    lcd.clear();
    _update_display = false;

    if (_state == ST_ONRUN)
    { // Display Speed

      lcd.setCursor(0, 0);
      lcd.print("Speed: ");
      lcd.setCursor(8, 0);
      lcd.print(param[par_speed].value);
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print(param[_selected_param].name);
      if (_MenuLevel == MENU_TOP)
        lcd.print("  ");
      else
        lcd.print("->");
      lcd.print(param[_selected_param].value);
    }
  }
}

// Assumed to call in ST_ONRUN state
void StopCycle()
{
  // Stop motor with decceleration
  if( stepper.DeccelFromSpeed(_MotorAccelTime) )
  {
    Serial.println("=====================");
    /// Maybe need to check if the motor is in FW feed
    while (stepper.run())
      ;
  }
  // Change state to mode select
  _state = ST_SELECT;
  // Stop timer and reset it's couting register
  Timer1.stop();
  TCNT1 = 1;
}

void ButtonS(bool buttonPressed)
{
  if (_state == ST_ONRUN)
  {
    switch (_start_mode)
    {
      case st_2T: 
        if (buttonPressed)
          // It's strange that we are here - S button cannot be pressed again without exiting ST_ONRUN
          // But there is nothing to do, it's already running. Just sending a warning.
          Serial.println("We should not be here!!! Wrong button press");
        else { // Button released. It's a stop command
          StopCycle();
          _update_display = true;}
        break;
      case st_4T: 
        if (buttonPressed) {// It's a stop command
          StopCycle();
          _update_display = true;}
        // else, there is nothisn to do
        break;
      default:
        Serial.println("We should not be here!!! Wrong start mode.");
        break;
    }
  }
  else // _state == ST_SELECT
  {
    if (buttonPressed)
    {
      UpdateStartSettings();
      _state = ST_ONRUN; // Enable running
      _update_display = true;
    }
  }
}

void ButtonP()
{
  Serial.println("Button P");
}

void ButtonA(bool postDelay)
{
  Serial.println("Button A");

  if (_state == ST_SELECT)
  {
    // change of parameter selection
    _selected_param += 1; // iterate by index of parameter
    if (_selected_param >= NUM_PARAM)
      _selected_param = 0;
    if (postDelay)
      delay(300); // to avoid rapid change of selection
    _update_display = true;
  }
}

void ButtonB(bool postDelay)
{
  Serial.println("Button B");

  if (_state == ST_SELECT)
  {
    param[_selected_param].value += param[_selected_param].inc; // incrementing selected parameter value
    if (param[_selected_param].value > param[_selected_param].max)
      param[_selected_param].value = param[_selected_param].min;

    if (postDelay)
      delay(200); // to avoid rapid change of selection
    _update_display = true;
  }
}
void ButtonC(bool postDelay)
{
  Serial.println("Button C");

  if (_state == ST_SELECT)
  {
    // change of parameter selection
    _selected_param -= 1; // iterate by index of parameter
    if (_selected_param < 0)
      _selected_param = NUM_PARAM - 1;
    if (postDelay)
      delay(300); // to avoid rapid change of selection
    _update_display = true;
  }
}
void ButtonD(bool postDelay)
{
  Serial.println("Button D");

  if (_state == ST_SELECT)
  {
    param[_selected_param].value -= param[_selected_param].inc; // incrementing selected parameter value
    if (param[_selected_param].value < param[_selected_param].min)
      param[_selected_param].value = param[_selected_param].max;

    if (postDelay)
      delay(200); // to avoid rapid change of selection
    _update_display = true;

  }
}

// Handle buttons that start an action
void StartButtons()
{
  byte buttonState = digitalRead (buttonPinS);
  if (buttonState != _buttonOldState_S) {
    if (millis () - _buttonPressTime_S >= debounceTime) { // debounce
      _buttonPressTime_S = millis ();  // when we closed the switch
      _buttonOldState_S =  buttonState;  // remember for next time
      if (buttonState == LOW) {
        Serial.println ("button S closed"); // DEBUGGING: print that button has been closed
        ButtonS(true);
      }
      else {
        Serial.println ("button S opened"); // DEBUGGING: print that button has been opened
        ButtonS(false);
      }
    }  // end if debounce time up
  } // end of state change
}


// Isolate select buttons in case we change to substitute them with select encoder
void SelectButtons()
{
  if (_state != ST_ONRUN)
  { // Handle all buttons
    byte pressed_button;
/*    if (digitalRead(buttonPinS) == LOW)
      pressed_button = bt_S;*/
    if (digitalRead(buttonPinP) == LOW)
      pressed_button = bt_P;
    else if (digitalRead(buttonPinA) == LOW)
      pressed_button = bt_A;
    else if (digitalRead(buttonPinB) == LOW)
      pressed_button = bt_B;
    else if (digitalRead(buttonPinC) == LOW)
      pressed_button = bt_C;
    else if (digitalRead(buttonPinD) == LOW)
      pressed_button = bt_D;
    else
      pressed_button = bt_Non;

    switch (pressed_button)
    {
      case bt_P: // Load
        ButtonP();
        break;
      case bt_A: // Select-Up
        ButtonA(true);
        break;
      case bt_C: // Select-Down
        ButtonC(true);
        break;
      case bt_B: // Left
        ButtonB(true);
        break;
      case bt_D: // Right
        ButtonD(true);
        break;
      default:
        break;
    }
    if ((pressed_button == bt_A) || (pressed_button == bt_B) || (pressed_button == bt_C) || (pressed_button == bt_D))
    {
      UpdateGlobalSettings();
    }
  }
}

void RotaryMenu() { //This handles the bulk of the menu functions without needing to install/include/compile a menu library
  // No modes selection if not in SELECT state
  if (_state != ST_SELECT)
    return;

  byte buttonState = digitalRead (buttonPinE);
  if (buttonState != _buttonOldState_E) {
    if (millis () - _buttonPressTime_E >= debounceTime) { // debounce
      _buttonPressTime_E = millis ();  // when we closed the switch
      _buttonOldState_E =  buttonState;  // remember for next time
      if (buttonState == LOW) {
        Serial.println ("Button closed"); // DEBUGGING: print that button has been closed
        _buttonPressed_E = 1;
      }
      else {
        Serial.println ("Button opened"); // DEBUGGING: print that button has been opened
        _buttonPressed_E = 0;
      }
    }  // end if debounce time up
  } // end of state change

  if (_buttonPressed_E)
  { // Tougggle menu level
    if (_MenuLevel == MENU_TOP) _MenuLevel = MENU_BOT;
    else _MenuLevel = MENU_TOP;
    _buttonPressed_E = 0; // reset the button status so one press results in one action
    _update_display = true;
    UpdateGlobalSettings();
    // Store params to eeprom here, once encoder button is pressed
    StoreParams();
  }


  if (oldEncPos != encoderPos) {
    Serial.println(encoderPos);// DEBUGGING. Sometimes the serial monitor may show a value just outside modeMax due to this function. The menu shouldn't be affected.
  
  bool INC_TRANS = false; // Incremental transition, e.g. from 255 to 0
  if (oldEncPos == MAX_ENC_POS && encoderPos == 0) INC_TRANS = true;
  bool DEC_TRANS = false; // Decremental transition, e.g. from 0 to 255
  if (oldEncPos == 0 && encoderPos == MAX_ENC_POS) DEC_TRANS = true;

    switch (_MenuLevel)
    {
      case MENU_TOP: // Top menu level
        if (((encoderPos > oldEncPos) && !DEC_TRANS ) || INC_TRANS) {
          ButtonA(false);
        }
        if (((encoderPos < oldEncPos) && !INC_TRANS ) || DEC_TRANS) {
          ButtonC(false);
        }
        break;
      case MENU_BOT: // Bottom (Parameter) menu level
        if (((encoderPos > oldEncPos) && !DEC_TRANS ) || INC_TRANS) {
          ButtonB(false);
        }
        if (((encoderPos < oldEncPos) && !INC_TRANS ) || DEC_TRANS) {
          ButtonD(false);
        }
        break;
      default:
        break;
    }
    oldEncPos = encoderPos;
    UpdateGlobalSettings();
  }
}


void loop()
{
  Display();
  SelectButtons();
  StartButtons();
  RotaryMenu();
  //MotorTest();
  Run_Modes();
}
