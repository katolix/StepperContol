// Copyright (C) 2017 Vladimir Tsymbal
// e-mail: vladimir.tsymbal@web.de
// v.0.1
// This software is furnished "as is", without technical support, and with no
// warranty, express or implied, as to its usefulness for any purpose.


#include <TimerOneCTC.h>
#include <rgb_lcd.h>
#include <Wire.h>
#include <AccelStepper.h>

// User definable parameters
//===================================================================================================//

#define STEPS_PER_ROTATION 200 // Number of motor steps per one rotation
#define PULLEY_DIA 30 // Diameter of pulley, mm

#define MIN_RUN_MODE 2 // 2 tackt start mode 
#define MAX_RUN_MODE 4 // 4 tackt start mode
#define MAX_MOTOR_SPEED 1000 // Maximum speed, steps/s
#define MIN_MOTOR_SPEED 100 // Minimum speed, steps/s

#define MAX_FEED_SPEED 10 // Maximum speed, m/min
#define MIN_FEED_SPEED 0 // Minimum speed, m/min

#define MAX_ACCEL_TIME 5 // Maximum time for acceleration, s
#define MIN_ACCEL_TIME 1 // Minimum time for acceleration, s
#define MAX_FW_TIME 8 // Maximum forward feed time, s
#define MIN_FW_TIME 0 // Minimum forward feed time, s
#define MAX_BW_TIME 8 // Maximum backward feed time, s
#define MIN_BW_TIME 0 // Minimum backward feed time, s
#define MAX_PAUSE 1 // Maximum pause time, s
#define MIN_PAUSE 0 // Minimum pause time, s

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::HALF4WIRE, 2, 4, 3, 5); // Defaults to 4 pins on 2, 3, 4, 5

// Define an LCD screen
rgb_lcd lcd;
// Define LCD screen backlight color
const int colorR = 100;
const int colorG = 100;
const int colorB = 0;

// Assigned board pins to buttons
// Specific to Arduino Mega 2560 board
// You might wont to change for your board
const int buttonPinA = 6;
const int buttonPinB = 7;
const int buttonPinC = 8;
const int buttonPinD = 9;
const int buttonPinS = 10;
const int buttonPinP = 11;

//===================================================================================================//

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

int _selected_param = 0; // int type in order to be able to decrement a param from 0
Param param[NUM_PARAM];
const Param* GetParam(byte id){
  for (byte i = 0; i < NUM_PARAM; i++)  {
    if ( param[i].id == id )
      return &param[i];  }
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
      this->func = func;};
    void CallFunction() {
      return func();};
};

// Action functions
void StepperRunSpeed() {
  stepper.runSpeed();
}
void StepperNoRun() {
}

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

float _FpeedSpeed; // Feed speed defined by motor speed and deameter of pulley 


// Global flags
bool _state = ST_SELECT;
bool _update_display = true;
bool reset_timer = true;
bool _is_newaction = true;
byte _start_mode = 0;

void setup()
{
  Serial.begin(9600);
  
  pinMode(buttonPinA, INPUT_PULLUP);
  pinMode(buttonPinB, INPUT_PULLUP);
  pinMode(buttonPinC, INPUT_PULLUP);
  pinMode(buttonPinD, INPUT_PULLUP);
  pinMode(buttonPinS, INPUT_PULLUP);


  lcd.begin(16, 2);
  lcd.setRGB(colorR, colorG, colorB);


  // Initial action parameters
  fw_feed.time_period = 1000; // ms
  bw_feed.time_period = 1000; // ms
  ps_feed.time_period = 0; // ms
  fw_feed.SetFunction(&StepperRunSpeed);
  bw_feed.SetFunction(&StepperRunSpeed);
  ps_feed.SetFunction(&StepperNoRun);

  // Setting parameters for display

  strcpy( param[par_speed].name, "Speed");
  param[par_speed].id = par_speed;
  param[par_speed].value = MAX_MOTOR_SPEED;
  param[par_speed].min = MIN_MOTOR_SPEED;
  param[par_speed].max = MAX_MOTOR_SPEED;
  param[par_speed].inc = 1;

  strcpy( param[par_fwtime].name, "FW time");
  param[par_fwtime].id = par_fwtime;
  param[par_fwtime].value = (float)fw_feed.time_period / 1000;
  param[par_fwtime].max = MAX_FW_TIME;
  param[par_fwtime].min = MIN_FW_TIME;
  param[par_fwtime].inc = 0.1;

  strcpy( param[par_pause].name, "Pause");
  param[par_pause].id = par_pause;
  param[par_pause].value = (float)ps_feed.time_period / 1000;
  param[par_pause].max = MAX_PAUSE;
  param[par_pause].min = MIN_PAUSE;
  param[par_pause].inc = 0.1;

  strcpy( param[par_bwtime].name, "BW time");
  param[par_bwtime].id = par_bwtime;
  param[par_bwtime].value = (float)bw_feed.time_period / 1000;
  param[par_bwtime].max = MAX_BW_TIME;
  param[par_bwtime].min = MIN_BW_TIME;
  param[par_bwtime].inc = 0.1;

  strcpy( param[par_accel].name, "Accel");
  param[par_accel].id = par_accel;
  param[par_accel].value = MIN_ACCEL_TIME;
  param[par_accel].max = MAX_ACCEL_TIME;
  param[par_accel].min = MIN_ACCEL_TIME;
  param[par_accel].inc = 1;

  strcpy( param[par_stmode].name, "Mode");
  param[par_stmode].value = MIN_RUN_MODE; // First mode
  param[par_stmode].id = par_stmode;
  param[par_stmode].min = MIN_RUN_MODE;
  param[par_stmode].max = MAX_RUN_MODE;
  param[par_stmode].inc = 2;



  stepper.setMaxSpeed(param[par_speed].value);
  stepper.setSpeed(param[par_speed].value);
  _update_display = true;


  // Initializing Timer1, by default 1 sec
  Timer1.initialize(1000000);
  Timer1.stop();


  // Populating run mode with actions (in order of appearance in the cycle)
  g_mode.AddAction(&fw_feed);
  g_mode.AddAction(&ps_feed);
  g_mode.AddAction(&bw_feed);

  _state == ST_SELECT; // Set in select state

  UpdateGlobalSettings(); // Sync-up params
}

// Call this funciton when changed some settings using buttons
// Parameters are copied to global settings
void UpdateGlobalSettings()
{
  // Feed speed is defined by the formula:
  // FpeedSpeed = ( MotorSpeed / STEPS_PER_ROTATION ) * 60 * (PULLEY_DIA / 1000) * PI
  _MotorSpeed = ( _FpeedSpeed * STEPS_PER_ROTATION * 1000 ) / (PULLEY_DIA * PI * 60);
  
  // Updating selected values
  _start_mode = (byte)param[par_stmode].value;
  _MotorSpeed = param[par_speed].value;
  _MotorAccelTime = param[par_accel].value;


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
  //Serial.print(_MotorSpeed);
  //Serial.println("");
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


void UpdateMotorSpeed(float dir)
{
  stepper.setSpeed(_MotorSpeed * dir);
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
      lcd.print(_MotorSpeed);
    }
    else
    {
      lcd.setCursor(0, 0);
      lcd.print(param[_selected_param].name);
      lcd.print("->");
      lcd.print(param[_selected_param].value);
    }
  }
}

// Assumed to call in ST_ONRUN state 
void StopCycle()
{
// Stop motor with decceleration
  stepper.DeccelFromSpeed(_MotorAccelTime); 
  Serial.println("=====================");
  /// Maybe need to check if the motor is in FW feed
  while (stepper.run())
  ;
  // Change state to mode select
  _state = ST_SELECT;
  // Stop timer and reset it's couting register
  Timer1.stop();
  TCNT1 = 1;
}

void ButtonNon()
{
  if (_state == ST_ONRUN)
  {
    switch (_start_mode)
    {
      case st_4T:
        // nothing to do, it's running
        break;
      case st_2T:
        StopCycle();
/*
        _state = ST_SELECT;
        // Stop cycle, call desceleratng function here /////////////!!!!!!!!!!!!!!!
        Timer1.stop(); // Create a stop procedure with stopping timer
        TCNT1 = 1;
        delay(200); // Remove it!
*/
        _update_display = true;

        break;
      default:
        Serial.print("WARNING!!! We should not get in to here");
        Serial.println("");
        break;
    }
  }
}
void ButtonS()
{
  if (_state == ST_ONRUN)
  {
    switch (_start_mode)
    {
      case st_2T: //  nothing to do, it's already running
        break;
      case st_4T: // stop cycle
        Serial.println("Button S - stop");
        StopCycle();
        _update_display = true;
        break;
      default:
        break;
    }
  }
  else // _state == ST_SELECT
  {
    Serial.println("Button S - start");

    UpdateStartSettings();
    _state = ST_ONRUN; // Enable running
    _update_display = true;
    delay(200); // to avoid immediate stop in 4T mode
  }
}

void ButtonP()
{
  Serial.println("Button P");
}

void ButtonA()
{
  Serial.println("Button A");

  if (_state == ST_SELECT)
  {
    // change of parameter selection
    _selected_param += 1; // iterate by index of parameter
    if (_selected_param >= NUM_PARAM)
      _selected_param = 0;
    delay(300); // to avoid rapid change of selection
    _update_display = true;
  }
}

void ButtonB()
{
  Serial.println("Button B");

  if (_state == ST_SELECT)
  {
    param[_selected_param].value += param[_selected_param].inc; // incrementing selected parameter value
    if (param[_selected_param].value > param[_selected_param].max)
      param[_selected_param].value = param[_selected_param].min;

    delay(200); // to avoid rapid change of selection
    _update_display = true;
  }
}
void ButtonC()
{
  Serial.println("Button C");

  if (_state == ST_SELECT)
  {
    // change of parameter selection
    _selected_param -= 1; // iterate by index of parameter
    if (_selected_param < 0)
      _selected_param = NUM_PARAM - 1;
    delay(300); // to avoid rapid change of selection
    _update_display = true;
  }
}
void ButtonD()
{
  Serial.println("Button D");

  if (_state == ST_SELECT)
  {
    param[_selected_param].value -= param[_selected_param].inc; // incrementing selected parameter value
    if (param[_selected_param].value < param[_selected_param].min)
      param[_selected_param].value = param[_selected_param].max;

    delay(200); // to avoid rapid change of selection
    _update_display = true;

  }
}

// Handle the Start button only, for the sake of speed
void RunButtons()
{   
  byte pressed_button;
  if(_state == ST_ONRUN)
  {
      if (digitalRead(buttonPinS) == LOW) // Change to direct port load
        ButtonS();
      else
        ButtonNon();
  }
}

// Isolate select buttons in case we change to substitute them with select encoder
void SelectButtons()
{
  if (_state != ST_ONRUN)
  { // Handle all buttons
    byte pressed_button;
    if (digitalRead(buttonPinS) == LOW)
      pressed_button = bt_S;
    else if (digitalRead(buttonPinP) == LOW)
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
      case bt_Non: // No buttons pressed
        ButtonNon();
        break;
      case bt_S: // Start
        ButtonS();
        break;
      case bt_P: // Load
        ButtonP();
        break;
      case bt_A: // Select-Up
        ButtonA();
        break;
      case bt_C: // Select-Down
        ButtonC();
        break;
      case bt_B: // Left
        ButtonB();
        break;
      case bt_D: // Right
        ButtonD();
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

void loop()
{
  Display();
  SelectButtons();
  RunButtons();
  // MotorTest();
  Run_Modes();
}
