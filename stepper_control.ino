// Copyright (C) 2017 Vladimir Tsymbal
// e-mail: vladimir.tsymbal@web.de
//
// This software is furnished "as is", without technical support, and with no
// warranty, express or implied, as to its usefulness for any purpose.


#include <TimerOne.h>
#include <rgb_lcd.h>
#include <Wire.h>


#include <AccelStepper.h>


#define STEPS_PER_ROTATION 200
#define MIN_RUN_MODE 0 // All 4 run modes: 
#define MAX_RUN_MODE 3 // 0-FW, 1-BW, 2-FW-PS, 3-FW-PS-BW
#define MAX_MOTOR_SPEED 1000 // Maximum speed, steps/s
#define MIN_MOTOR_SPEED 100 // Minimum speed, steps/s
#define MAX_MOTOR_FEED 2000 // Maximum feed, steps
#define MIN_MOTOR_FEED 20 // Minimum feed, steps
#define MAX_FW_TIME 8 // Maximum forward feed time, s
#define MIN_FW_TIME 0 // Minimum forward feed time, s
#define MAX_BW_TIME 8 // Maximum backward feed time, s
#define MIN_BW_TIME 0 // Minimum backward feed time, s
#define MAX_PAUSE 1 // Maximum pause length, s
#define MIN_PAUSE 0 // Minimum pause length, s

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::HALF4WIRE, 2, 4, 3, 5); // Defaults to 4 pins on 2, 3, 4, 5

// Fefine an lcd screen
rgb_lcd lcd;

const int colorR = 100;
const int colorG = 100;
const int colorB = 0;

const int buttonPinA = 6;
const int buttonPinB = 7;
const int buttonPinC = 8;
const int buttonPinD = 9;



#define ST_SELECT 1
#define ST_ONRUN 0

// Types of modes (can be added more combinations)
enum run_mode
{
  rm_FWD, // Forward
  rm_BKW, // Backward
  rm_FWP, // Forward-Pause
  rm_FPB, // Forward-Pause-Backward
  RUN_MODES_NUM
};
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
};

typedef struct params
{
  float value;
  float inc;
  int disp;
  int max;
  int min;
  char name[5];
} Param;
#define NUM_PARAM 6
int selected_param = 0;
Param param[NUM_PARAM];

// Action types with their timing parameters
class Action
{
  public:
  Action(byte type, bool dir) : act_type(type), dir_CW(dir){}
  byte act_type;
  long time_period; // ms
  bool dir_CW;
  bool is_newaction;
  void Reset(){is_newaction=true;}
//  void (*func)();
};

// According to the 3 action types:
Action fw_feed(ac_FW, true);
Action bw_feed(ac_BW, false);
Action ps_feed(ac_PS, true); 

// The Mode class combines one or several actions,
// being called one by one in a loop
// Actions are global objects, the calass only manages acccess
class Mode
{
public:
  void AddAction(Action* p);
  Action* GetAction();
  void NextAction(); // Call to change for a next Action
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
Action* Mode::GetAction()
{
  return list[curr];
}
void Mode::NextAction()
{
  curr++;
  if(curr == N)
    curr = 0;
  list[curr]->Reset();
}
void Mode::Reset()
{
  for(curr=0; curr<N; curr++)
    list[curr]->Reset();
  curr = 0;
}
  
byte g_mode = 0; // Global index of current Mode
Mode run_mode_list[RUN_MODES_NUM]; // Global list of available Modes

// Global updatable variables
float _MotorSpeed; // steping motor speed, steps/sec
float _MotorAccel = MAX_MOTOR_SPEED; //motor acceleration, steps/sec
float _FeedSteps = 400; // number of steps for a single feed loop, steps
//bool _MotorDirCW;

float _Pos=0; // Current position of motor, steps
//bool new_loop;

bool state = ST_SELECT;
bool update_display = true;
volatile bool timer_flag = false;
bool reset_timer = true;

// When Timer is over, rise a flag
void timerIsr(){
  timer_flag = true; }

void setup()
{
  Serial.begin(9600);
  
  state == ST_SELECT; // By default stay in select state

  pinMode(buttonPinA, INPUT_PULLUP);
  pinMode(buttonPinB, INPUT_PULLUP);
  pinMode(buttonPinC, INPUT_PULLUP);
  pinMode(buttonPinD, INPUT_PULLUP);

  lcd.begin(16, 2);    
  lcd.setRGB(colorR, colorG, colorB);

// Initial action parameters
  fw_feed.time_period = 1000; // ms
  fw_feed.Reset();
  bw_feed.time_period = 1000; // ms
  bw_feed.Reset();
  ps_feed.time_period = 0; // ms
  ps_feed.Reset();


  // Setting parameters for display
  strcpy( param[0].name, "Mode");
  param[0].value = 0; // First mode
  param[0].min = MIN_RUN_MODE;
  param[0].max = MAX_RUN_MODE;
  param[0].inc = 1;
  
  strcpy( param[1].name, "Speed");
  param[1].value = MAX_MOTOR_SPEED;
  param[1].min = MIN_MOTOR_SPEED;
  param[1].max = MAX_MOTOR_SPEED;
  param[1].inc = 1;
  
  strcpy( param[2].name, "Feed");
  param[2].value = MIN_MOTOR_FEED;
  param[2].max = MAX_MOTOR_FEED;
  param[2].min = MIN_MOTOR_FEED;
  param[2].inc = 1;

  strcpy( param[3].name, "FWtim");
  param[3].value = (float)fw_feed.time_period / 1000;
  param[3].max = MAX_FW_TIME;
  param[3].min = MIN_FW_TIME;
  param[3].inc = 0.1;

  strcpy( param[4].name, "BWtim");
  param[4].value = (float)bw_feed.time_period / 1000;
  param[4].max = MAX_BW_TIME;
  param[4].min = MIN_BW_TIME;
  param[4].inc = 0.1;
 
  strcpy( param[5].name, "Pause");
  param[5].value = (float)ps_feed.time_period / 1000;
  param[5].max = MAX_PAUSE;
  param[5].min = MIN_PAUSE;
  param[5].inc = 0.1;

  
  stepper.setMaxSpeed(param[1].value); // bad, absolute index
  stepper.setSpeed(param[1].value); // bad, absolute index
  update_display = true;

 
  //_MotorDirCW = true;
  // Initializing Timer1 and assigning ISR handler
  Timer1.initialize(1000000); 
  Timer1.attachInterrupt( timerIsr );
  Timer1.stop();

// Populating run modes with actions
// Mode FWD:
  run_mode_list[rm_FWD].AddAction(&fw_feed);
  run_mode_list[rm_FWD].Reset();
// Mode BKD:
  run_mode_list[rm_BKW].AddAction(&bw_feed);
  run_mode_list[rm_BKW].Reset();
// Mode FWP:
  run_mode_list[rm_FWP].AddAction(&fw_feed);
  run_mode_list[rm_FWP].AddAction(&ps_feed);
  run_mode_list[rm_FWP].Reset();
// Mode FPB:
  run_mode_list[rm_FPB].AddAction(&fw_feed);
  run_mode_list[rm_FPB].AddAction(&ps_feed);
  run_mode_list[rm_FPB].AddAction(&bw_feed);
  run_mode_list[rm_FPB].Reset();
}

// Call this funciton when changed some settings using buttons
// Parameters are copied to global settings
void UpdateGlobalSettings()
{
    // Updating selected values - // bad pactice, absolute index,  but will change later
    g_mode = (byte)param[0].value; // Set curren tmode
    _MotorSpeed = param[1].value;
    _MotorAccel = _MotorSpeed - _MotorSpeed / 16; // Setting acceleration speed 6.25% slower than current motor speed
    _FeedSteps = param[2].value;

    fw_feed.time_period = param[3].value * 1000; // Convert FP sec into ms
    bw_feed.time_period = param[4].value * 1000;
    ps_feed.time_period = param[5].value * 1000;

    run_mode_list[g_mode].Reset();

Serial.print("UpdateGlobalSettings");
Serial.println("");

}


void MotorTest()
{
//Serial.print(_MotorSpeed);
//Serial.println("");

  stepper.runSpeed();
}

byte act_type = ac_FW;
bool is_newaction = true;


void Run_Modes()
{
  if(state == ST_ONRUN)
  {
    // Get the ongoing action from the current mode
    Mode* mode = &run_mode_list[g_mode];  
    Action* ac = mode->GetAction();
    switch(act_type)
    {
      case ac_FW:
        // Action: FW Feed
          if(is_newaction){
            long timer = fw_feed.time_period;
            if(timer == 0)
            {
              // Set next action and break
              // No need to stop timer, reset action - it's done in previous action
              act_type = ac_PS;
              break;
            }            
            is_newaction = false;
            // Action just started. 
            UpdateMotorSpeed(true); // CW dir
            if(reset_timer)
            {
              // Set up the timer
              //long timer = fw_feed.time_period * 1000;
              Timer1.setPeriod(timer * 1000);
              //Timer1.start();
Serial.print("FW Timer start ");
Serial.print(timer);
Serial.println("");
              
            }
          }
          if (timer_flag != true){ // Timer is not finished
            //ac->Func(); //Motor step FW //////////////
            stepper.runSpeed();            
            }
          else
          { // Action time is over
            Timer1.stop();
            timer_flag = false; // timer flag down
            reset_timer = true; // reset timer next time

Serial.print("FW Timer stop");
Serial.println("");

            // Toggle next action in the current mode
            act_type = ac_PS;
            is_newaction = true;
          }
        break;
///////////////////////////////////////////////////////////////////////////////
      case ac_PS:
        // Action: PS Feed
        //if(!is_finished){
          if(is_newaction){
            long timer = ps_feed.time_period;
            if(timer == 0)
            {
              // Set next action and break
              // No need to stop timer, reset action - it's done in previous action
              act_type = ac_BW;
              break;
            }            
            is_newaction = false;
            // Action just started. 
            UpdateMotorSpeed(true); // CW dir
            if(reset_timer)
            {
              // Set up the timer
              //long timer = ps_feed.time_period * 1000;
              Timer1.setPeriod(timer * 1000);
              //Timer1.start();
              
Serial.print("PS Timer start ");
Serial.print(timer);
Serial.println("");
            }
          }
          if (timer_flag != true) // Timer is not finished
            //ac->Func(); 
            int i = 1;
            //delay(1); // instead of motor stepping 
          else
          { // Action time is over
            Timer1.stop();
            timer_flag = false; // timer flag down
            reset_timer = true; // reset timer next time

Serial.print("PS Timer stop");
Serial.println("");

            // Toggle next action in the current mode
            act_type = ac_BW; 
            is_newaction = true;
          }
        //}
        break;

///////////////////////////////////////////////////////////////////////////////      
      case ac_BW:
        // Action: BW Feed
        //if(!is_finished){
          if(is_newaction){
            long timer = bw_feed.time_period;
            if(timer == 0)
            {
              // Set next action and break
              // No need to stop timer, reset action - it's done in previous action
              act_type = ac_FW;
              break;
            }            
            is_newaction = false;
            // Action just started. 
            UpdateMotorSpeed(false); // CCW dir
            if(reset_timer)
            {
              // Set up the timer
              //long timer = bw_feed.time_period * 1000;
              Timer1.setPeriod(timer * 1000);
              //Timer1.start();
              
Serial.print("BW Timer start ");
Serial.print(timer);
Serial.println("");
            }
          }
          if (timer_flag != true){ // Timer is not finished
            //ac->Func(); //Motor step FW ///////////////////////////////////////////
//Serial.print("BW Speed: ");
//Serial.print(stepper.speed());
//Serial.println("");

            stepper.runSpeed();
          }
          else
          { // Action time is over
            Timer1.stop();
            timer_flag = false; // timer flag down
            reset_timer = true; // reset timer next time

Serial.print("BW Timer stop");
Serial.println("");

            // Toggle next action in the current mode
            act_type = ac_FW; 
            is_newaction = true;
          }
        //}
        break;
/////////////////////////////////////////////////////////////////////////////////////////

      default:
        break;
    }
  }
}
/*
void Run_Modes()
{
  if(state == ST_ONRUN)
  {
    // Get the ongoing action from the current mode
    Mode* mode = &run_mode_list[g_mode];  
    Action* ac = mode->GetAction();
    byte act_type = ac->act_type;
    
    switch(act_type)
    {
      case ac_FW:
      case ac_BW:
        // Action: FW/BW Feed
        if(!ac->is_finished){
          if(ac->is_newaction){
            ac->is_newaction = false;
            // Action just started. 
            UpdateMotorSpeed(ac->dir_CW);
            if(reset_timer)
            {
              // Set up the timer
              long timer = ac->time_period * 1000000;
              Timer1.setPeriod(timer);
              //Timer1.start();
Serial.print("Timer start ");
Serial.print(timer);
Serial.println("");

            }

          }
          if (timer_flag != true)
            //ac->Func(); //Motor step FW ///////////////////////////////////////////
            stepper.runSpeed();
          else
          { // Action time is over
            ac->is_finished = true;
            Timer1.stop();
            timer_flag = false; // timer flag down
            reset_timer = true; // reset timer next time

Serial.print("Timer stop");
Serial.println("");

            // Toggle next action in the current mode
            mode->NextAction();
          }
        }
        break;
//      case ac_BW:
//        stepper.runSpeed();        
//
//        break;

      case ac_PS: // Pause
        // Action: Pause Feed
        if(!ac->is_finished){
          if(ac->is_newaction){
            ac->is_newaction = false;
            // Action just started. 
            UpdateMotorSpeed(ac->dir_CW); // no need, but leave it for commonless
            if(reset_timer)
            {
              // Set up the timer
              long timer = ac->time_period * 1000000;
              Timer1.setPeriod(timer);
              //Timer1.start();
            }
          }
          if (timer_flag != true)
            //ac->Func(); // Later implement an Action function for sleeping
            delay(1); // Sleep for 1 ms
          else
          { // Action time is over
            ac->is_finished = true;
            Timer1.stop();
            timer_flag = false; // timer flag down
            reset_timer = true; // reset timer next time
            // Toggle next action in the current mode
            mode->NextAction();
          }
        }
        break;
      default:
        break;
    }
  }
}
*/
// Call this funciton when need to change motor settings
// Make sure it's not called frequently
/*
void UpdateMotor()
{
  stepper.setMaxSpeed(_MotorSpeed);
  _MotorDirCW ? stepper.setSpeed(_MotorSpeed) : stepper.setSpeed(-_MotorSpeed);
  stepper.setAcceleration(_MotorAccel);
}
*/
void UpdateMotorSpeed(bool isCW)
{
Serial.print(isCW);
Serial.println("");

  if(isCW)
    stepper.setSpeed(_MotorSpeed);
  else  
    stepper.setSpeed(-_MotorSpeed);
}

/*
void Motor()
{
  if(mode == mode_run)
  {
    if (new_loop)
    {
      _Pos=0;
      new_loop=0;
      stepper.setCurrentPosition(_Pos);
      if(update_motor) // Set speed parameters only when changed
      { 
        update_motor = false;
        stepper.setMaxSpeed(_MotorSpeed);
        stepper.setAcceleration(_MotorAccel);
      }
    }
    // At the moment, the feed steps = full motor cycle (acs-mov-des) 
    if(_Pos < _FeedSteps)
    {
      _Pos += _FeedSteps;
      stepper.runToNewPosition(_Pos);
    }
    else
      new_loop = 1;
  } // end if mode_run
  
}
*/

void Pause(){
  if(state == ST_ONRUN)
  {
    }
}

void Display()
{
  if(update_display)
  {
    lcd.clear();
    update_display = false;

    if(state == ST_ONRUN)
    { // Display Speed
  
      lcd.setCursor(0, 0);
      lcd.print("Speed: ");
      lcd.setCursor(8, 0);
      lcd.print(_MotorSpeed);
    }
    else
    {    
      lcd.setCursor(0, 0);
      lcd.print(param[selected_param].name);
      lcd.print(" -> ");
      lcd.print(param[selected_param].value);
    }
  }
}
void Control()
{
/*
  // Changing motorspeed during run
  float val = analogRead(0);
  float tmp = (MAX_MOTOR_SPEED - MIN_MOTOR_SPEED) / 1024.0;
  _MotorSpeed = MIN_MOTOR_SPEED + tmp * val; // Setting motor sped in a range [MIN,MAX] depending on potentiometer
  _MotorAccel = _MotorSpeed - _MotorSpeed / 16; // Setting acceleration speed 6.25% slower than current motor speed
*/
}

void ButtonNon()
{
  if(state == ST_ONRUN)
  {
    state = ST_SELECT;
  }
  //update_display = true;
}
void ButtonA()
{
  
  if(state == ST_ONRUN)
  { // nothing to do
  }
  else
  {
Serial.print("Button A");
Serial.println("");

    state = ST_ONRUN; // Enable running motor
    update_display = true;
  }
}
void ButtonB()
{
//Serial.print("Button B");
//Serial.println("");
  if(state == ST_ONRUN)
  {
    // Only changing motor speed in the run mode
  }
  else
  {
    param[selected_param].value += param[selected_param].inc; // incrementing selected parameter value
    if(param[selected_param].value > param[selected_param].max)
      param[selected_param].value = param[selected_param].min;

    delay(200); // to avoid rapid change of selection
    update_display = true;
  }
}
void ButtonC()
{
Serial.print("Button C");
Serial.println("");
  
  if(state == ST_ONRUN)
  { // nothing to do
  }
  else
  { 
    // change of parameter selection
    selected_param +=1; // iterate by index of parameter
    if (selected_param == NUM_PARAM)
      selected_param = 0;
    delay(300); // to avoid rapid change of selection
    update_display = true;
  }
}
void ButtonD()
{
Serial.print("Button D");
Serial.println("");
  if(state == ST_ONRUN)
  {
    // Only changing motor speed in the run mode
  }
  else
  {
    param[selected_param].value -= param[selected_param].inc; // incrementing selected parameter value
    if(param[selected_param].value < param[selected_param].min)
      param[selected_param].value = param[selected_param].max;

    delay(200); // to avoid rapid change of selection
    update_display = true;

  }
}


void Buttons()
{
  byte pressed_button;
  if (digitalRead(buttonPinA)==LOW)
    pressed_button = bt_A;
  else if (digitalRead(buttonPinC)==LOW)
    pressed_button = bt_C;
  else if (digitalRead(buttonPinB)==LOW)
    pressed_button = bt_B;
  else if (digitalRead(buttonPinD)==LOW)
    pressed_button = bt_D;
  else
    pressed_button = bt_Non;

  switch(pressed_button)
  {
    case bt_Non: // No buttons pressed
      ButtonNon();
      break; 
    case bt_A: // Run/Stop
      ButtonA();
      break;
    case bt_C: // Select
      ButtonC();
      break;
    case bt_B: // Left/Down
      ButtonB();
      break;
    case bt_D: // Right/Up
      ButtonD();
      break;
    default:
      break;
  }
  if((pressed_button == bt_B) || (pressed_button == bt_C) || (pressed_button == bt_D))
  {
    UpdateGlobalSettings();
    //UpdateMotor();
  }
}

void loop()
{
  Display();
  Buttons();
//  MotorTest();
  Run_Modes();
//  Control();
//  Motor();
//  Pause();
}

