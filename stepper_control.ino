// Copyright (C) 2017 Vladimir Tsymbal
// e-mail: vladimir.tsymbal@web.de
// v.0.1
// This software is furnished "as is", without technical support, and with no
// warranty, express or implied, as to its usefulness for any purpose.


#include <TimerOne.h>
#include <rgb_lcd.h>
#include <Wire.h>


#include <AccelStepper.h>


#define STEPS_PER_ROTATION 200
#define MIN_RUN_MODE 0 // 2 tackt run mode 
#define MAX_RUN_MODE 1 // 4 tackt run mode
#define MAX_MOTOR_SPEED 1000 // Maximum speed, steps/s
#define MIN_MOTOR_SPEED 100 // Minimum speed, steps/s
#define MAX_ACCEL_TIME 5 // Maximum time for acceleration, s
#define MIN_ACCEL_TIME 1 // Minimum time for acceleration, s
#define MAX_FW_TIME 8 // Maximum forward feed time, s
#define MIN_FW_TIME 0 // Minimum forward feed time, s
#define MAX_BW_TIME 8 // Maximum backward feed time, s
#define MIN_BW_TIME 0 // Minimum backward feed time, s
#define MAX_PAUSE 1 // Maximum pause length, s
#define MIN_PAUSE 0 // Minimum pause length, s

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::HALF4WIRE, 2, 4, 3, 5); // Defaults to 4 pins on 2, 3, 4, 5

// Define an LCD screen
rgb_lcd lcd;

const int colorR = 100;
const int colorG = 100;
const int colorB = 0;

const int buttonPinA = 6;
const int buttonPinB = 7;
const int buttonPinC = 8;
const int buttonPinD = 9;
const int buttonPinS = 10;
const int buttonPinP = 11;


#define ST_SELECT 1
#define ST_ONRUN 0

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
enum start_mode
{
  st_2T,
  st_4T,
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
int selected_param = 0;
Param param[NUM_PARAM];
const Param* GetParam(byte id)
{
  for(byte i=0; i<NUM_PARAM; i++)
  {
    if( param[i].id == id )
    return &param[i];
  }
}

// Action types with their timing parameters
class Action
{
  public:
  Action(byte type, bool dir) : act_type(type), dir_CW(dir){}
  byte act_type;
  long time_period; // ms
  bool dir_CW;
//  bool is_newaction;
//  void Reset(){is_newaction=true;}
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
Action* Mode::GetAction(){
  return list[curr];}
  
int Mode::GetActionsNumber(){
  return N;}
  
void Mode::NextAction(){
  curr++;
  if(curr == N)
    curr = 0;
  //list[curr]->Reset();
}
void Mode::Reset(){
//  for(curr=0; curr<N; curr++)
//    list[curr]->Reset();
  curr = 0;
}
  
Mode g_mode; // Global mode

// Global updatable variables
float _MotorSpeed; // steping motor speed, steps/sec
float _MotorAccel = MAX_MOTOR_SPEED; //motor acceleration, steps/sec
float _FeedSteps = 400; // number of steps for a single feed loop, steps
//bool _MotorDirCW;

float _Pos=0; // Current position of motor, steps

bool state = ST_SELECT;
bool update_display = true;
volatile bool timer_flag = false;
bool reset_timer = true;
bool is_newaction = true;

byte start_mode = 0;

// When Timer is over, rise a flag
void timerIsr(){
Serial.print("ISR");
Serial.println("");

  timer_flag = true; }

void setup()
{
  Serial.begin(9600);
  
  state == ST_SELECT; // By default stay in select state

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
  param[par_stmode].inc = 1;


  
  stepper.setMaxSpeed(param[par_speed].value); 
  stepper.setSpeed(param[par_speed].value); 
  update_display = true;

 
  //_MotorDirCW = true;
  // Initializing Timer1 and assigning ISR handler
  Timer1.initialize(1000000); 
  Timer1.stop();
  Timer1.attachInterrupt( timerIsr );
  //Timer1.start();
  

// Populating run mode with actions (in order of appearance in the cycle)
  g_mode.AddAction(&fw_feed);
  g_mode.AddAction(&ps_feed);
  g_mode.AddAction(&bw_feed);

  UpdateGlobalSettings();
}

// Call this funciton when changed some settings using buttons
// Parameters are copied to global settings
void UpdateGlobalSettings()
{

    // Updating selected values - // bad pactice, absolute index,  but will change later
    start_mode = (byte)param[par_stmode].value; // Set current mode
    _MotorSpeed = param[par_speed].value;
    _MotorAccel = param[par_accel].value;
  

    fw_feed.time_period = param[par_fwtime].value * 1000; // Convert FP sec into ms
    bw_feed.time_period = param[par_bwtime].value * 1000;
    ps_feed.time_period = param[par_pause].value * 1000;

    g_mode.Reset();

Serial.print("UpdateGlobalSettings");
Serial.println("");

}

// Call this function once when starting a cycle
void UpdateStartSettings()
{
  //Timer1.stop(); // Need to stop timer as it might continue counting from previous cycle 
  timer_flag = false;
  reset_timer = true; // Flag to reset timer when startina a cycle
  is_newaction = true;
  g_mode.Reset(); // Make Actions in order for a new cycle
}


void MotorTest()
{
//Serial.print(_MotorSpeed);
//Serial.println("");
  if(state == ST_ONRUN)
    stepper.runSpeed();
}


void Run_Modes()
{
  if(state == ST_ONRUN)
  {
    // Get the ongoing action from the current mode
    Action* ac = g_mode.GetAction();
    byte act_type = ac->act_type;

    switch(act_type)
    {
      case ac_FW:
        // Action: FW Feed
          if(is_newaction){
Serial.print("FW new action");
Serial.println("");

            long timer = ac->time_period;
            if(timer == 0)
            {
              // Set next action and break
              // No need to stop timer, reset action - it's done in previous action
              g_mode.NextAction();
              break;
            }            
            is_newaction = false;
            // Action just started. 
            UpdateMotorSpeed(true); // CW dir
            if(reset_timer)
            {
              // Set up the timer
Serial.print("FW Timer start - before");
//Serial.print(timer);
Serial.println("");
              Timer1.setPeriod(timer * 1000);
Serial.print("FW Timer start - after");
Serial.println("");
            }
          }
          if (timer_flag != true){ // Timer is not finished
            //ac->Func(); //Motor step FW //////////////
            stepper.runSpeed();
        //Serial.print(stepper.speed());
        //Serial.println("");
            
            }
          else
          { // Action time is over
            Timer1.stop();
            timer_flag = false; // timer flag down
            reset_timer = true; // reset timer next time

Serial.print("FW Timer stop");
Serial.println("");

            // Toggle next action in the current mode
            g_mode.NextAction();
            is_newaction = true;
          }
        break;
///////////////////////////////////////////////////////////////////////////////
      case ac_PS:
        // Action: PS Feed
          if(is_newaction){
            long timer = ac->time_period;
            if(timer == 0)
            {
              // Set next action and break
              // No need to stop timer, reset action - it's done in previous action
              g_mode.NextAction();
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
            g_mode.NextAction(); 
            is_newaction = true;
          }
        break;

///////////////////////////////////////////////////////////////////////////////      
      case ac_BW:
        // Action: BW Feed
          if(is_newaction){
            long timer = ac->time_period;
            if(timer == 0)
            {
              // Set next action and break
              // No need to stop timer, reset action - it's done in previous action
              g_mode.NextAction();
              break;
            }            
            is_newaction = false;
            // Action just started. 
            UpdateMotorSpeed(false); // CCW dir
            if(reset_timer)
            {
              // Set up the timer
              //long timer = bw_feed.time_period * 1000;
Serial.print("BW Timer start - before");
Serial.println("");
              Timer1.setPeriod(timer * 1000);              
Serial.print("BW Timer start - after");
//Serial.print(timer);
Serial.println("");
            }
          }
          if (timer_flag != true){ // Timer is not finished
            //ac->Func(); //Motor step FW ///////////////////////////////////////////
//Serial.print("BW");
//Serial.println("");

            stepper.runSpeed();
        //Serial.print(stepper.speed());
        //Serial.println("");

          }
          else
          { // Action time is over
            Timer1.stop();
            timer_flag = false; // timer flag down
            reset_timer = true; // reset timer next time

Serial.print("BW Timer stop");
Serial.println("");

            // Toggle next action in the current mode
            g_mode.NextAction(); 
            is_newaction = true;
          }
        break;
/////////////////////////////////////////////////////////////////////////////////////////

      default:
        break;
    }
  }
}

void UpdateMotorSpeed(bool isCW)
{
  if(isCW)
    stepper.setSpeed(_MotorSpeed);
  else  
    stepper.setSpeed(-_MotorSpeed);
Serial.print(stepper.speed());
Serial.println("");

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
      lcd.print("->");
      lcd.print(param[selected_param].value);
    }
  }
}


void ButtonNon()
{
  if(state == ST_ONRUN)
  {    
    switch (start_mode)
    {
    case st_4T:
      // nothing to do, it's running
      break;
    case st_2T:
      state = ST_SELECT;
      update_display = true;
      // Stop cycle, call desceleratng function here /////////////!!!!!!!!!!!!!!!
 Timer1.stop(); // Create a stop procedure with stopping timer
 TCNT1 = 1;
      delay(200); // Remove it!
 
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
  if(state == ST_ONRUN)
  {
    switch (start_mode)
    {
      case st_2T: //  nothing to do, it's already running
        break;
      case st_4T: // stop cycle 
        state = ST_SELECT;
        update_display = true;
        //call desceleratng function here ///////////////////!!!!!!!!!!!!!!
Timer1.stop(); // Create a stop procedure with stopping timer
        delay(200); // Remove it!

    Serial.print("Button S - stop");
    Serial.println("");

        break;
      default:
        break;
    }
  }
  else // state == ST_SELECT
  {
    Serial.println("");
    Serial.print("Button S - start");
    Serial.println("");

    UpdateStartSettings();
    state = ST_ONRUN; // Enable running
    update_display = true;
    delay(100); // to avoid immediate stop in 4T mode
  }
}  
  
void ButtonP()
{  
  Serial.print("Button P");
  Serial.println("");     
}

void ButtonA()
{
Serial.print("Button A");
Serial.println("");
  
  if(state == ST_ONRUN)
  { // nothing to do
  }
  else
  { 
    // change of parameter selection
    selected_param +=1; // iterate by index of parameter
    if (selected_param >= NUM_PARAM)
      selected_param = 0;
    delay(300); // to avoid rapid change of selection
    update_display = true;
  }
}

void ButtonB()
{
Serial.print("Button B");
Serial.println("");

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
    selected_param -=1; // iterate by index of parameter
    if (selected_param < 0)
      selected_param = NUM_PARAM-1;
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
  if(state == ST_ONRUN)
  { // Handle the Start button only for the sake of speed
      if (digitalRead(buttonPinS)==LOW)
        pressed_button = bt_S;
      else
        pressed_button = bt_Non;
  }
  else
  { // Handle all buttons
    if (digitalRead(buttonPinS)==LOW)
      pressed_button = bt_S;
    else if (digitalRead(buttonPinP)==LOW)
      pressed_button = bt_P;
    else if (digitalRead(buttonPinA)==LOW)
      pressed_button = bt_A;
    else if (digitalRead(buttonPinB)==LOW)
      pressed_button = bt_B;
    else if (digitalRead(buttonPinC)==LOW)
      pressed_button = bt_C;
    else if (digitalRead(buttonPinD)==LOW)
      pressed_button = bt_D;
    else
      pressed_button = bt_Non;
  }

  switch(pressed_button)
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
  if((pressed_button == bt_A) || (pressed_button == bt_B) || (pressed_button == bt_C) || (pressed_button == bt_D))
  {
    UpdateGlobalSettings();
  }
}

void loop()
{
  Display();
  Buttons();
 // MotorTest();
  Run_Modes();
//  Pause();
}
