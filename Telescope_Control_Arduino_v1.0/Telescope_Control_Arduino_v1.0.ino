#include<Arduino.h>
#include <AccelStepper.h>
#include <NeoSWSerial.h>

//SoftwareSerial Bluetooth(11, 12); // RX, TX
NeoSWSerial Bluetooth(11, 12); // RX, TX

/////////////////////////////////////
const byte numChars = 64;
char receivedChars[numChars];

boolean newData = false;

char command[64] = {0};
int integerFromPC = 0;
float floatFromPC = 0.0;
char recvChar;

////////////////////////////////
long RA_steps_to_go = 0;
long DEC_steps_to_go = 0;

long alignRA = 0;
long alignDEC = 0;
int RA_current_speed, DEC_current_speed = 0;
int q, e, d, r, z, w, tracking_ra_flag, tracking_dec_flag, RA_manual_control, DEC_manual_control, DEC_auto_control, RA_auto_control, RA_direction_slow, DEC_direction_slow = 0;

bool motors_stopped = true;
float sidereal_tracking_speed = 0.0;
bool second_move_execution = false;
long goto_ra_bl_fix = 0;
long goto_dec_bl_fix = 0;
long previous_ra_bl_fix = 0;
long previous_dec_bl_fix = 0;
int previous_ra_goto_direction=1;
int previous_dec_goto_direction=1;
long received_ra_bl_fix = 0;
long received_dec_bl_fix = 0;
boolean motors_initialized = false;
boolean energy_saving_is_on = false;
int dec_goto_direction = 1;
int ra_goto_direction = 1;
int stop_and_restore_dir = 1;
int same_move_counter=0;
int previous_ra_backlash_fix=0;
int previous_dec_backlash_fix=0;
int current_ra_backlash_fix=0;
int current_dec_backlash_fix=0;
boolean at_initial_position=true;
long ra_initialization_steps=0;
long dec_initialization_steps=0;
long ra_stop_tracking_limit_up=0;
long ra_horizon_tracking_limit=0;
boolean tracking_reached_limit=false;
boolean auto_tracking=false;
int auto_centering_speed;
double ra_configured_max_speed ;
double dec_configured_max_speed ;
float ra_tracking_rate=0;






AccelStepper RA_stepper(AccelStepper::DRIVER, 5, 4 ); //Right Ascension's axis motor driver . Arduino's pin 4 controls the RA driver's direction pin.Pin 5 controls the driver's step pin.
AccelStepper DEC_stepper(AccelStepper::DRIVER, 3, 2); //DEClination's axis motor driver . Arduino's pin 2 controls the RA driver's direction pin.Pin 3 controls the driver's step pin.

void setup() {
  DEC_stepper.setEnablePin(9);
  RA_stepper.setEnablePin(10);
  pinMode(6, OUTPUT); //MS3 pin of the motor driver
  pinMode(7, OUTPUT); //MS2 pin of the motor driver
  pinMode(8, OUTPUT); //MS1 pin of the motor driver
  /*default microstepping mode=16x.valid for both tmc2208 and a4988 chips :*/
  digitalWrite(6, HIGH);  //MS3 pin of the motor driver
  digitalWrite(7, HIGH); //MS2 pin of the motor driver
  digitalWrite(8, HIGH); //MS1 pin of the motor driver
  digitalWrite(9, LOW); //ENABLE pin of the DEC motor driver , LOW : enabled
  digitalWrite(10, LOW); //ENABLE pin of the RA motor driver,  LOW: enable



  /*Set of the maximum Speed(in steps/sec) and Acceleration(in steps/sec^2) for the motors: */
  RA_stepper.setMaxSpeed(500);
  RA_stepper.setAcceleration(200);
  DEC_stepper.setMaxSpeed(500);
  DEC_stepper.setAcceleration(200);





 // Serial.begin(9600);
 // Serial.println(F("<Arduino is ready>"));
  Bluetooth.begin(9600);
  Bluetooth.println(F("Mount initialization..."));
  // motors_initialization();
  Bluetooth.println(F("OK, Waiting for command..."));


}

/*Initialization of the motors. Move to one direction 5000 steps,
  Next move to the opposite direction 5000 steps. Starting position remains at (0,0) steps.
  This procedure is done to make sure that, at the starting position, any back-lash is limited to one direction for each motor.
  So when the motors start to move there will be either 0 back-lash, or the maximum amount of back-lash. Not something in between.
  for testing purposes..this may prove to be redundant  */




/*Selection of X4 microstepping mode of the motor driver.It is valid for both the tmc2208 and a4988 chips*/
void microstep_mode_X4() {

  digitalWrite(6, LOW);  //MS3 pin of the motor driver
  digitalWrite(7, HIGH); //MS2 pin of the motor driver
  digitalWrite(8, LOW); //MS1 pin of the motor driver

}

/*Selection of X16 microstepping mode of the motor driver.It is valid for both the tmc2208 and a4988 chips*/
void microstep_mode_X16() {

  digitalWrite(6, HIGH);  //MS3 pin of the motor driver
  digitalWrite(7, HIGH); //MS2 pin of the motor driver
  digitalWrite(8, HIGH); //MS1 pin of the motor driver
}

/*move the RA motor*/
void define_ra_direction();
void move_RA_motor(long RA_steps) {
  digitalWrite(10, LOW); //ENABLE the RA motor driver

  if (RA_auto_control != 1) {
    RA_stepper.setCurrentPosition(RA_stepper.currentPosition() / 4);
    microstep_mode_X4();
  }
  Bluetooth.println(F("RA in moveRA position :"));
  Bluetooth.println(RA_stepper.currentPosition());


  tracking_ra_flag = 0;
  tracking_dec_flag = 0;
  RA_manual_control = 0;
  RA_auto_control = 1;

  Bluetooth.println(F("Moving RA to"));
  Bluetooth.println(RA_steps);
  alignRA = 0;

  if(!second_move_execution){
goto_ra_bl_fix=0;}
  if(second_move_execution){
goto_ra_bl_fix = previous_ra_bl_fix;}

  /*move clockwise: */
  if (RA_stepper.currentPosition() < RA_steps) {
    RA_stepper.moveTo(RA_steps + 0);
    Bluetooth.println(F("RA moving clockwise!"));
    Bluetooth.println(F("RA fix APPLIED :"));
    Bluetooth.println(0);
  }

  /*move counter-clockwise: */
  

  if (RA_stepper.currentPosition() > RA_steps)
  {
if(ra_goto_direction==1 && second_move_execution){
   // previous_ra_bl_fix=current_ra_backlash_fix;
    //  goto_ra_bl_fix = previous_ra_bl_fix;
    goto_ra_bl_fix = current_ra_backlash_fix;
       Bluetooth.println(F(" got current ra_fix !"));
}

   /* if(previous_ra_goto_direction != ra_goto_direction && !second_move_execution){
      previous_ra_bl_fix=current_ra_backlash_fix;
      goto_ra_bl_fix = previous_ra_bl_fix;
       Bluetooth.println(F("previous_ra_goto_direction: "));
    Bluetooth.println(previous_ra_goto_direction);
       Bluetooth.println(F("ra_goto_direction: "));
    Bluetooth.println(ra_goto_direction);}*/
    RA_steps_to_go = RA_steps_to_go + goto_ra_bl_fix;
    RA_stepper.moveTo(RA_steps_to_go);
    Bluetooth.println(F("RA moving counter-clockwise!"));
    Bluetooth.print(goto_ra_bl_fix);
    Bluetooth.println(F(" RA fix APPLIED !"));
  }

  
previous_ra_goto_direction =ra_goto_direction;
  define_ra_direction();
  
  if(previous_ra_goto_direction != ra_goto_direction){
     Bluetooth.println(F(" RA DIRECTION CHANGED"));
      Bluetooth.println(F("previous_ra_goto_direction: "));
    Bluetooth.println(previous_ra_goto_direction);
       Bluetooth.println(F("ra_goto_direction: "));
    Bluetooth.println(ra_goto_direction);
    previous_ra_bl_fix=current_ra_backlash_fix;;
     
    }
 if(previous_ra_goto_direction == ra_goto_direction){
   Bluetooth.println(F("previous_ra_goto_direction: "));
    Bluetooth.println(previous_ra_goto_direction);
       Bluetooth.println(F("ra_goto_direction: "));
    Bluetooth.println(ra_goto_direction);
     Bluetooth.println(F(" RA DIRECTION MAINTAINED"));
    }
    

}

void define_dec_direction();
/*move the DEC motor*/
void move_DEC_motor(long DEC_steps) {
  digitalWrite(9, LOW); //ENABLE pin of the DEC motor driver
  if (DEC_auto_control != 1) {
    microstep_mode_X4();
    DEC_stepper.setCurrentPosition(DEC_stepper.currentPosition() / 4);
  }
  Bluetooth.println(DEC_stepper.currentPosition());

  tracking_ra_flag = 0;
  tracking_dec_flag = 0;
  DEC_manual_control = 0;
  DEC_auto_control = 1;

  Bluetooth.println(F("Moving DEC to"));
  Bluetooth.println(DEC_steps_to_go);
  alignDEC = 0;
  
  if(!second_move_execution){
goto_dec_bl_fix=0;}
  if(second_move_execution){
goto_dec_bl_fix = previous_dec_bl_fix;}

  /*move clockwise: */
  if (DEC_stepper.currentPosition() < DEC_steps )  {
   // DEC_steps_to_go = DEC_steps_to_go + 0 ;
    DEC_stepper.moveTo(DEC_steps + 0);
    Bluetooth.println(F("DEC moving clockwise!"));
    Bluetooth.println(F("0 DEC fix APPLIED !"));
    
  }

  /*move counter-clockwise: */

  if (DEC_stepper.currentPosition() > DEC_steps ||
  (DEC_stepper.currentPosition() == DEC_steps  && dec_goto_direction==-1  ))
  {
    DEC_steps_to_go = DEC_steps_to_go + goto_dec_bl_fix ;
    DEC_stepper.moveTo(DEC_steps_to_go);
    Bluetooth.println(F("DEC moving counter-clockwise!"));
    Bluetooth.print( goto_dec_bl_fix );
    Bluetooth.println(F(" DEC fix APPLIED !"));
  }
 previous_dec_goto_direction= dec_goto_direction;
  define_dec_direction();
  if(previous_dec_goto_direction != dec_goto_direction){
     Bluetooth.println(F(" DEC DIRECTION CHANGED"));
     previous_dec_bl_fix=received_dec_bl_fix;
    }
 if(previous_dec_goto_direction == dec_goto_direction){
     Bluetooth.println(F(" DEC DIRECTION MAINTAINED"));
    }   

}


void recvWithStartEndMarkers();
void showNewData();
void parseData();
void move_command();
void configuration_command();
void initialize_command();
void micro_move_ra_clockwise();
void micro_move_dec_clockwise();
void micro_move_dec_counterclockwise();
void speed_ra_increase();
void speed_ra_decrease();
void speed_dec_increase();
void stop_and_send_offsets();
void stop_all();

void check_dec_backlash();
void check_ra_backlash();

void cross_move();
void track();

void speed_dec_decrease();
void micro_move_ra_counterclockwise();
void set_energy_mode();
void blocking_dec_move(long steps,float acceleration);
void blocking_ra_move(long steps,float accelaration);







void loop() {

  if (tracking_ra_flag == 0 && tracking_dec_flag == 0 && RA_manual_control == 0  && RA_auto_control == 1 ) {
    RA_stepper.run();
  }
  if (tracking_ra_flag == 0 && tracking_dec_flag == 0 && DEC_manual_control == 0  && DEC_auto_control == 1) {
    DEC_stepper.run();
  }

  if (DEC_stepper.distanceToGo() == 0 && DEC_auto_control == 1) {
    DEC_auto_control = 0;
    DEC_stepper.setCurrentPosition(DEC_stepper.currentPosition() * 4);
  }
  if (RA_stepper.distanceToGo() == 0 && RA_auto_control == 1 ) {
    RA_auto_control = 0;
    RA_stepper.setCurrentPosition(RA_stepper.currentPosition() * 4);


  }

  if (DEC_stepper.distanceToGo() == 0 && RA_stepper.distanceToGo() == 0 && RA_auto_control == 0 && DEC_auto_control == 0 && motors_stopped == false) {
     //Bluetooth.println(DEC_stepper.currentPosition());
   //  Bluetooth.println(RA_stepper.currentPosition());
    if ((RA_stepper.currentPosition() == received_ra_bl_fix*4 || RA_stepper.currentPosition()==-received_ra_bl_fix*4 || RA_stepper.currentPosition()==0)
     && (DEC_stepper.currentPosition() == received_dec_bl_fix*4 || DEC_stepper.currentPosition()==-received_dec_bl_fix*4 || DEC_stepper.currentPosition()==0))
    {
      
     if(RA_steps_to_go == received_ra_bl_fix ){
        blocking_ra_move(-received_ra_bl_fix, RA_stepper.maxSpeed());
        ra_goto_direction=1;
        RA_direction_slow=1;
        
      }
        if(DEC_steps_to_go == received_dec_bl_fix ){
        blocking_dec_move(-received_dec_bl_fix, DEC_stepper.maxSpeed());
        dec_goto_direction=1;
        DEC_direction_slow=1;
       
      }
      
      Bluetooth.println(F("returned_to_initial_position"));
      at_initial_position=true;
      motors_stopped = true;
      same_move_counter=2;
    }
    else {

      
      Bluetooth.println(F("motors_stopped")); motors_stopped = true;
      same_move_counter = same_move_counter + 1 ;

      
    }
     send_motor_directions();
     //Bluetooth.print(F("same_move_counter :"));
   //  Bluetooth.println(same_move_counter);

     if(same_move_counter==2){
/*TODO next 3 lines maybe unecessery, test if so ..*/
 //microstep_mode_X4();
 
    // previous_ra_backlash_fix=ra_backlash_fix;
    // previous_dec_backlash_fix=dec_backlash_fix;
     get_current_backlash_fixes();
      }
     
         
    microstep_mode_X16();
 
     if (energy_saving_is_on) {
      digitalWrite(9, HIGH); //DISABLE the DEC motor driver
      digitalWrite(10, HIGH); //DISABLE the RA motor driver
    }
    if(auto_tracking && !at_initial_position && same_move_counter==2){
      Bluetooth.println(F("auto_tracking_started"));
      Bluetooth.println(F("request_tracking"));}
  }

  if (RA_manual_control == 1  || DEC_manual_control == 1 ) {

    RA_stepper.runSpeed();
    DEC_stepper.runSpeed();
  }

  if (ra_tracking_rate < 0 && (tracking_ra_flag == 1 || tracking_dec_flag == 1) ) {
    RA_stepper.runSpeed();
    DEC_stepper.runSpeed();
    if(RA_steps_to_go < 0 && DEC_steps_to_go > 0 && RA_stepper.currentPosition() < - ra_stop_tracking_limit_up * 4 ){ tracking_reached_limit= true ; stop_all();}
    if(((RA_steps_to_go > 0 || (RA_steps_to_go < 0 && DEC_steps_to_go < 0))) &&( RA_stepper.currentPosition() < ra_horizon_tracking_limit * 4)  ){tracking_reached_limit= true ; stop_all();}
   
  }

  if (ra_tracking_rate > 0 && (tracking_ra_flag == 1 || tracking_dec_flag == 1) ) {
    RA_stepper.runSpeed();
    DEC_stepper.runSpeed();
    if(RA_steps_to_go > 0 && DEC_steps_to_go < 0 && RA_stepper.currentPosition() >  ra_stop_tracking_limit_up * 4 ){ tracking_reached_limit= true ; stop_all();}
    if(((RA_steps_to_go < 0 || (RA_steps_to_go > 0 && DEC_steps_to_go > 0))) &&( RA_stepper.currentPosition() > ra_horizon_tracking_limit * 4)  ){tracking_reached_limit= true ; stop_all();}
   
  }



  recvWithStartEndMarkers();
  showNewData();

}



void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  while (Bluetooth.available() > 0 && newData == false) {
    rc = Bluetooth.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
        Bluetooth.read();
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}

void showNewData() {
  if (newData == true) {
    Bluetooth.print("received :");
    Bluetooth.println(receivedChars);
    newData = false;
    parseData();
  }
}

void parseData() {

  // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(receivedChars, ":");     // get the first part - the command
  strcpy(command, strtokIndx); // copy it to command
  if (strcmp(command, "move") == 0) {
    move_command();
  }
  if (strcmp(command, "config") == 0) {
    configuration_command();
  }
  if (strcmp(command, "initialize") == 0) {
    initialize_command();
  }

  if (strcmp(command, "micro_move_ra+") == 0) {
    micro_move_ra_clockwise();
  }
  if (strcmp(command, "micro_move_ra-") == 0) {
    micro_move_ra_counterclockwise();
  }
  if (strcmp(command, "micro_move_dec+") == 0) {
    micro_move_dec_clockwise();
  }
  if (strcmp(command, "micro_move_dec-") == 0) {
    micro_move_dec_counterclockwise();
  }
  if (strcmp(command, "r+") == 0) {
    speed_ra_increase();
  }
  if (strcmp(command, "r-") == 0) {
    speed_ra_decrease();
  }
  if (strcmp(command, "d+") == 0) {
    speed_dec_increase();
  }
  if (strcmp(command, "d-") == 0) {
    speed_dec_decrease();
  }
  if (strcmp(command, "stop&send_offsets") == 0) {
    stop_and_send_offsets();
  }
  if (strcmp(command, "stop") == 0) {
    char * strtokIndx; // this is used by strtok() as an index
    strtokIndx = strtok(NULL, ":");
    stop_and_restore_dir = atoi(strtokIndx);    // convert this part to a Int
    stop_all();
  }


  if (strcmp(command, "b_lash_dec") == 0) {
    check_dec_backlash();
  }
  if (strcmp(command, "b_lash_ra") == 0) {
    check_ra_backlash();
  }
 
  if (strcmp(command, "move_with_b_lash") == 0) {
    move_command();
    second_move_execution = true;
    Bluetooth.println(F("move_with_backlash_fix.."));
  }

  if (strcmp(command, "cross_move") == 0) {
    cross_move();
  }

  if (strcmp(command, "track") == 0) {
    track();
  }

  if (strcmp(command, "set_energy_saving") == 0) {
    set_energy_mode();
  }
 if (strcmp(command, "request_energy_saving") == 0) {
      Bluetooth.print(F("esm_currently:"));
      Bluetooth.println(energy_saving_is_on);
      
  }
  
  if (strcmp(command, "get_motor_directions") == 0) {
    send_motor_directions();
  }
   if (strcmp(command, "set_auto_tracking") == 0) {
    set_auto_tracking();
  }

    if (strcmp(command, "set_calibration_speed") == 0) {
    set_calibration_motor_speeds();
  }
  if (strcmp(command, "center_object") == 0) {
    center_object();
  }
   if (strcmp(command, "request_dec_position") == 0) {
    Bluetooth.print(F("returned_current_dec_position:"));
    Bluetooth.println(DEC_stepper.currentPosition());
  }

   if (strcmp(command, "update_current_bl_fixes") == 0) {
    update_current_backlash_fixes();
  }

  

}
void configuration_command() {
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ":");
  ra_configured_max_speed = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ":");
  dec_configured_max_speed = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ":");
  sidereal_tracking_speed = atof(strtokIndx);
  strtokIndx = strtok(NULL, ":");
  int RA_motor_is_inverted = atoi(strtokIndx);
  strtokIndx = strtok(NULL, ":");
  int DEC_motor_is_inverted = atoi(strtokIndx);
  if (RA_motor_is_inverted == 1) {
    RA_stepper.setPinsInverted(1, 0, 0);
  }
  if (RA_motor_is_inverted == 0) {
    RA_stepper.setPinsInverted(0, 0, 0);
  }
  if (DEC_motor_is_inverted == 1) {
    DEC_stepper.setPinsInverted(1, 0, 0);
  }
  if (DEC_motor_is_inverted == 0) {
    DEC_stepper.setPinsInverted(0, 0, 0);
  }



  RA_stepper.setMaxSpeed(ra_configured_max_speed);
  RA_stepper.setAcceleration(ra_configured_max_speed * 0.4);
  DEC_stepper.setMaxSpeed(dec_configured_max_speed);
  DEC_stepper.setAcceleration(dec_configured_max_speed * 0.4);
  Bluetooth.println(ra_configured_max_speed);
  Bluetooth.println(dec_configured_max_speed);
  Bluetooth.println(sidereal_tracking_speed);
  Bluetooth.println(F("configuration_received"));


}
void move_command() {


  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(NULL, ":");     // get the command name
  strcpy(command, strtokIndx); // copy it to command

  if (strcmp(command, "RA") == 0) {
    strtokIndx = strtok(NULL, ";"); // this continues where the previous call left off
    RA_steps_to_go = atol(strtokIndx);     // convert this part to a Long
  }

  strtokIndx = strtok(NULL, ":");
  strcpy(command, strtokIndx);

  if (strcmp(command, "DEC") == 0) {
    strtokIndx = strtok(NULL, ":");
    DEC_steps_to_go = atol(strtokIndx);     // convert this part to a Long

  }


  strtokIndx = strtok(NULL, ":");
  second_move_execution = atoi(strtokIndx);
 
  if(!second_move_execution){
same_move_counter=0;
  }

 strtokIndx = strtok(NULL, ":"); // this continues where the previous call left off
 current_ra_backlash_fix = atoi(strtokIndx);     // convert this part to a Long
 strtokIndx = strtok(NULL, ";"); // this continues where the previous call left off
 current_dec_backlash_fix = atoi(strtokIndx);     // convert this part to a Long
 
  
if(at_initial_position){
  Bluetooth.println(F("START POSITION DETECTED !!!"));
  get_current_backlash_fixes();
at_initial_position=false;}

  if (second_move_execution) {
    Bluetooth.print(F("second_move_execution_true:"));
    Bluetooth.println(second_move_execution);
  //  second_move_execution = false;
  }
  else if (!second_move_execution) {
    Bluetooth.print(F("second_move_execution_false:"));
    Bluetooth.println(second_move_execution);
    // received_ra_bl_fix = 0;
     //received_dec_bl_fix = 0;
   // second_move_execution = true;
  }

  move_RA_motor(RA_steps_to_go );
  move_DEC_motor(DEC_steps_to_go );
  motors_stopped = false;
  Bluetooth.println(F("motors_not_stopped"));
}



void initialize_command() {

  if (!motors_initialized) {
    digitalWrite(9, LOW); //ENABLE the DEC motor driver
    digitalWrite(10, LOW); //ENABLE the RA motor driver

  
    char * strtokIndx; // this is used by strtok() as an index
    strtokIndx = strtok(NULL, ":");
    strcpy(command, strtokIndx); // copy it to command
    ra_initialization_steps = atol(strtokIndx);     // convert this part to a Long
    strtokIndx = strtok(NULL, ":");
    strcpy(command, strtokIndx);
    dec_initialization_steps = atol(strtokIndx);     // convert this part to a Long
  /* 90 + 1 degrees ra limit ,since during initialization RA motor moves 5 degrees ,5*18=90 degrees  , 1 degree is added so that the tracking will continue for 4(=60/15) minutes after Zenith is reached  */
    ra_stop_tracking_limit_up = ra_initialization_steps*18+(ra_initialization_steps/5) ; 
     blocking_dec_move(-dec_initialization_steps , DEC_stepper.maxSpeed()*0.4);
     blocking_dec_move(dec_initialization_steps , DEC_stepper.maxSpeed()*0.4);
     blocking_ra_move(-ra_initialization_steps , RA_stepper.maxSpeed()*0.4);
     blocking_ra_move(ra_initialization_steps , RA_stepper.maxSpeed()*0.4);
 
     RA_stepper.setCurrentPosition(0);
     DEC_stepper.setCurrentPosition(0);
      motors_initialized = true;
  }
 
  if (energy_saving_is_on) {
    digitalWrite(9, HIGH); //DISABLE the DEC motor driver
    digitalWrite(10, HIGH); //DISABLE the RA motor driver
  }

  Bluetooth.println(F("initialization_done"));
}


void track() {
  /*track on both axis*/
    char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ":");
  strcpy(command, strtokIndx); // copy it to command
  ra_tracking_rate = atof(strtokIndx);     // convert this part to a Long
  strtokIndx = strtok(NULL, ":");
  strcpy(command, strtokIndx);
  float dec_tracking_rate = atof(strtokIndx);     // convert this part to a Long
  strtokIndx = strtok(NULL, ":");
  ra_horizon_tracking_limit = atol(strtokIndx);
  
  digitalWrite(10, LOW); //ENABLE the RA motor driver
  if(dec_tracking_rate!=0){
  digitalWrite(9, LOW);} //ENABLE the DEC motor driver
  
  if (RA_auto_control == 0 && DEC_auto_control == 0) {

if((ra_tracking_rate<0) && (ra_goto_direction == 1 || RA_direction_slow == 1) ){
      blocking_ra_move(received_ra_bl_fix , RA_stepper.maxSpeed());
      RA_direction_slow = -1;
      
    }

    if((ra_tracking_rate>0) && (ra_goto_direction == -1 || RA_direction_slow ==-1) ){
      blocking_ra_move(-received_ra_bl_fix , RA_stepper.maxSpeed());
      RA_direction_slow = 1;
    }

    if((dec_tracking_rate<0) && (dec_goto_direction == 1 || DEC_direction_slow == 1) ){
      blocking_dec_move(received_dec_bl_fix , DEC_stepper.maxSpeed());
      DEC_direction_slow = -1;
    }

    if((dec_tracking_rate>0) && (dec_goto_direction == -1 || DEC_direction_slow ==-1) ){
      blocking_dec_move(-received_dec_bl_fix , DEC_stepper.maxSpeed());
      DEC_direction_slow = 1;
    }


  //  if(dec_tracking_rate!=0){tracking_dec_flag = 1}
  //tracking_ra_flag = 1;
    
    Bluetooth.println(F("Start Tracking at calculated rates on both axis"));
    Bluetooth.println(F("ra_tracking_rate : "));
    Bluetooth.println(ra_tracking_rate, 6);
    Bluetooth.println(F("dec_tracking_rate : "));
    Bluetooth.println(dec_tracking_rate, 6);
    RA_stepper.setSpeed(ra_tracking_rate);
     tracking_ra_flag = 1;
    if(dec_tracking_rate!=0){
    DEC_stepper.setSpeed(dec_tracking_rate);tracking_dec_flag = 1;}
     
      
  }
}


 void set_calibration_motor_speeds(){
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ":");
  int auto_centering_speed = atoi(strtokIndx);     // convert this part to a int
   
   if (DEC_stepper.isRunning() &&  DEC_manual_control == 1) {
     if(DEC_stepper.speed()>0){
    DEC_stepper.setSpeed( abs(sidereal_tracking_speed) * auto_centering_speed);}
     if(DEC_stepper.speed()<0){
    DEC_stepper.setSpeed( -abs(sidereal_tracking_speed) * auto_centering_speed);}
     Bluetooth.print(F("dec_speed_set_to : "));
     Bluetooth.println(DEC_stepper.speed());
  } 
 
  
  }

void micro_move_ra_clockwise() {
  /*micro_move_ra+*/

  if (RA_auto_control == 0 && DEC_auto_control == 0) {
    digitalWrite(10, LOW); //ENABLE the RA motor driver

    if (ra_goto_direction == -1 || RA_direction_slow == -1) {
      blocking_ra_move(-received_ra_bl_fix , RA_stepper.maxSpeed());
    }
    RA_manual_control = 1;
    tracking_ra_flag = 0;
    tracking_dec_flag = 0;
    RA_direction_slow = 1;
    RA_stepper.setSpeed(2 * sidereal_tracking_speed);
  }
}

void micro_move_ra_counterclockwise() {

  if (RA_auto_control == 0 && DEC_auto_control == 0) {
    digitalWrite(10, LOW); //ENABLE the RA motor driver
    if (ra_goto_direction == 1 || RA_direction_slow == 1) {
      blocking_ra_move(received_ra_bl_fix , RA_stepper.maxSpeed());
    }
    tracking_ra_flag = 0;
    tracking_dec_flag = 0;
    RA_manual_control = 1;
    RA_direction_slow = -1;
    RA_stepper.setSpeed(-2 * sidereal_tracking_speed);
  }
}

//TODO when changing microstepping modes  floor the steps
void micro_move_dec_clockwise() {
  /*micro_move_dec+*/

  if (DEC_auto_control == 0 && RA_auto_control == 0) {
    digitalWrite(9, LOW); //ENABLE the DEC motor driver


    if (dec_goto_direction == -1 || DEC_direction_slow == -1) {
      blocking_dec_move(-received_dec_bl_fix , DEC_stepper.maxSpeed());
    }

    DEC_direction_slow = 1;
    tracking_ra_flag = 0;
    tracking_dec_flag = 0;
    DEC_manual_control = 1;


    DEC_stepper.setSpeed(2 * sidereal_tracking_speed);
  }
}

void micro_move_dec_counterclockwise() {
  /*micro_move_dec-*/

  if (DEC_auto_control == 0 && RA_auto_control == 0) {
    digitalWrite(9, LOW); //ENABLE the DEC motor driver

    if (dec_goto_direction == 1 || DEC_direction_slow == 1) {
      blocking_dec_move(received_dec_bl_fix , DEC_stepper.maxSpeed());
    }


    DEC_direction_slow = -1;
    tracking_ra_flag = 0;
    tracking_dec_flag = 0;
    DEC_manual_control = 1;


    DEC_stepper.setSpeed(-2 * sidereal_tracking_speed);
  }
}

void speed_ra_increase() {
  /*speed_ra++*/
  RA_current_speed = RA_stepper.speed();
  if (RA_stepper.isRunning() &&  RA_manual_control == 1 && abs(RA_current_speed) < RA_stepper.maxSpeed()) {
    RA_stepper.setSpeed(RA_current_speed * 2);
    //Bluetooth.println(RA_stepper.speed());
  }
}

void speed_ra_decrease() {
  /*speed_ra--*/
  RA_current_speed = RA_stepper.speed();
  if (RA_stepper.isRunning() &&  RA_manual_control == 1 && abs(RA_current_speed) > 2*sidereal_tracking_speed) {
    RA_stepper.setSpeed(RA_current_speed / 2);
    //Bluetooth.println(RA_stepper.speed());
  }
}


void speed_dec_increase() {
  /*speed_dec++*/
  DEC_current_speed = DEC_stepper.speed();
  if (DEC_stepper.isRunning() &&  DEC_manual_control == 1 && abs(DEC_current_speed) < DEC_stepper.maxSpeed()) {
    DEC_stepper.setSpeed(DEC_current_speed * 2);
    //Bluetooth.println(DEC_stepper.speed());
  }
}

void speed_dec_decrease() {
  /*speed_dec--*/
  DEC_current_speed = DEC_stepper.speed();
  if (DEC_stepper.isRunning()  && DEC_manual_control == 1 && abs(DEC_current_speed) >  2*sidereal_tracking_speed) {
  DEC_stepper.setSpeed(DEC_current_speed / 2);
    //Bluetooth.println(DEC_stepper.speed());
  }
}

void stop_and_send_offsets()
{

if(RA_auto_control==1 || DEC_auto_control==1){
  return;
}

stop_all();

  /*stop&send_offsets*/
 
  Bluetooth.println(F("current DEC step number is :  "));
  Bluetooth.println(DEC_stepper.currentPosition());
  Bluetooth.println(F("DEC Steps to go number is :  "));
  Bluetooth.println(DEC_steps_to_go);

  Bluetooth.println(DEC_stepper.currentPosition());

  if (DEC_manual_control == 1 )
  {
    alignDEC = (DEC_stepper.currentPosition() - DEC_steps_to_go * 4) / 4;
  }

  if (RA_manual_control == 1 )
  {
    alignRA = (RA_stepper.currentPosition() - RA_steps_to_go * 4) / 4;

  }

  Bluetooth.print("align_offsets:");
  Bluetooth.print(alignRA);
  Bluetooth.print(",");
  Bluetooth.println(alignDEC);
 
}





void stop_all() {
  /*stop*/

  if (RA_auto_control == 1 || DEC_auto_control == 1) {
    
   // RA_stepper.setMaxSpeed(RA_stepper.maxSpeed()/4);
   // DEC_stepper.setMaxSpeed(DEC_stepper.maxSpeed()/4);
  unsigned long  time_now = millis();
   
    RA_stepper.stop();
    
   

     //  while(millis() < time_now + 1000){
        // add the code you want to keep running here
        //wait approx. [period] ms
   // }   
    
     
    
     second_move_execution = true;
      DEC_stepper.stop();
       Bluetooth.println(F("move_canceled")); 
     //RA_stepper.setMaxSpeed(RA_stepper.maxSpeed()*4);
     //DEC_stepper.setMaxSpeed(DEC_stepper.maxSpeed()*4);
    
    send_motor_directions();

  }


  if (RA_manual_control == 1 ) {
    RA_stepper.setSpeed(0);
    //RA_stepper.setCurrentPosition(RA_stepper.currentPosition());

    /* restore the motor's direction to the one of the last goto move : */
    if (ra_goto_direction == 1 && RA_direction_slow == -1) {
      if(stop_and_restore_dir==1){
      blocking_ra_move(-received_ra_bl_fix, RA_stepper.maxSpeed());}
      RA_direction_slow = 1;

    }

    if (ra_goto_direction == -1 && RA_direction_slow == 1 ) {
      if(stop_and_restore_dir==1){
      blocking_ra_move(+received_ra_bl_fix , RA_stepper.maxSpeed() );}
      RA_direction_slow = -1;

    }
   if (energy_saving_is_on) {
     digitalWrite(10, HIGH); //DISABLE the RA motor driver
  } 

  }

  if (DEC_manual_control == 1 ) {
    DEC_stepper.setSpeed(0);
    //DEC_stepper.setCurrentPosition(DEC_stepper.currentPosition());

    /* restore the motor's direction to the one of the last goto move : */
    if (dec_goto_direction == 1 && DEC_direction_slow == -1 ) {
      if(stop_and_restore_dir==1){
      blocking_dec_move(-received_dec_bl_fix , DEC_stepper.maxSpeed());}
      DEC_direction_slow = 1;

    }

    if (dec_goto_direction == -1 && DEC_direction_slow == 1 ) {
      if(stop_and_restore_dir==1){
      blocking_dec_move(+received_dec_bl_fix , DEC_stepper.maxSpeed());}
      DEC_direction_slow = -1;

    }

    if (energy_saving_is_on) {
    digitalWrite(9, HIGH); //DISABLE the DEC motor driver
  }

  }


  if (tracking_ra_flag == 1) {
    RA_stepper.setSpeed(0);
    tracking_ra_flag = 0;
    RA_manual_control = 1;
  }

  if (tracking_dec_flag == 1) {
    DEC_stepper.setSpeed(0);
    tracking_dec_flag = 0;
    DEC_manual_control = 1;
  }

if(tracking_reached_limit){
  Bluetooth.println(F("ra_tracking_limit_reached"));
  tracking_reached_limit= false ;
     Bluetooth.print(F("ra_horizon_tracking_limit : "));
   Bluetooth.println(ra_horizon_tracking_limit);
   
  }  

  
  
}




void check_dec_backlash() {

  digitalWrite(9, LOW); //ENABLE the DEC motor driver
   char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ";"); // this continues where the previous call left off
  DEC_steps_to_go = atol(strtokIndx);     // convert this part to a Long
if(dec_goto_direction==-1){
    DEC_steps_to_go=-DEC_steps_to_go;
  }
  blocking_dec_move(DEC_steps_to_go , DEC_stepper.maxSpeed()*0.4);
  blocking_dec_move(-DEC_steps_to_go , DEC_stepper.maxSpeed()*0.4);

  DEC_auto_control = 0;
  if (energy_saving_is_on) {
    digitalWrite(9, HIGH); //DISABLE the DEC motor driver
  }
}

void check_ra_backlash() {

  digitalWrite(10, LOW); //ENABLE the RA motor driver
   char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ";"); // this continues where the previous call left off
  RA_steps_to_go = atol(strtokIndx);     // convert this part to a Long
  if(ra_goto_direction==-1){
    RA_steps_to_go=-RA_steps_to_go;
  }

  blocking_ra_move(RA_steps_to_go , RA_stepper.maxSpeed()*0.4);
  blocking_ra_move(-RA_steps_to_go , RA_stepper.maxSpeed()*0.4);

  RA_auto_control = 0;
  if (energy_saving_is_on) {
    digitalWrite(10, HIGH); //DISABLE the RA motor driver
  }
}




void cross_move() {

  digitalWrite(9, LOW); //ENABLE the DEC motor driver
  digitalWrite(10, LOW); //ENABLE the RA motor driver


  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ":");

  strcpy(command, strtokIndx); // copy it to command

  long ra_cross_steps = atol(strtokIndx);     // convert this part to a Long
  strtokIndx = strtok(NULL, ":");
  strcpy(command, strtokIndx);
  long dec_cross_steps = atol(strtokIndx);     // convert this part to a Long
  long test_dec_backlash=current_dec_backlash_fix;
  long test_ra_backlash=current_ra_backlash_fix;
  Bluetooth.print(F("dec_cross_steps"));
   Bluetooth.println(dec_cross_steps);
    Bluetooth.print(F("ra_cross_steps"));
   Bluetooth.println(ra_cross_steps);
   
if(dec_goto_direction==-1){
    dec_cross_steps=-dec_cross_steps;
    test_dec_backlash=-test_dec_backlash;
  }
  
  if(ra_goto_direction==-1){
    ra_cross_steps=-ra_cross_steps;
     test_ra_backlash=-test_ra_backlash;
  }
Bluetooth.print(F("dec_cross_steps"));
   Bluetooth.println(dec_cross_steps);
    Bluetooth.print(F("ra_cross_steps"));
   Bluetooth.println(ra_cross_steps);
double dec_to_ra_gear_ratio = dec_initialization_steps/(float)ra_initialization_steps;
 
//match the actual(taking into account the difference in ra and dec gears teeth) moving ra and dec speeds before start moving :
//Also , reduce thet speed to 1/4 of the dec speed ,during the cross move .
DEC_stepper.setMaxSpeed(dec_to_ra_gear_ratio*(dec_configured_max_speed/4));
RA_stepper.setMaxSpeed(dec_configured_max_speed/4);

     blocking_dec_move(+ dec_cross_steps , DEC_stepper.maxSpeed()*0.4);
     Bluetooth.println(F("cross_move_checkpoint:1"));
     blocking_dec_move(- 2 * dec_cross_steps + test_dec_backlash , DEC_stepper.maxSpeed()*0.4);
     Bluetooth.println(F("cross_move_checkpoint:2"));
     blocking_dec_move(dec_cross_steps - test_dec_backlash , DEC_stepper.maxSpeed()*0.4);
     Bluetooth.println(F("cross_move_checkpoint:3"));
     blocking_ra_move(+ ra_cross_steps , RA_stepper.maxSpeed()*0.4);
     Bluetooth.println(F("cross_move_checkpoint:4"));
     blocking_ra_move(-2 * ra_cross_steps + test_ra_backlash , RA_stepper.maxSpeed()*0.4);
     Bluetooth.println(F("cross_move_checkpoint:5"));
     blocking_ra_move(ra_cross_steps - test_ra_backlash , RA_stepper.maxSpeed()*0.4);
      Bluetooth.println(F("cross_move_checkpoint:6"));
 

/*Restore max speed values for both motors aftet the cross move :*/
DEC_stepper.setMaxSpeed(dec_configured_max_speed);
RA_stepper.setMaxSpeed(ra_configured_max_speed);
blocking_dec_move(0, DEC_stepper.maxSpeed()*0.4);
blocking_ra_move(0 , RA_stepper.maxSpeed()*0.4);

Bluetooth.println(F("cross_move_done"));

  if (energy_saving_is_on) {
    digitalWrite(9, HIGH); //DISABLE the DEC motor driver
    digitalWrite(10, HIGH); //DISABLE the RA motor driver
  }

  /*  microstep_mode_X16();
      RA_stepper.setCurrentPosition(RA_stepper.currentPosition() * 4);
      RA_auto_control=0;
    DEC_stepper.setCurrentPosition(DEC_stepper.currentPosition() * 4);
      DEC_auto_control=0;*/
}



void set_energy_mode() {
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ":"); // this continues where the previous call left off
  int received_energy_mode = atoi(strtokIndx);     // convert this part to a Long
  if (received_energy_mode == 1) {
    energy_saving_is_on = true;
    if (RA_auto_control == 0 && RA_manual_control == 0 && tracking_ra_flag == 0 ) {
      digitalWrite(10, HIGH);
    }  //DISABLE the RA motor driver
    if (DEC_auto_control == 0 && DEC_manual_control == 0 && tracking_dec_flag == 0 ) {
      digitalWrite(9, HIGH);
    }   //DISABLE the DEC motor driver

  }
  if (received_energy_mode == 0) {
    energy_saving_is_on = false;
    if (RA_auto_control == 0 && RA_manual_control == 0 && tracking_ra_flag == 0 ) {
      digitalWrite(10, LOW);
    }  //ENABLE the RA motor driver
    if (DEC_auto_control == 0 && DEC_manual_control == 0 && tracking_dec_flag == 0 ) {
      digitalWrite(9, LOW);
    }   //ENABLE the DEC motor driver
  }

}



void define_dec_direction() {
  if (DEC_stepper.distanceToGo() > 0) {
    Bluetooth.println("DEC moving clockwise!!");
    dec_goto_direction = 1;
    DEC_direction_slow = 1;
  }
  if (DEC_stepper.distanceToGo() < 0) {
    Bluetooth.println("DEC moving counter-clockwise!!");
    dec_goto_direction = -1;
    DEC_direction_slow = -1;
  }

}

void define_ra_direction() {
  if (RA_stepper.distanceToGo() > 0) {
   Bluetooth.println("RA moving clockwise!!");
    ra_goto_direction = 1;
    RA_direction_slow = 1;
  }
  if (RA_stepper.distanceToGo() < 0) {
    Bluetooth.println("RA moving counter-clockwise!!");
    ra_goto_direction = -1;
    RA_direction_slow = -1;
  }



}


void blocking_dec_move(long steps,float acceleration) {
  
  microstep_mode_X4();
  DEC_stepper.setAcceleration(acceleration);
  DEC_stepper.setCurrentPosition(DEC_stepper.currentPosition() / 4);
  DEC_stepper.runToNewPosition(DEC_stepper.currentPosition() + steps);
  microstep_mode_X16();
  DEC_stepper.setCurrentPosition(DEC_stepper.currentPosition() * 4);
  DEC_stepper.setAcceleration(DEC_stepper.maxSpeed()* 0.4 );

}

void blocking_ra_move(long steps,float acceleration) {
  microstep_mode_X4();
  RA_stepper.setAcceleration(acceleration);
  RA_stepper.setCurrentPosition(RA_stepper.currentPosition() / 4);
  RA_stepper.runToNewPosition(RA_stepper.currentPosition() + steps);
  microstep_mode_X16();
  RA_stepper.setCurrentPosition(RA_stepper.currentPosition() * 4);
  RA_stepper.setAcceleration(RA_stepper.maxSpeed()* 0.4 );
}

void send_motor_directions(){
   Bluetooth.print(F("goto_ending_directions:"));
   Bluetooth.print(ra_goto_direction);
   Bluetooth.print(F(":"));
   Bluetooth.println(dec_goto_direction);     
  
  }


void  get_current_backlash_fixes(){
  received_ra_bl_fix = current_ra_backlash_fix;
  received_dec_bl_fix = current_dec_backlash_fix;
  
  }


  void set_auto_tracking(){
    char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ":"); // this continues where the previous call left off
 int received_auto_tracking = atoi(strtokIndx);     // convert this part to a integer
  if (received_auto_tracking == 1) {
    auto_tracking = true;
   }
  if (received_auto_tracking == 0) {
    auto_tracking = false;
   }
      
  }


  void center_object(){
     char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ":");     // this continues where the previous call left off
  int received_ra_correction = atoi(strtokIndx);     // convert this part to a integer
   strtokIndx = strtok(NULL, ":");     // this continues where the previous call left off
  int received_dec_correction = atoi(strtokIndx);     // convert this part to a integer
  digitalWrite(9, LOW); //ENABLE the DEC motor driver
  
   
 digitalWrite(10, LOW); //ENABLE the RA motor driver

    
  blocking_ra_move(received_ra_correction, RA_stepper.maxSpeed()*0.4); 
  blocking_dec_move(received_dec_correction, DEC_stepper.maxSpeed()*0.4); 
 
   Bluetooth.print(F("received_ra_correction : "));
   Bluetooth.println(received_ra_correction);
   Bluetooth.print(F("received_dec_correction : "));
   Bluetooth.println(received_dec_correction);


   
   //change direction to clockwise:
   
    if (ra_goto_direction == -1 || RA_direction_slow == -1 && received_ra_correction>0) {
     // blocking_ra_move(-received_ra_bl_fix , RA_stepper.maxSpeed());
     micro_move_ra_clockwise();
     stop_all();
     
    }
    //change direction to counter-clockwise:
     if (ra_goto_direction == 1 || RA_direction_slow == 1 && received_ra_correction<0) {
     // blocking_ra_move(received_ra_bl_fix , RA_stepper.maxSpeed());
      micro_move_ra_counterclockwise();
      stop_all();
    }


//change direction to clockwise:
 if (dec_goto_direction == -1 || DEC_direction_slow == -1 && received_dec_correction>0) {
     // blocking_dec_move(-received_dec_bl_fix , DEC_stepper.maxSpeed());
      micro_move_dec_clockwise();
      stop_all();
    }
    //change direction to counter-clockwise:
    if (dec_goto_direction == 1 || DEC_direction_slow == 1 && received_dec_correction<0) {
     // blocking_dec_move(received_dec_bl_fix , DEC_stepper.maxSpeed());
      micro_move_dec_counterclockwise();
      stop_all();
    }
    
      Bluetooth.println(F("auto_tracking_started"));
      Bluetooth.println(F("request_tracking"));
      Bluetooth.println(F("auto_centering_done"));
  
    }


    
  void  update_current_backlash_fixes(){
     char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(NULL, ":"); // this continues where the previous call left off
 current_ra_backlash_fix = atoi(strtokIndx);     // convert this part to a integer
 strtokIndx = strtok(NULL, ":"); // this continues where the previous call left off
 current_dec_backlash_fix = atoi(strtokIndx);     // convert this part to a integer
   received_ra_bl_fix = current_ra_backlash_fix;
  received_dec_bl_fix = current_dec_backlash_fix;
    Bluetooth.print(F("current_ra_backlash_fix : "));
   Bluetooth.println(current_ra_backlash_fix);
   Bluetooth.print(F(" current_dec_backlash_fix : "));
   Bluetooth.println( current_dec_backlash_fix);
    }




   
