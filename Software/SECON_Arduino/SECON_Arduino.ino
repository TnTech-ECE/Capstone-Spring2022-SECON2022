//WORK ON THE 2ND NET CHECK WHEN TURNING RIGHT


#include <QTRSensors.h>
#include <NewPing.h>
#include <Adafruit_PWMServoDriver.h>
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

bool net_positions[7] = {false};

int net_index = 0;
int net_count = 0;
int serial_data = '0';

int check_nets_counter = 0;
int net_shots[7];
int servo_counter = 3;
int shooting_mech_index = 0;




int timer_isr_counter = 0;

#define SERVOMIN  150 // This is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (out of 4096)
#define USMIN  600 // This is the rounded 'minimum' microsecond length based on the minimum pulse of 150
#define USMAX  2400 // This is the rounded 'maximum' microsecond length based on the maximum pulse of 600
#define SERVO_FREQ 50 // Analog servos run at ~50 Hz updates

#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion
/* Arm dimensions( mm ) */
#define BASE_HGT 67.31      //base hight 2.65"
#define HUMERUS 146.05      //shoulder-to-elbow "bone" 5.75"
#define ULNA 187.325        //elbow-to-wrist "bone" 7.375"
#define GRIPPER 100.00          //gripper (incl.heavy duty wrist rotate mechanism) length 3.94"

/* Servo names/numbers */
/* Base servo HS-485HB */
#define BAS_SERVO 0
/* Shoulder Servo HS-5745-MG */
#define SHL_SERVO 1
/* Elbow Servo HS-5745-MG */
#define ELB_SERVO 2
/* Wrist servo HS-645MG */
#define WRI_SERVO 8
/* Wrist rotate servo HS-485HB */
#define WRO_SERVO 7
/* Gripper servo HS-422 */
#define GRI_SERVO 5

#define TURNTABLE_SERVO 6 //servo that turns the upper chassis

/* Shooting servo*/
#define SHOOT_SERVO 15


#define SHOOT_EN 17 //enable for relay that drives the shooting motor

float hum_sq = HUMERUS * HUMERUS;
float uln_sq = ULNA * ULNA;

long bas_POS, shl_POS, elb_POS, wri_POS, wro_POS, gri_POS = 0;

//-----------------STATES----------------
bool fwd = true;
bool no_nets = true;
bool rev = false;
bool servos_homed = false;

bool turned_right = false;
bool turned_left = false;


bool net_checks_complete[7] = {false};

void read_distance();

bool return_home = false;

bool first_grab = true;
bool second_grab = false;

bool turning_right = false;
bool turning_left = false;
bool return_nets_3_4 = false;

//---------------------------------------


//-----------DEFINES FOR DRIVETRAIN-------
#define EN_FR 7 //front right motor
#define EN_FL 6 //front left motor
#define EN_RR 5 //rear right motor
#define EN_RL 3 //rear left motor

#define FR_REV 52
#define FR_FWD 53
#define FL_REV 51
#define FL_FWD 50

#define RL_REV 46
#define RL_FWD 47
#define RR_FWD 49
#define RR_REV 48

#define START_SWITCH 4

//#define SHOOTING_EN 17

QTRSensors qtr1; //front sensor
QTRSensors qtr2; //middle sensor
QTRSensors qtr3; //rear sensor

int qtr1_calibMax[8] = {2500};
int qtr1_calibMin[8] = {968, 816, 560, 612, 408, 560, 688, 884};

int qtr2_calibMax[8] = {2500};
int qtr2_calibMin[8] = {736, 1324, 1176, 928, 832, 736, 876, 928};

int qtr3_calibMax[8] = {2500};
int qtr3_calibMin[8] = {652, 652, 508, 552, 604, 552, 604, 1008};

uint16_t position1, position2, position3 = 0; //positions for the 3 line followers

int ignore_count = 0;

/*#define dist_fl 0
#define dist_fr 9
#define dist_rr 3
#define dist_rl 11

float distance_fl, distance_fr, distance_rr, distance_rl = 0.0;*/

NewPing front_sensor(A0, A1, 100); 
NewPing back_sensor(A14, A15, 100);

int distance_front = 0;
int distance_back = 0;


//---------------PID Variables------------------------------
int error, last_error = 0;
int correction = 0;
int p, d, i = 0;
float kp = 0.04;
float ki = 0.0;
float kd = 0.30;
int max_speed = 70;
//--------------------------------------------------------

int motor_speed_fr, motor_speed_fl, motor_speed_rr, motor_speed_rl = 0;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


//----------------FUNCTION PROTOTYPES----------------------
//void motor_fwd();
void robot_fwd();
void motor_stop();
void motors_turn_right();
void motors_turn_left();
void robot_straight_slow();
void run_arm_routine();
void shooting_mech_aim();
void shooting_mech_load();
void crab_walk_right();
void crab_walk_left();
void robot_rev();
void read_distance();
void update_sensors();
void check_nets();
void robot_slow_rev();
void crab_walk_right_turn();
void flick_marshmallow();
void robot_drive_strt_after_turn();
void drive_to_nets_3_4();
void drive_to_nets_5_6();
void drive_to_net_2();
void shoot_nets();
void locate_tree_1();
void locate_tree_2();


//-----------------------------------------------------------

void setup() {
  pinMode(START_SWITCH, INPUT);
  digitalWrite(START_SWITCH, HIGH);

  Serial.begin(9600);
  //Serial.print("Startup");
  //shooting_mech_fire();
  pwm.begin();/*
  In theory the internal oscillator (clock) is 25MHz but it really isn't
  that precise. You can 'calibrate' this by tweaking this number until
  you get the PWM update frequency you're expecting!
  The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
  is used for calculating things like writeMicroseconds()
  Analog servos run at ~50 Hz updates, It is importaint to use an
  oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
  1) Attach the oscilloscope to one of the PWM signal pins and ground on
     the I2C PCA9685 chip you are setting the value for.
  2) Adjust setOscillatorFrequency() until the PWM update frequency is the
     expected value (50Hz for most ESCs)
  Setting the value here is specific to each individual I2C PCA9685 chip and
  affects the calculations for the PWM update frequency.
  Failure to correctly set the int.osc value will cause unexpected PWM results
*/
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates
  pwm.setPWM(BAS_SERVO, 0, 410);
  pwm.setPWM(SHL_SERVO, 0, 420);
  pwm.setPWM(ELB_SERVO, 0, 400);
  pwm.setPWM(WRI_SERVO, 0, 100);
  pwm.setPWM(WRO_SERVO, 0, 325);
  pwm.setPWM(GRI_SERVO, 0, 100);
  servos_homed = true;
  //delay(1500);

  pwm.setPWM(TURNTABLE_SERVO, 0, 80);

  //Define motor direction and PWM pins
  pinMode(EN_RR, OUTPUT); //Front Left PWM
  pinMode(EN_RL, OUTPUT); //Front Right PWM
  pinMode(EN_FR, OUTPUT); //Rear Left PWM
  pinMode(EN_FL, OUTPUT); //Rear Right PWM
  pinMode(RR_FWD, OUTPUT);
  pinMode(RR_REV, OUTPUT);
  pinMode(RL_FWD, OUTPUT);
  pinMode(RL_REV, OUTPUT);
  pinMode(FR_FWD, OUTPUT);
  pinMode(FR_REV, OUTPUT);
  pinMode(FL_FWD, OUTPUT);
  pinMode(FL_REV, OUTPUT);
  pinMode(A0, OUTPUT);
  pinMode(A1, INPUT);
  pinMode(A14, OUTPUT);
  pinMode(A15, INPUT);
  pinMode(SHOOT_EN, OUTPUT);
  
//  digitalWrite(SHOOTING_REV, LOW);


  pinMode(SHOOT_EN, OUTPUT);

  //Set up Pololu Sensors with Library Commands
  qtr3.setTypeRC();
  qtr3.setSensorPins((const uint8_t[]) {
    28, 29, 26, 27, 24, 25, 22, 23
  }, SensorCount);//Define which pins are used
  //qtr3.setEmitterPin(34);

  //Set up Pololu Sensors with Library Commands
  qtr2.setTypeRC();
  qtr2.setSensorPins((const uint8_t[]) {
    36, 37, 34, 35, 32, 33, 30, 31
  }, SensorCount);//Define which pins are used
  //qtr2.setEmitterPin(44);

  qtr1.setTypeRC();
  qtr1.setSensorPins((const uint8_t[]) {
    44, 45, 42, 43, 40, 41, 38, 39
  }, SensorCount);//Define which pins are used
  //qtr1.setEmitterPin(23);

  //Calibrate sensors, take 400 readings
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr1.calibrate();
    qtr2.calibrate();
    qtr3.calibrate();
  }
    //AUTOMATICALLY CALIBRATE LINE FOLLOWING SENSORS
  /*for(int i = 0; i < SensorCount; i++) {
    qtr1.calibrationOn.maximum[i] = qtr1_calibMax[i];
    qtr1.calibrationOn.minimum[i] = qtr1_calibMin[i];

    qtr2.calibrationOn.maximum[i] = qtr2_calibMax[i];
    qtr2.calibrationOn.minimum[i] = qtr2_calibMin[i];

    qtr3.calibrationOn.maximum[i] = qtr3_calibMax[i];
    qtr3.calibrationOn.minimum[i] = qtr3_calibMin[i];
  }
  delay(4000);*/

  //Setup TIMER5 overflow interrupts
  noInterrupts();
  TCCR5A = 0;
  TCCR5B = 0;
  TCCR5B |= (1 << CS52) | (1 << CS50);
  OCR5A = 8192;
  TCCR5B |= (1 << WGM52);
  TIMSK5 |= (1 << OCIE5A);
  TCNT5 = 0;
  interrupts();
  //ADMUX = 0b01100000;
  //ADCSRA = 0b10000100;
  

}


void loop() {
  while(digitalRead(START_SWITCH) == 1) {
    net_positions[7] = {false};
    net_index = 0;
    net_count = 0;
    serial_data = '0';

    check_nets_counter = 0;
    net_shots[7];
    servo_counter = 3;
    shooting_mech_index = 0;




    timer_isr_counter = 0;



//-----------------STATES----------------
    fwd = true;
    no_nets = true;
    rev = false;
    servos_homed = false;

    turned_right = false;
    turned_left = false;


    net_checks_complete[7] = {false};

    read_distance();

    return_home = false;

    first_grab = true;
    second_grab = false;

    turning_right = false;
    turning_left = false;
    return_nets_3_4 = false;

    pwm.setPWM(BAS_SERVO, 0, 410);
    pwm.setPWM(SHL_SERVO, 0, 420);
    pwm.setPWM(ELB_SERVO, 0, 400);
    pwm.setPWM(WRI_SERVO, 0, 100);
    pwm.setPWM(WRO_SERVO, 0, 325);
    pwm.setPWM(GRI_SERVO, 0, 100);
    servos_homed = true;
    //delay(1500);

    pwm.setPWM(TURNTABLE_SERVO, 0, 80);

}

  //Serial.println("Start");
update_sensors();

/*update_sensors();
if(turned_right) {
  if(distance_back >= 40) {
    if(2500 < position1 < 5000) {
      if(position2 == 0 || position2 == 7000) {
        ignore_sensor_fwd = false;
      }
    }
  }
}*/


   check_nets(); //Check for net 0
   //Serial.println("Check net 0 done");
   
   net_index++; //ignore net 1

   //Serial.print("Check nets 0 complete");

   locate_tree_1();


   drive_to_net_2(); //contains right turn, line up at net 2
   check_nets(); //check for net 2

  //drive_to_nets_3_4();
  //check_nets(); //check for nets 3
  //check_nets(); //check for net 4
  //flick_marshmallow();
  net_index++;
  net_index++;
  
  flick_marshmallow(); //drive to gap, flick marshmallow off

  locate_tree_2();

  drive_to_nets_5_6();
  check_nets(); //check for net 5
  //check_nets();// check for net 6
  net_index++;
  
  int arm_move_var_1 = 420;
  int arm_move_var_2 = 400;
  
   //Arm move into position for shot
   for (arm_move_var_1 = 420; arm_move_var_1 != 300; arm_move_var_1 = arm_move_var_1 - 5)

  {

    pwm.setPWM(SHL_SERVO, 0, arm_move_var_1);

    delay(10);

    if (arm_move_var_2 != 325)

    {
      pwm.setPWM(ELB_SERVO, 0, arm_move_var_2);

      ;

      delay(10);

      arm_move_var_2 = arm_move_var_2 - 5;

    }

  }
  
  shoot_nets();
          
return_home = true;
while(digitalRead(START_SWITCH == 0)) {
  if(return_home) {
    if(!turned_left) {
      robot_rev();
      update_sensors();
      if(distance_back <= 10) {
        motor_stop();

        turning_left = true;
        motors_turn_left();
        turned_left = true;
        motor_stop();
      }
    }
    if(turned_left) {
      //robot_slow_rev();
      rev = true;
      fwd = false;
      update_sensors();
      while(distance_back >= 8) {
        update_sensors();
        robot_rev();
      }
      motor_stop();
      update_sensors();
          digitalWrite(SHOOT_EN, HIGH);
          delay(1000);
          shooting_mech_index = 3;
          shooting_mech_load();
          delay(3000);
          digitalWrite(SHOOT_EN, LOW);
          
          return_home = false;
    
      }
   }
 }
 delay(500);
}
  

ISR(TIMER5_COMPA_vect) {

  Serial.println("isr");
  timer_isr_counter++;

  /*if(turned_right) {
    ignore_count++;
  }
  
  check_nets_counter++;

  if(turning_right) {
    turn_right_counter++;
  }
  if(turning_left) {
    turn_left_counter++;
  }

  if(turned_right) {

      turn_right_drive_straight_counter++;
    
  }

  if(return_nets_3_4) {
    return_nets_3_4_counter++;
  }*/
}

void drive_to_net_2() {
  update_sensors();
     fwd = true;
     rev = false;
     max_speed = 80;
  while((distance_front >= 12) && !turned_right) {
        update_sensors();
        //Serial.print(distance_front);
        //Serial.println("      ");
     
        robot_fwd();
     }

   robot_straight_slow();



       motor_stop();
       crab_walk_right_turn();
       motor_stop();
       /*fwd = true;
       rev = false;
       crab_walk_right_turn();
       turning_right = true;*/
       turning_right = true;
       motors_turn_right();
       //fwd = true;
       //rev = false;
       //delay(1000);
       turned_right = true;
     

   if(turned_right) {

    max_speed = 70;

    robot_drive_strt_after_turn();
    delay(500);
    


}
}

void locate_tree_1() {
  update_sensors();
  fwd = true;
  rev = false;
  first_grab = true;
  update_sensors();
    while(distance_front >= 28) {
      //Serial.println(distance_front);
      update_sensors();
      robot_fwd();
    }
      motor_stop();
      delay(500);
      update_sensors(); // redundancy
      max_speed = 65;
      if(distance_front > 22) {
        while(distance_front >= 22) {
        //Serial.println(distance_front);
        update_sensors();
        robot_fwd();
        }
      }

      motor_stop();
        
      run_arm_routine();
      first_grab = false;
      second_grab = true;
      
}

void drive_to_nets_3_4() {
  update_sensors();
  fwd = true;
  rev = false;
  while(distance_back <= 55) {
    update_sensors();
    robot_fwd();
  }
  
  motor_stop();
  delay(500);
  
  update_sensors(); // redundancy

  if(distance_back < 60) {
    while(distance_back <= 60) {
    update_sensors();
    robot_fwd();
    }
  }

    motor_stop();
  
}

void locate_tree_2() {
  update_sensors();
  fwd = true;
  rev = false;
    while(distance_front >= 50 || distance_back <= 65) {
     update_sensors();
     robot_fwd();
    }

    motor_stop();
    update_sensors(); // redundancy

    if(distance_front > 50 || distance_back < 65) {
      while(distance_front >= 50 || distance_back <= 65) {
      update_sensors();
      robot_fwd();
      }
    }

      motor_stop();
      run_arm_routine();
    
}

void drive_to_nets_5_6() {
  update_sensors();
  fwd = true;
  rev = false;
  while(distance_front >= 29) {
    update_sensors();
    robot_fwd();
  }
  motor_stop();
  delay(500);

  if(distance_front > 15) {
    update_sensors();
    while(distance_front >= 15) {
      update_sensors();
      robot_fwd();
    }
  }

  motor_stop();
  
}

void flick_marshmallow() {
  update_sensors();
  fwd = true;
  rev = false;
  while(distance_back <= 74) {
    update_sensors();
    robot_fwd();
  }
  
  motor_stop();

  if(distance_back < 74) {
    while(distance_back <= 74) {
      update_sensors();
      robot_fwd();
    }
  }


    motor_stop;
  
  
 //******************************************* Start of Sweep ***********************************
//Serial.println("Move the turntable");
  for(int i = 80; i <= 260; i+=5)
{
 // Serial.print("moving turntable");
  pwm.setPWM(TURNTABLE_SERVO, 0, i);
  //Serial.print("moved turntable");
  delay(70);
  //Serial.print("delayed");
}
//Serial.println("Tried to move turntable");
    //home
  pwm.setPWM(BAS_SERVO, 0, 405);

  pwm.setPWM(SHL_SERVO, 0, 400);

  pwm.setPWM(ELB_SERVO, 0, 390);

  pwm.setPWM(WRI_SERVO, 0, 110);

  pwm.setPWM(WRO_SERVO, 0, 310);

  pwm.setPWM(GRI_SERVO, 0, 100);

  delay(100);



  //pwm.setPWM(TURNTABLE_SERVO, 0, 200);



  delay(100);



int a = 405;

 int b = 400;

 int c = 390;

int  d = 110;

 int e = 310;

 int f = 100;

 int delay_time = 20;
 int delay_time_movement = 100;

   for (b = 400; b != 210; b = b - 5)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

    if (c != 340)

    {
      pwm.setPWM(ELB_SERVO, 0, c);

      Serial.println(b);

      delay(delay_time);

      c = c - 5;

    }
    if(d != 180)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    delay(delay_time);

    d = d + 5;

  }
  pwm.setPWM(GRI_SERVO, 0, 300);
  
  }
      for (e = 310; e != 450; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

  //creep down
     for (b = 210; b != 207; b = b - 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);
  
  }
  

  for(int i = 260; i >= 150; i-=5)
{
  pwm.setPWM(TURNTABLE_SERVO, 0, i);
  delay(70);
}

 for(e = 450; e != 310; e = e - 5)
 {
    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
 }

  pwm.setPWM(WRI_SERVO, 0, 280);

  pwm.setPWM(ELB_SERVO, 0, 290);


  delay(500);
  
  //home sequence

  //creep and grip
  for (b = 207; b != 210; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);
  
  }
  pwm.setPWM(GRI_SERVO, 0, 100);

  for (b = 210; b != 400; b = b + 5)
  {
    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

  }

    for(c = 290; c != 390; c = c + 5)
    {
      pwm.setPWM(ELB_SERVO, 0, c);

      Serial.println(b);

      delay(delay_time);

    if(d != 110)
    {

        pwm.setPWM(WRI_SERVO, 0, d);

        delay(delay_time);

        d = d - 5;      
    }

    }

 pwm.setPWM(WRI_SERVO, 0, 180);
 

  delay(100);  

    for(int i = 150; i >= 80; i-=5)
{
  pwm.setPWM(TURNTABLE_SERVO, 0, i);
  delay(70);
}

//**************************************
// second sweep
//**************************************

 for(int i = 80; i <= 260; i+=5)
{
 // Serial.print("moving turntable");
  pwm.setPWM(TURNTABLE_SERVO, 0, i);
  //Serial.print("moved turntable");
  delay(70);
  //Serial.print("delayed");
}
//Serial.println("Tried to move turntable");
    //home
  pwm.setPWM(BAS_SERVO, 0, 405);

  pwm.setPWM(SHL_SERVO, 0, 400);

  pwm.setPWM(ELB_SERVO, 0, 390);

  pwm.setPWM(WRI_SERVO, 0, 110);

  pwm.setPWM(WRO_SERVO, 0, 310);

  pwm.setPWM(GRI_SERVO, 0, 100);

  delay(100);


   for (b = 400; b != 210; b = b - 5)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

    if (c != 340)

    {
      pwm.setPWM(ELB_SERVO, 0, c);

      Serial.println(b);

      delay(delay_time);

      c = c - 5;

    }
    if(d != 180)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    delay(delay_time);

    d = d + 5;

  }
  pwm.setPWM(GRI_SERVO, 0, 300);
  
  }
      for (e = 310; e != 450; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

  //creep down
     for (b = 210; b != 207; b = b - 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);
  
  }
  

  for(int i = 260; i >= 150; i-=5)
{
  pwm.setPWM(TURNTABLE_SERVO, 0, i);
  delay(70);
}

 for(e = 450; e != 310; e = e - 5)
 {
    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
 }

  pwm.setPWM(WRI_SERVO, 0, 280);

  pwm.setPWM(ELB_SERVO, 0, 290);


  delay(500);
  
  //home sequence

  //creep and grip
  for (b = 207; b != 210; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);
  
  }
  pwm.setPWM(GRI_SERVO, 0, 100);

  for (b = 210; b != 400; b = b + 5)
  {
    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

  }

    for(c = 290; c != 390; c = c + 5)
    {
      pwm.setPWM(ELB_SERVO, 0, c);

      Serial.println(b);

      delay(delay_time);

    if(d != 110)
    {

        pwm.setPWM(WRI_SERVO, 0, d);

        delay(delay_time);

        d = d - 5;      
    }

    }

 pwm.setPWM(WRI_SERVO, 0, 180);
 

  delay(100);  

    for(int i = 150; i >= 80; i-=5)
{
  pwm.setPWM(TURNTABLE_SERVO, 0, i);
  delay(70);
}

  //****************************************************************************************************************************************
  //----------------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------- END MARSH CODE -----------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------------------------------------------
  //****************************************************************************************************************************************
  
}

void shoot_nets() {
  do {

      no_nets = true;
      net_count = 0;
      for(int i = 0; i < 7; i++) {
        if(net_positions[i]) {
          no_nets = false;
          net_count++;
        }
      }
      
    
      fwd = false;
      rev = true;
      if(net_positions[5]) {
          motor_stop();
          crab_walk_left();
          motor_stop();
          digitalWrite(SHOOT_EN, HIGH);
          delay(250);
          shooting_mech_index = net_shots[5];
          shooting_mech_load();
          
          //delay(1000);
          digitalWrite(SHOOT_EN, LOW);
          net_positions[5] = false;
          return_from_left();
          motor_stop();
          
      }

       if(net_positions[6]) {
        update_sensors();
        while(distance_front <= 22) {
          update_sensors();
          robot_rev();
        }
        
        motor_stop();

        if(distance_front < 22) {
          while(distance_front <= 22) {
            update_sensors();
            robot_rev();
          }
        }

        motor_stop();
        
        
       for(int i = 80; i <= 470; i+= 5) {
          pwm.setPWM(TURNTABLE_SERVO, 0, i);
          delay(70);
        }

        crab_walk_right();
        motor_stop();
        digitalWrite(SHOOT_EN, HIGH);
        delay(250);
        shooting_mech_index = net_shots[6];
        shooting_mech_load();
        
        //delay(1000);
        digitalWrite(SHOOT_EN, LOW);

        for(int i = 470; i >= 80; i-= 5) {
          pwm.setPWM(TURNTABLE_SERVO, 0, i);
          delay(70);
        }

        net_positions[6] = false;
        return_from_right();
        motor_stop();
      
       }

       //max_speed = 100;
       if(net_positions[3]) {
        return_nets_3_4 = true;
        update_sensors();
        TCNT5 = 0;
        timer_isr_counter = 0;
        while(distance_front <= 50) {
          update_sensors();
          robot_rev();
        }
        
        motor_stop();
        delay(500);
        update_sensors();
        
        while(distance_back >= 70) {
          update_sensors();
          robot_rev();
        }

        motor_stop();
        update_sensors();
        if(distance_back > 70) {
           while(distance_back >= 70) {
            update_sensors();
            robot_rev();
          }
        }

        
        motor_stop();
        crab_walk_left();
        digitalWrite(SHOOT_EN, HIGH);
        delay(250);
        shooting_mech_index = net_shots[3];
        shooting_mech_load();
        
        //delay(1000);
        digitalWrite(SHOOT_EN, LOW);

        net_positions[3] = false;
        return_from_left();
        motor_stop();
        
      } 

      
      if(net_positions[4]) {
        update_sensors();
        while(distance_back >= 58) {
          update_sensors();
          robot_rev();
        }

        motor_stop();

        update_sensors();

        if(distance_back >= 58) {
           while(distance_back >= 58) {
            update_sensors();
            robot_rev();
          }
        }

        motor_stop();
        

       for(int i = 80; i <= 460; i+= 5) {
          pwm.setPWM(TURNTABLE_SERVO, 0, i);
          delay(70);
        }

        crab_walk_right();
        digitalWrite(SHOOT_EN, HIGH);
        delay(250);
        shooting_mech_index = net_shots[4];
        shooting_mech_load();
        
        //delay(1000);
        digitalWrite(SHOOT_EN, LOW);

        for(int i = 460; i >= 80; i-= 5) {
          pwm.setPWM(TURNTABLE_SERVO, 0, i);
          delay(70);
        }

        return_from_right();
        motor_stop();
        net_positions[4] = false;
       
        
      }

   
    
      if(net_positions[2]) {
        update_sensors();
        while(distance_back >= 22) {
          update_sensors();
          robot_rev();
        }
        
        motor_stop();

        if(distance_back >= 19) {
          while(distance_back >= 19) {
            update_sensors();
            robot_rev();
          }
        }

        motor_stop();
        crab_walk_left();
        motor_stop();
        digitalWrite(SHOOT_EN, HIGH);
        delay(250);
        shooting_mech_index = net_shots[2];
        shooting_mech_load();
        
        //delay(1000);
        digitalWrite(SHOOT_EN, LOW);
        return_from_left();
        motor_stop();
        net_positions[2] = false;
        
        
      }
      if(net_positions[1]) {
        
          /*motors_turn_left();
          robot_fwd();
          if(distance_front <= 4) {
          motor_stop();
          delay(1000);
          digitalWrite(SHOOT_EN, HIGH);
          delay(1000);
          shooting_mech_index = net_shots[1];
          //shooting_mech_load();
          delay(1000);
          fwd = false;
          rev = true;
          shoot_nets = false;
          return_home = true;
          */
          net_positions[1] = false;
   
          //}
        
      }
      if(net_positions[0]) {
        net_positions[0] = false;
      }
  } while(!no_nets);
}

void robot_drive_strt_after_turn() {
  update_sensors();
  fwd = true;
  rev = false;
  TCNT5 = 0;
  timer_isr_counter = 0;
  while(timer_isr_counter < 5) {
    Serial.print(timer_isr_counter);
    update_sensors();
    robot_fwd();
  }

  motor_stop();

  rev = true;
  fwd = false;
  update_sensors();
  while(distance_back >= 25) {
    update_sensors();
    robot_rev();
  }

  motor_stop();
}

void update_sensors() {
  position1 = qtr1.readLineWhite(sensorValues); //Front
  //Serial.println(position1);
  position2 = qtr2.readLineWhite(sensorValues); //Middle
  //Serial.println(position1);
  position3 = qtr3.readLineWhite(sensorValues); //Back
  //Serial.println(position3);

  if (fwd) {
    error = 3500 - position1;
  }
  
  if(rev) {
    error = 3500 - position3;
  }
  
  p = error;
  i = i + error;
  d = error - last_error;
  last_error = error;

  correction = kp * p + ki * i + kd * d; //PID control equation

  if (fwd) {
  motor_speed_fr = (max_speed - correction); //THESE MAY NEED TO BE CHANGED DEPENDING ON HOW SENSOR IS ORIENTED
  motor_speed_rr = (max_speed - correction);
  
  motor_speed_fl = (max_speed + correction);
  motor_speed_rl = (max_speed + correction);
  }
  
  if(rev) {
  motor_speed_fr = (max_speed + correction); //THESE MAY NEED TO BE CHANGED DEPENDING ON HOW SENSOR IS ORIENTED
  motor_speed_rr = (max_speed + correction);
  
  motor_speed_fl = (max_speed - correction);
  motor_speed_rl = (max_speed - correction);
  }
  
  if (motor_speed_fr > max_speed) {
  motor_speed_fr = max_speed;
  }
  
  if (motor_speed_fl > max_speed) {
  motor_speed_fl = max_speed;
  }
  
  if (motor_speed_rr > max_speed) {
  motor_speed_rr = max_speed;
  }
  
  if (motor_speed_rl > max_speed) {
  motor_speed_rl = max_speed;
  }
  
  if (motor_speed_fr < 0) {
  motor_speed_fr = 0;
  }
  
  if (motor_speed_rr < 0) {
  motor_speed_rr = 0;
  }
  
  if (motor_speed_fl < 0) {
  motor_speed_fl = 0;
  }
  
  if (motor_speed_rl < 0) {
  motor_speed_rl = 0;
  }

  read_distance();
}

void read_distance() {
  int distance_temp = 0;
  distance_temp = front_sensor.ping_cm(100);
  if(distance_temp != 0) {
  distance_front = distance_temp;
  }

  distance_temp = 0;
  distance_temp = back_sensor.ping_cm(100);
  if(distance_temp != 0) {
  distance_back = distance_temp;
  }

  /*Serial.print(distance_back);
  Serial.print("      ");
  Serial.print(distance_front);
  Serial.println();*/
  
}

void check_nets() {
  Serial.println("Enter check nets");
  TCNT5 = 0;
  timer_isr_counter = 0;

  while(timer_isr_counter < 8) {
  //Serial.print("Check nets loop");
  Serial1.begin(9600);
  delay(50);
  //Raspberry PI communication
  if (Serial1.available() > 0) {
    serial_data = Serial1.read();
    Serial.print(net_index);
    Serial.print("      ");
    Serial.print(serial_data);
    Serial.println();
    delay(80);
    if(net_index == 0) {
      //Serial.println("Check nets 0");
      if((serial_data == 'X' || serial_data == 'W') && net_positions[net_index] == false) {
        net_positions[net_index] = true;
        net_shots[0] = servo_counter;

        
        servo_counter--;
      }
    }
    else if(net_index == 1) {
      if((serial_data == 'X' || serial_data == 'W') && net_positions[net_index] == false){
        net_positions[net_index] = true;
        net_count++;
        read_distance();
        net_shots[1] = servo_counter;
        servo_counter--;
      }
    }
    else if(net_index == 2) {
      if((serial_data == 'X' || serial_data == 'W') && net_positions[net_index] == false){
        net_positions[net_index] = true;
        net_count++;
        read_distance();
         net_shots[2] = servo_counter;
         servo_counter--;
      }
    }
    else if(net_index == 3) {
       if((serial_data == 'X' || serial_data == 'W') && net_positions[net_index] == false){
        net_positions[net_index] = true;
        net_count++;
        read_distance();
        net_shots[3] = servo_counter;
        servo_counter--;
      }
    }
     else if(net_index == 4) {
       if((serial_data == 'Z' || serial_data == 'W') && net_positions[net_index] == false){
        net_positions[net_index] = true;
        net_count++;
        read_distance();
        net_shots[4] = servo_counter;
        servo_counter--;
      }
    }
    else if(net_index == 5) {
      if((serial_data == 'X' || serial_data == 'W') && net_positions[net_index] == false) {
        net_positions[net_index] = true;
        net_count++;
        read_distance();
         //net_3_dist = distance_front;
         net_shots[5] = servo_counter;
         servo_counter--;
      }
    }
    else if(net_index == 6) {
      if((serial_data == 'Z' || serial_data == 'W') && net_positions[net_index] == false) {
        net_positions[net_index] = true;
        net_count++;
        read_distance();
         net_shots[6] = servo_counter;
         servo_counter--;
      }
    }
  }
  Serial1.end();
  }

  Serial.println("Exit Check nets");
  net_index++;
}

void return_from_left() {
  update_sensors();
   while (position3 < 3500) {
    update_sensors();
    digitalWrite(FR_FWD, LOW);
    digitalWrite(FR_REV, HIGH);

    digitalWrite(FL_FWD, HIGH);
    digitalWrite(FL_REV, LOW);

    digitalWrite(RL_FWD, LOW);
    digitalWrite(RL_REV, HIGH);

    digitalWrite(RR_FWD, HIGH);
    digitalWrite(RR_REV, LOW);

    analogWrite(EN_FL, 110);
    analogWrite(EN_FR, 110);
    analogWrite(EN_RL, 110);
    analogWrite(EN_RR, 110);
  }
  motor_stop();
}

void return_from_right() {
  update_sensors();
   while (position3 > 3500) {
    update_sensors();
     digitalWrite(FR_FWD, HIGH);
    digitalWrite(FR_REV, LOW);

    digitalWrite(FL_FWD, LOW);
    digitalWrite(FL_REV, HIGH);

    digitalWrite(RL_FWD, HIGH);
    digitalWrite(RL_REV, LOW);

    digitalWrite(RR_FWD, LOW);
    digitalWrite(RR_REV, HIGH);

    analogWrite(EN_FL, 110);
    analogWrite(EN_FR, 110);
    analogWrite(EN_RL, 110);
    analogWrite(EN_RR, 110);
  }
  motor_stop();
}

void crab_walk_right_turn() {
  update_sensors();
  while (position3 > 500) {
    update_sensors();
    digitalWrite(FR_FWD, LOW);
    digitalWrite(FR_REV, HIGH);

    digitalWrite(FL_FWD, HIGH);
    digitalWrite(FL_REV, LOW);

    digitalWrite(RL_FWD, LOW);
    digitalWrite(RL_REV, HIGH);

    digitalWrite(RR_FWD, HIGH);
    digitalWrite(RR_REV, LOW);

    analogWrite(EN_FL, 110);
    analogWrite(EN_FR, 100);
    analogWrite(EN_RL, 100);
    analogWrite(EN_RR, 100);
  }
  motor_stop();
}

void crab_walk_left() {
  update_sensors();
  while (position1 > 1000) {
      update_sensors();
    digitalWrite(FR_FWD, HIGH);
    digitalWrite(FR_REV, LOW);

    digitalWrite(FL_FWD, LOW);
    digitalWrite(FL_REV, HIGH);

    digitalWrite(RL_FWD, HIGH);
    digitalWrite(RL_REV, LOW);

    digitalWrite(RR_FWD, LOW);
    digitalWrite(RR_REV, HIGH);

    analogWrite(EN_FL, 100);
    analogWrite(EN_FR, 100);
    analogWrite(EN_RL, 100);
    analogWrite(EN_RR, 100);
  }
  motor_stop();
}

void crab_walk_right() {
  update_sensors();
  while (position1 < 6000) {
    update_sensors();
    digitalWrite(FR_FWD, LOW);
    digitalWrite(FR_REV, HIGH);

    digitalWrite(FL_FWD, HIGH);
    digitalWrite(FL_REV, LOW);

    digitalWrite(RL_FWD, LOW);
    digitalWrite(RL_REV, HIGH);

    digitalWrite(RR_FWD, HIGH);
    digitalWrite(RR_REV, LOW);

    analogWrite(EN_FL, 100);
    analogWrite(EN_FR, 100);
    analogWrite(EN_RL, 100);
    analogWrite(EN_RR, 100);
  }
  motor_stop();
}

void motors_turn_right() {
  update_sensors();
  TCNT5 = 0;
    timer_isr_counter = 0;
  while ((position1 < 3000 && timer_isr_counter < 8) || timer_isr_counter == 0) {
    Serial.println(timer_isr_counter);
    update_sensors();
    digitalWrite(FR_FWD, LOW);
    digitalWrite(FR_REV, HIGH);

    digitalWrite(FL_FWD, HIGH);
    digitalWrite(FL_REV, LOW);

    digitalWrite(RL_FWD, HIGH);
    digitalWrite(RL_REV, LOW);

    digitalWrite(RR_FWD, LOW);
    digitalWrite(RR_REV, HIGH);

    analogWrite(EN_FL, 80);
    analogWrite(EN_FR, 80);
    analogWrite(EN_RL, 80);
    analogWrite(EN_RR, 80);
  }

  motor_stop();

}

void motors_turn_left() {
  update_sensors();
  while (position3 > 3700) {
    update_sensors();
    digitalWrite(FR_FWD, HIGH);
    digitalWrite(FR_REV, LOW);

    digitalWrite(FL_FWD, LOW);
    digitalWrite(FL_REV, HIGH);

    digitalWrite(RL_FWD, LOW);
    digitalWrite(RL_REV, HIGH);

    digitalWrite(RR_FWD, HIGH);
    digitalWrite(RR_REV, LOW);

    analogWrite(EN_FL, 80);
    analogWrite(EN_FR, 80);
    analogWrite(EN_RL, 80);
    analogWrite(EN_RR, 80);
  }
  motor_stop();

}



void robot_rev () {

  digitalWrite(FR_FWD, LOW);
  digitalWrite(FR_REV, HIGH);

  digitalWrite(FL_FWD, LOW);
  digitalWrite(FL_REV, HIGH);

  digitalWrite(RL_FWD, LOW);
  digitalWrite(RL_REV, HIGH);

  digitalWrite(RR_FWD, LOW);
  digitalWrite(RR_REV, HIGH);

  analogWrite(EN_FL, motor_speed_fl);
  analogWrite(EN_FR, motor_speed_fr);
  analogWrite(EN_RL, motor_speed_rl);
  analogWrite(EN_RR, motor_speed_rr);

}

void robot_fwd() {

  digitalWrite(FR_FWD, HIGH);
  digitalWrite(FR_REV, LOW);

  digitalWrite(FL_FWD, HIGH);
  digitalWrite(FL_REV, LOW);

  digitalWrite(RL_FWD, HIGH);
  digitalWrite(RL_REV, LOW);

  digitalWrite(RR_FWD, HIGH);
  digitalWrite(RR_REV, LOW);

  analogWrite(EN_FR, motor_speed_fr);
  analogWrite(EN_FL, motor_speed_fl);
  analogWrite(EN_RR, motor_speed_rr);
  analogWrite(EN_RL, motor_speed_rl);

}

void motor_stop() {

  digitalWrite(FR_FWD, LOW);
  digitalWrite(FR_REV, LOW);

  digitalWrite(FL_FWD, LOW);
  digitalWrite(FL_REV, LOW);

  digitalWrite(RL_FWD, LOW);
  digitalWrite(RL_REV, LOW);

  digitalWrite(RR_FWD, LOW);
  digitalWrite(RR_REV, LOW);

  delay(500);

}

void robot_straight_slow() {
  update_sensors();
  while(distance_front >= 12) {
    update_sensors();
  digitalWrite(FR_FWD, HIGH);
  digitalWrite(FR_REV, LOW);

  digitalWrite(FL_FWD, HIGH);
  digitalWrite(FL_REV, LOW);

  digitalWrite(RL_FWD, HIGH);
  digitalWrite(RL_REV, LOW);

  digitalWrite(RR_FWD, HIGH);
  digitalWrite(RR_REV, LOW);

  analogWrite(EN_FR, 60);
  analogWrite(EN_FL, 60);
  analogWrite(EN_RR, 60);
  analogWrite(EN_RL, 60);
  }
  motor_stop();
}

void robot_slow_rev() {
   update_sensors();
  while(distance_back >= 10) {
    update_sensors();
  digitalWrite(FR_FWD, LOW);
  digitalWrite(FR_REV, HIGH);

  digitalWrite(FL_FWD, LOW);
  digitalWrite(FL_REV, HIGH);

  digitalWrite(RL_FWD, LOW);
  digitalWrite(RL_REV, HIGH);

  digitalWrite(RR_FWD, LOW);
  digitalWrite(RR_REV, HIGH);

  analogWrite(EN_FR, 65);
  analogWrite(EN_FL, 60);
  analogWrite(EN_RR, 65);
  analogWrite(EN_RL, 60);
  }
  motor_stop();
}

void shooting_mech_load() {
  int a = 410;
  int b = 420;
  int c = 400;
  int d = 100;
  int e = 325;
  int f = 100;
  int delay_time = 20;
  int delay_time_movement = 100;
  if(shooting_mech_index == 1) {
  //********************************************************first shoot*****************************************

 
  
//----------------------------------------------------------------
// 
//  This section is for opening and closing the shooting mech
//  pwm 250 moves servo mag out while pwm 500 moves servo mag in
//
//---------------------------------------------------------------- 

  pwm.setPWM(SHOOT_SERVO, 0, 250);
 
  delay(350);
  
  pwm.setPWM(SHOOT_SERVO, 0, 0);

  delay(1000);
  
  pwm.setPWM(SHOOT_SERVO, 0, 500);

  delay(350);
  
  pwm.setPWM(SHOOT_SERVO, 0, 0);

 
  }

  
//***************************************************************************second shot*****************************************
  if(shooting_mech_index == 2) {
  
  //Arm move into position for shot

  
  
//----------------------------------------------------------------
// 
//  This section is for opening and closing the shooting mech
//  pwm 250 moves servo mag out while pwm 500 moves servo mag in
//
//---------------------------------------------------------------- 


  pwm.setPWM(SHOOT_SERVO, 0, 250);
 
  delay(700);
  
  pwm.setPWM(SHOOT_SERVO, 0, 0);

  delay(1000);
  
  pwm.setPWM(SHOOT_SERVO, 0, 500);

  delay(700);

  pwm.setPWM(SHOOT_SERVO, 0, 0);

 
  }

  //***************************************************************************third shot*****************************************
  if(shooting_mech_index == 3) {
  
  //Arm move into position for shot

  
//----------------------------------------------------------------
// 
//  This section is for opening and closing the shooting mech
//  pwm 250 moves servo mag out while pwm 500 moves servo mag in
//
//---------------------------------------------------------------- 

  pwm.setPWM(SHOOT_SERVO, 0, 250);
 
  delay(1100);
  
  pwm.setPWM(SHOOT_SERVO, 0, 0);

  delay(1000);
  
  pwm.setPWM(SHOOT_SERVO, 0, 500);

  delay(1100);

  pwm.setPWM(SHOOT_SERVO, 0, 0);


  
   
  }
}


void run_arm_routine() {
  Serial.print("Arm");
  /*// put your main code here, to run repeatedly:
   #define RADIUS 80.0
  //float angle = 0;
  float zaxis,yaxis;
  for( float angle = 0.0; angle < 360.0; angle += 1.0 ) {
      yaxis = RADIUS * sin( radians( angle )) + 200;
      zaxis = RADIUS * cos( radians( angle )) + 200;
      set_arm( 0, yaxis, zaxis, 0 );
      delay( 1 );
  }*/
  int a = 0;
  int userinput = 1;
  int b = 0;
  int c = 0;
  int d = 0;
  int e =0;
  int f = 0;
  int delay_time = 20;
  int delay_time_movement = 100;

  pwm.begin();/*
* In theory the internal oscillator (clock) is 25MHz but it really isn't
* that precise. You can 'calibrate' this by tweaking this number until
* you get the PWM update frequency you're expecting!
* The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
* is used for calculating things like writeMicroseconds()
* Analog servos run at ~50 Hz updates, It is importaint to use an
* oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
* 1) Attach the oscilloscope to one of the PWM signal pins and ground on
*    the I2C PCA9685 chip you are setting the value for.
* 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
*    expected value (50Hz for most ESCs)
* Setting the value here is specific to each individual I2C PCA9685 chip and
* affects the calculations for the PWM update frequency. 
* Failure to correctly set the int.osc value will cause unexpected PWM results
*/
pwm.setOscillatorFrequency(27000000);
pwm.setPWMFreq(SERVO_FREQ);  // Analog servos run at ~50 Hz updates


  //home
  pwm.setPWM(BAS_SERVO, 0, 415);

  pwm.setPWM(SHL_SERVO, 0, 400);

  pwm.setPWM(ELB_SERVO, 0, 390);

  pwm.setPWM(WRI_SERVO, 0, 110);

  pwm.setPWM(WRO_SERVO, 0, 310);

  pwm.setPWM(GRI_SERVO, 0, 100);

  delay(100);



  //pwm.setPWM(TURNTABLE_SERVO, 0, 200);



  delay(100);



 a = 415;

  b = 400;

  c = 390;

  d = 110;

  e = 310;

  f = 100;

  delay_time = 20;
  delay_time_movement = 100;
 


delay(delay_time_movement);

if(first_grab){
    //********************************************************grab 1.1*****************************************
 //grab beads
   for (c = 390; c != 250; c = c - 5)

  {

    pwm.setPWM(ELB_SERVO, 0, c);

    delay(delay_time);

    if (b != 390)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b - 5;

    }

    if (d != 120)

    {

      pwm.setPWM(WRI_SERVO, 0, d);

      Serial.println(d);

      delay(delay_time);

      d = d + 5;

    }

  }





  //move to grab 2
  for(d = 120; d != 205; d = d + 5)
  {

    pwm.setPWM(WRI_SERVO, 0, d);
    
    delay(delay_time);
    
    if(b != 305)
    {
      
       pwm.setPWM(SHL_SERVO, 0, b);
    
       delay(delay_time);
       
       b = (b - 5);
    
    }
    if (c != 240)

    {

      pwm.setPWM(ELB_SERVO, 0, c);


      delay(delay_time);

      c = c - 5;

    }
    
  }
  
  delay(1000);



  //grip beads

  for (f = 0; f != 500; f = f + 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  delay(1000);


  //pull beads off I


  for (b = 305; b != 400; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);


    delay(delay_time);

    if (d != 190)

    {

      pwm.setPWM(WRI_SERVO, 0, d);

      Serial.println(d);

      delay(delay_time);

      d = d - 1;

    }

  }


  //move to drop

  for (a = 415; a != 135; a = a - 5)

  {

    pwm.setPWM(BAS_SERVO, 0, a);

    delay(delay_time);

    if (b != 350)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b - 5;

    }

    if (c != 310)
    {

      pwm.setPWM(ELB_SERVO, 0, c);

      Serial.println(c);

      delay(delay_time);

      c = c + 5;

    }

  }

  delay(delay_time_movement);



  //up and down

  for (d = 220; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 120 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }



  delay(delay_time_movement);



  //drop the beads

  for (f = 500; f != 0; f = f - 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  for (e = 325; e != 480; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

    for (e = 480; e != 150; e = e - 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }


    for (d = 120; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 120 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }
  
  for (e = 150; e != 310; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

 
  delay(1000);

 //********************************************************grab 1.2*****************************************
   //grab beads

  
   for (a = 130; a != 415; a = a + 5)

  {

    pwm.setPWM(BAS_SERVO, 0, a);

    delay(delay_time);

    if(c != 250)

    {

      pwm.setPWM(ELB_SERVO, 0, c);
      
      delay(delay_time);
      
      c = c - 5;
      
    }

    if (b != 390)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b + 5;

    }

  }

  delay(delay_time_movement);

  //move to grab 2
  for(d = 120; d != 205; d = d + 5)
  {

    pwm.setPWM(WRI_SERVO, 0, d);
    
    delay(delay_time);
    
    if(b != 310)
    {
      
       pwm.setPWM(SHL_SERVO, 0, b);
    
       delay(delay_time);
       
       b = (b - 5);
    
    }
    if (c != 240)

    {

      pwm.setPWM(ELB_SERVO, 0, c);


      delay(delay_time);

      c = c - 5;

    }
    
  }
  
  delay(1000);

  //inch forward

  for (b = 310; b != 303; b = b - 0.5)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

    if(d != 210)

    {

       pwm.setPWM(WRI_SERVO, 0, d);
    
       delay(delay_time);

       d = d + 0.5;
      
    }

  }

  delay(1000);



  //grip beads

  for (f = 0; f != 500; f = f + 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  delay(1000);


  //pull beads off I

  for (b = 303; b != 320; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

  }

  for (b = 320; b != 400; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);


    delay(delay_time);

    if (d != 190)

    {

      pwm.setPWM(WRI_SERVO, 0, d);

      Serial.println(d);

      delay(delay_time);

      d = d - 1;

    }

  }


  //move to drop

  for (a = 425; a != 150; a = a - 5)

  {

    pwm.setPWM(BAS_SERVO, 0, a);

    delay(delay_time);

    if (b != 340)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b - 5;

    }

    //was c != 310
    if (c != 290)
    {

      pwm.setPWM(ELB_SERVO, 0, c);

      Serial.println(c);

      delay(delay_time);

      c = c + 5;

    }

  }

  delay(delay_time_movement);



  //up and down

  for (d = 220; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 140 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }



  delay(delay_time_movement);



  //drop the beads

  for (f = 500; f != 0; f = f - 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  for (e = 325; e != 480; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

    for (e = 480; e != 150; e = e - 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }


    for (d = 140; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 140 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }
  
  for (e = 150; e != 325; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }


   pwm.setPWM(BAS_SERVO, 0, 400);
      delay(1000);

  pwm.setPWM(SHL_SERVO, 0, 400);

  pwm.setPWM(ELB_SERVO, 0, 390);

  pwm.setPWM(WRI_SERVO, 0, 110);

  pwm.setPWM(WRO_SERVO, 0, 310);

  pwm.setPWM(GRI_SERVO, 0, 0);

  delay(1000);

  /*  for (a = 130; a != 415; a = a + 5)

  {

    pwm.setPWM(BAS_SERVO, 0, a);

    delay(delay_time);

    if(c != 250)

    {

      pwm.setPWM(ELB_SERVO, 0, c);
      
      delay(delay_time);
      
      c = c - 5;
      
    }

    if (b != 390)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b + 5;

    }

  }

  delay(delay_time_movement);

  //move to grab 2
  for(d = 120; d != 205; d = d + 5)
  {

    pwm.setPWM(WRI_SERVO, 0, d);
    
    delay(delay_time);
    
    if(b != 310)
    {
      
       pwm.setPWM(SHL_SERVO, 0, b);
    
       delay(delay_time);
       
       b = (b - 5);
    
    }
    if (c != 240)

    {

      pwm.setPWM(ELB_SERVO, 0, c);


      delay(delay_time);

      c = c - 5;

    }
    
  }
  
  delay(1000);

  //inch forward

  for (b = 310; b != 303; b = b - 0.5)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

    if(d != 210)

    {

       pwm.setPWM(WRI_SERVO, 0, d);
    
       delay(delay_time);

       d = d + 0.5;
      
    }

  }

  delay(1000);



  //grip beads

  for (f = 0; f != 500; f = f + 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  delay(1000);


  //pull beads off I

  for (b = 303; b != 320; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

  }

  for (b = 320; b != 400; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);


    delay(delay_time);

    if (d != 190)

    {

      pwm.setPWM(WRI_SERVO, 0, d);

      Serial.println(d);

      delay(delay_time);

      d = d - 1;

    }

  }


  //move to drop

  for (a = 425; a != 150; a = a - 5)

  {

    pwm.setPWM(BAS_SERVO, 0, a);

    delay(delay_time);

    if (b != 340)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b - 5;

    }

    if (c != 290)
    {

      pwm.setPWM(ELB_SERVO, 0, c);

      Serial.println(c);

      delay(delay_time);

      c = c + 5;

    }

  }

  delay(delay_time_movement);



  //up and down

  for (d = 220; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 140 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }



  delay(delay_time_movement);



  //drop the beads

  for (f = 500; f != 0; f = f - 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  for (e = 325; e != 480; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

    for (e = 480; e != 150; e = e - 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }


    for (d = 140; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 140 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }
  
  for (e = 150; e != 325; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }
  
  pwm.setPWM(BAS_SERVO, 0, 400);
      delay(1000);

  pwm.setPWM(SHL_SERVO, 0, 400);

  pwm.setPWM(ELB_SERVO, 0, 390);

  pwm.setPWM(WRI_SERVO, 0, 110);

  pwm.setPWM(WRO_SERVO, 0, 310);

  pwm.setPWM(GRI_SERVO, 0, 0);

    delay(100);

 


delay(delay_time_movement);*/
}
  //****************************************************************************************************************************************
  //----------------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------- END FIRST GRAB -----------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------------------------------------------
  //****************************************************************************************************************************************



  //****************************************************************************************************************************************
  //----------------------------------------------------------------------------------------------------------------------------------------
  //------------------------------------------------- THE CODE BELOW IS FOR THE SECOND GRAB -------------------------------------------------
  //----------------------------------------------------------------------------------------------------------------------------------------
  //****************************************************************************************************************************************
   
  //********************************************************grab 2.1 *****************************************

  
  //grab beads
if(second_grab) {

 a = 390;

  b = 400;

  c = 390;

  d = 110;

  e = 310;

  f = 100;

  pwm.setPWM(BAS_SERVO, 0, a);

  for (c = 390; c != 250; c = c - 5)

  {

    pwm.setPWM(ELB_SERVO, 0, c);

    delay(delay_time);

    if (b != 390)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b - 5;

    }

    if (d != 120)

    {

      pwm.setPWM(WRI_SERVO, 0, d);

      Serial.println(d);

      delay(delay_time);

      d = d + 5;

    }

  }





  //move to grab 2
  for(d = 120; d != 205; d = d + 5)
  {

    pwm.setPWM(WRI_SERVO, 0, d);
    
    delay(delay_time);
    
    if(b != 320)
    {
      
       pwm.setPWM(SHL_SERVO, 0, b);
    
       delay(delay_time);
       
       b = (b - 5);
    
    }
    if (c != 245)

    {

      pwm.setPWM(ELB_SERVO, 0, c);


      delay(delay_time);

      c = c - 5;

    }
    
  }
  
  delay(1000);


  //grip beads

  for (f = 0; f != 500; f = f + 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  delay(1000);


  //pull beads off I

  for (b = 320; b != 400; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);


    delay(delay_time);

    if (d != 190)

    {

      pwm.setPWM(WRI_SERVO, 0, d);

      Serial.println(d);

      delay(delay_time);

      d = d - 1;

    }

  }


  //move to drop

  for (a = 400; a != 300; a = a - 5)

  {

    pwm.setPWM(BAS_SERVO, 0, a);

    delay(delay_time);

    if (b != 320)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b - 5;

    }

    if (c != 290)
    {

      pwm.setPWM(ELB_SERVO, 0, c);

      Serial.println(c);

      delay(delay_time);

      c = c + 5;

    }

  }
    
    for (a = 300; a != 155; a = a - 5)

  {

    pwm.setPWM(BAS_SERVO, 0, a);

    delay(delay_time);
  }
  

  delay(delay_time_movement);



  //up and down

  for (d = 220; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 140 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }



  delay(delay_time_movement);



  //drop the beads

  for (f = 500; f != 0; f = f - 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  for (e = 325; e != 480; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

    for (e = 480; e != 150; e = e - 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }


    for (d = 140; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 140 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }
  
  for (e = 150; e != 310; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

 
  delay(1000);

 //********************************************************grab 2.2*****************************************
   //grab beads
   for (a = 160; a != 400; a = a + 5)

  {

    pwm.setPWM(BAS_SERVO, 0, a);

    delay(delay_time);

    if(c != 250)

    {

      pwm.setPWM(ELB_SERVO, 0, c);
      
      delay(delay_time);
      
      c = c - 5;
      
    }

    if (b != 390)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b + 5;

    }

  }

  delay(delay_time_movement);

  //move to grab 2
  for(d = 120; d != 205; d = d + 5)
  {

    pwm.setPWM(WRI_SERVO, 0, d);
    
    delay(delay_time);
    
    if(b != 320)
    {
      
       pwm.setPWM(SHL_SERVO, 0, b);
    
       delay(delay_time);
       
       b = (b - 5);
    
    }
    if (c != 240)

    {

      pwm.setPWM(ELB_SERVO, 0, c);


      delay(delay_time);

      c = c - 5;

    }
    
  }
  
  delay(1000);

  //inch forward

  for (b = 320; b != 315; b = b - 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

    if(d != 210)

    {

       pwm.setPWM(WRI_SERVO, 0, d);
    
       delay(delay_time);

       d = d + 1;
      
    }

  }

  delay(1000);



  //grip beads

  for (f = 0; f != 500; f = f + 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  delay(1000);


  //pull beads off I

  for (b = 315; b != 320; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);

    delay(delay_time);

  }

  for (b = 320; b != 400; b = b + 1)

  {

    pwm.setPWM(SHL_SERVO, 0, b);


    delay(delay_time);

    if (d != 190)

    {

      pwm.setPWM(WRI_SERVO, 0, d);

      Serial.println(d);

      delay(delay_time);

      d = d - 1;

    }

  }


  //move to drop

  for (a = 405; a != 150; a = a - 5)

  {

    pwm.setPWM(BAS_SERVO, 0, a);

    delay(delay_time);

    if (b != 340)

    {

      pwm.setPWM(SHL_SERVO, 0, b);

      Serial.println(b);

      delay(delay_time);

      b = b - 5;

    }

    if (c != 310)
    {

      pwm.setPWM(ELB_SERVO, 0, c);

      Serial.println(c);

      delay(delay_time);

      c = c + 5;

    }

  }

  delay(delay_time_movement);



  //up and down

  for (d = 220; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 140 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }



  delay(delay_time_movement);



  //drop the beads

  for (f = 500; f != 0; f = f - 5)

  {

    pwm.setPWM(GRI_SERVO, 0, f);

    delay(delay_time);

  }

  for (e = 325; e != 480; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

    for (e = 480; e != 150; e = e - 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }


    for (d = 140; d != 270; d = d + 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }

  for (d = 270; d != 140 ; d = d - 5)

  {

    pwm.setPWM(WRI_SERVO, 0, d);

    Serial.println(d);

    delay(delay_time);

  }
  
  for (e = 150; e != 325; e = e + 5)

  {

    pwm.setPWM(WRO_SERVO, 0, e);
    
    delay(delay_time);
    
  }

 
  delay(1000);
  
  pwm.setPWM(BAS_SERVO, 0, 405);
      delay(1000);

  pwm.setPWM(SHL_SERVO, 0, 400);

  pwm.setPWM(ELB_SERVO, 0, 390);

  pwm.setPWM(WRI_SERVO, 0, 110);

  pwm.setPWM(WRO_SERVO, 0, 310);

  pwm.setPWM(GRI_SERVO, 0, 0);
}
}
