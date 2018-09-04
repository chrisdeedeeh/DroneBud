/******************************************************************************************************************************************************

  /\\\\\\\\\\\\                                                                 /\\\\\\\\\\\\\                         /\\\        
  \/\\\////////\\\                                                              \/\\\/////////\\\                      \/\\\       
   \/\\\      \//\\\                                                             \/\\\       \/\\\                      \/\\\     
    \/\\\       \/\\\  /\\/\\\\\\\     /\\\\\     /\\/\\\\\\       /\\\\\\\\      \/\\\\\\\\\\\\\\   /\\\    /\\\        \/\\\     
     \/\\\       \/\\\ \/\\\/////\\\  /\\\////\\\  \/\\\////\\\    /\\\/////\\\    \/\\\/////////\\\ \/\\\   \/\\\   /\\\\\\\\\    
      \/\\\       \/\\\ \/\\\   \///  /\\\   \//\\\ \/\\\  \//\\\  /\\\\\\\\\\\     \/\\\       \/\\\ \/\\\   \/\\\  /\\\////\\\   
       \/\\\       \/\\\ \/\\\        \//\\\  /\\\   \/\\\   \/\\\ \//\\///////      \/\\\       \/\\\ \/\\\   \/\\\ \/\\\  \/\\\ 
        \/\\\\\\\\\\\\/   \/\\\         \///\\\\\/    \/\\\   \/\\\  \//\\\\\\\\\\    \/\\\\\\\\\\\\\/  \//\\\\\\\\\  \//\\\\\\\/\\
         \////////////     \///            \/////      \///    \///    \//////////     \/////////////     \/////////    \///////\//  v2.1

******************************************************************************************************************************************************

Welcome to The Drone Bud flight control template. This is a fully functioning drone flight control
program, but only if you use these components:
  
  Arduino Due
  MPU 6050
  BMP 280 Altitude Hold Coming Soon

However, this program is well documented and easy to understand. It's a breeze to put in your own sensors
and even completely different sensors. If the sensor you have in mind has an arduino library, your job
will be even easier. 


Thanks to http://patorjk.com/software/taag/ for that sweet font
******************************************************************************************************************************************************/

#include <Servo.h>
#include <SPI.h>
#include <PID_v1.h>
#include <Adafruit_Sensor.h>
#include <MPU6050_tockn.h>
#include <Wire.h>
#include <math.h>

  MPU6050 mpu6050(Wire);
  
  /*******
   * PID Gains
   *******/

   double Xp = 1.5; double Xi = 0.0; double Xd = 0.02;
   double Yp = 1.5; double Yi = 0.0; double Yd = 0.02;
   double Zp = 0.3; double Zi = 0; double Zd = 0;
   double Up = 1; double Ui = 0; double Ud = 0;


  double roll, pitch, yawrate, upspeed;
  double xchange, ychange, zchange, upchange;
  double altered_inputx, altered_inputy, altered_inputz, altered_inputup;
  
  /*******
   * PID Controllers
   *******/
  PID PIDx(&roll,&xchange,&altered_inputx,Xp,Xi,Xd,DIRECT);
  PID PIDy(&pitch,&ychange,&altered_inputy,Yp,Yi,Yd,DIRECT);
  PID PIDz(&yawrate,&zchange,&altered_inputz,Zp,Zi,Zd,DIRECT);
  PID PIDup(&upspeed,&upchange,&altered_inputup,Up,Ui,Ud,DIRECT);

  /***********************
  Flight Control Variables
  ************************/
    
  static int throttleMAX = 2000; //2000 The max throttle makes sure that esc outputs don't exceed the max signal length.  
  static int throttleMIN = 1100; //1000 
  static double midpointup = 1500; //midpoints can be trimmed right on the controller, read them then type them in here. 
  static double midpointroll = 1500;
  static double midpointpitch = 1500;
  static double midpointyaw = 1500; 

  static float max_angle = 40.0;
  
  unsigned long looptime = 0;
  unsigned long looptimeholder = 0;
  
  long frontrightchange = 1000;
  long frontleftchange = 1000;
  long backrightchange = 1000;
  long backleftchange = 1000;
  
  Servo frontright;
  Servo frontleft;
  Servo backright;
  Servo backleft; //motor pins, where the escs are connected to. 

  static int rollpin = 31;
  volatile long startroll = 0; volatile long inputroll = 0; volatile byte stateroll = 0; 
  //These record the variables for the receiver inputs. They record the start time of the signal, |^
  static int pitchpin = 33;
  volatile long startpitch = 0; volatile long inputpitch = 0; volatile byte statepitch = 0; 
  //the length of the signal, and whether it is on. The "state_" variable records the time, and  
  static int yawpin = 37;
  volatile long startyaw = 0; volatile long inputyaw = 0; volatile byte stateyaw = 0; //is used below in the PID calculation. 
  static int throttlepin = 35;
  volatile long startthrottle = 0; volatile long inputthrottle = 0; volatile byte statethrottle = 0;
  static int onoffpin = 41;
  volatile long startonoff = 0; volatile long inputonoff = 0; volatile byte stateonoff = 0;

  bool armed = false;
  
//Interrupt functions for the receiver. They are needed because the receiver sends its signals independent of the arduino's timer. They could come at any time.
//The microcontroller uses interrupts to get the data when it enters. 
void interrupt(){             // Declare the interrupt function
  long timenow = micros();    //create a variable that stores the current time in microseconds
  if((digitalRead(rollpin) == HIGH) && stateroll == 0){  //This checks whether a signal is coming in, and if it is, set the state to high. 
   startroll = timenow;
   stateroll = 1;
  } else if ((digitalRead(rollpin) == LOW) && stateroll == 1){ //When the signal stops, the length of the signal is recorded, which indicates the position of the stick. 
    inputroll = timenow - startroll;
    stateroll = 0;
  } 
  if((digitalRead(pitchpin) == HIGH) && statepitch == 0){ //The if statement above is repeated for every input channel. There are 6. 
   startpitch = timenow;
   statepitch = 1;
  } else if ((digitalRead(pitchpin) == LOW ) && statepitch == 1){
    inputpitch = timenow - startpitch;
    statepitch = 0;
  } 
     timenow = micros();
  if((digitalRead(yawpin) == HIGH) && stateyaw == 0){
   startyaw = timenow;
   stateyaw = 1;
  } else if ((digitalRead(yawpin) == LOW ) && stateyaw == 1){
    inputyaw = timenow - startyaw;
    stateyaw = 0;
  } 
  if((digitalRead(throttlepin) == HIGH) && statethrottle == 0){
   startthrottle = timenow;
   statethrottle = 1;
  } else if ((digitalRead(throttlepin) == LOW ) && statethrottle == 1){
    inputthrottle = timenow - startthrottle;
    statethrottle = 0;
  }
      timenow = micros();
  if((digitalRead(onoffpin) == HIGH) && stateonoff == 0){
   startonoff = timenow;
   stateonoff = 1;
  } else if ((digitalRead(onoffpin) == LOW ) && stateonoff == 1){
    inputonoff = timenow - startonoff;
    stateonoff = 0;
  } 
}

void setup() { 
  
  Serial.begin(9600);
  
  frontright.attach(3);
  frontleft.attach(6);
  backright.attach(5);
  backleft.attach(4); //attach our servo pins
      
  pinMode(LED_BUILTIN, OUTPUT); //used to indicate armed status
  pinMode(rollpin,INPUT_PULLUP);
  pinMode(yawpin,INPUT_PULLUP);
  pinMode(pitchpin,INPUT_PULLUP);
  pinMode(throttlepin,INPUT_PULLUP);
  
  attachInterrupt(rollpin,interrupt,CHANGE);
  attachInterrupt(yawpin,interrupt,CHANGE);
  attachInterrupt(pitchpin,interrupt,CHANGE); //Attach the interrupt function to the receiver pins. 
  attachInterrupt(throttlepin,interrupt,CHANGE);
  attachInterrupt(onoffpin,interrupt,CHANGE);
  
  frontleft.writeMicroseconds(975);
  frontright.writeMicroseconds(975);
  backleft.writeMicroseconds(975);
  backright.writeMicroseconds(975); //write low

  PIDx.SetMode(AUTOMATIC); PIDx.SetSampleTime(2); PIDx.SetOutputLimits(-500, 500);
  PIDy.SetMode(AUTOMATIC); PIDy.SetSampleTime(2); PIDy.SetOutputLimits(-500, 500);
  PIDz.SetMode(AUTOMATIC); PIDz.SetSampleTime(2); PIDz.SetOutputLimits(-500, 500);
  PIDup.SetMode(AUTOMATIC); PIDup.SetSampleTime(2); PIDup.SetOutputLimits(-500, 500);

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  delay(3000); //wait for the loop. 
}



//********************
// These functions control the PID control algorithms. There is one function per axis. 
//********************


void alterX(){
  PIDx.Compute();
  //Serial.print(xchange);Serial.print("\t");
  frontrightchange -= xchange;
  frontleftchange += xchange;
  backrightchange -= xchange;
  backleftchange += xchange;
  }
void alterY(){
  PIDy.Compute();
    //Serial.print(ychange);Serial.print("\t");
  frontrightchange -= ychange;
  frontleftchange -= ychange;
  backrightchange += ychange;
  backleftchange += ychange;
  }
void alterZ(){
  PIDz.Compute();
    //Serial.print(zchange);Serial.print("\t");
  frontrightchange -= zchange;
  frontleftchange += zchange;
  backrightchange += zchange;
  backleftchange -= zchange;
  }
 void alterUp(){
  double throttle = inputthrottle;
  if (throttle > 1800){
    throttle = 1800;
  }
  double upchange = throttle;
//PIDup.Compute()
  frontrightchange = upchange;
  frontleftchange = upchange;
  backrightchange = upchange;
  backleftchange = upchange;
  }

volatile int counter = 0;
volatile long looptimecounter = 0;
void loop() {
  
  //Get the last loop time.
  looptime = micros() - looptimeholder;
  looptimeholder = micros();
  /*
  looptimecounter += looptime;
  counter += 1;
  if(counter == 10){
      Serial.println(looptimecounter / 10);
      looptimecounter = 0;
      counter = 0;
  }
  */
  mpu6050.update();
  
  roll = mpu6050.getAngleX();
  pitch = mpu6050.getAngleY();
  yawrate = mpu6050.getGyroZ();


  //Serial.print(roll);Serial.print("\t");
  //Serial.print(pitch);Serial.print("\t");
  //Serial.print(yawrate);Serial.print("\t");


  altered_inputx = (inputroll - 1500.0) / 500.0 * max_angle;
  altered_inputy = (inputpitch - 1500.0) / 500.0 * max_angle;
  altered_inputz = inputyaw - 1500.0;
  altered_inputup = inputthrottle - 1500.0;
  
  if(inputonoff > 1500){
    armed = true;
  }else{
    armed = false;
  }
  if(armed == true){

  alterUp(); //This has to go first, because it initializes the baseline throttle values
  alterX();
  alterY();
  alterZ(); 
  }
  
  if(frontrightchange < throttleMIN)frontrightchange = throttleMIN;
  if(frontleftchange < throttleMIN)frontleftchange = throttleMIN;
  if(backrightchange < throttleMIN)backrightchange = throttleMIN;
  if(backleftchange < throttleMIN)backleftchange = throttleMIN;
  
  if(frontrightchange > throttleMAX)frontrightchange = throttleMAX;
  if(frontleftchange > throttleMAX)frontleftchange = throttleMAX;
  if(backrightchange > throttleMAX)backrightchange = throttleMAX;
  if(backleftchange > throttleMAX)backleftchange = throttleMAX; //make sure that motor values do not go above 2000 or below 1000. 

if(armed == false){
   frontrightchange = 1000;
   frontleftchange = 1000;
   backrightchange = 1000;
   backleftchange = 1000;
    }

  /*  //Uncomment for Serial Monitor Values to be displayed
  Serial.print(inputonoff);
  Serial.print("\t");
  Serial.print(inputyaw);  Serial.print("\t");
  Serial.print(inputroll);  Serial.print("\t");
  Serial.print(inputthrottle);  Serial.print("\t");
  Serial.print(inputpitch); Serial.print("\t");*/
 Serial.print("rpy:"); 
 Serial.print(roll);Serial.println("\t");/*
 Serial.print(pitch);Serial.print("\t");
 Serial.print(yawrate);Serial.print("\t");
 Serial.print("\t frontright:"); Serial.print(frontrightchange);
 Serial.print("\t frontleft: ");Serial.print(frontleftchange);
 Serial.print("\t backright:"); Serial.print(backrightchange);
 Serial.print("\t backleft:");Serial.println(backleftchange);
*/
  
  
  frontright.writeMicroseconds(frontrightchange);
  frontleft.writeMicroseconds(frontleftchange);
  backright.writeMicroseconds(backrightchange);
  backleft.writeMicroseconds(backleftchange);

}


