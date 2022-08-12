
/*********************************************************************
 * Module name: lidar_scanner_pseudo_code.ino
 *
 * Author:  
 * 
 * Created on July 11th, 2022
 * 
 * Version  1.0: 
 *          1.1: Converted output to xyz file format for direct viewing in
 *                MeshLab 07/16/2022
 *                
 *                  
 ************************************************************************/

#define VERSION "1.0"

/************************************************************************
*
* Include section
*
*************************************************************************/
#include <Servo.h>
#include <Wire.h>
#include <LIDARLite.h>

/************************************************************************
*
* Harware Connections and Constants
*
*************************************************************************/
const int SERVO_PIN=9; //Servo Signal Connected to D3
const int STEPPER_STEP=7; //Stepper motor step control board (DRV8829) on D9
const int STEPPER_DIR=6; //Stepper direction on D8, other stepper motor driver connections are left unconnected and remain at default values
const int LIGHT_SENSOR=4; //light sensor analog input
const int TIME_BETWEEN_READINGS=10; //time between readings ms
const int TILT_SERVO_SCALE_FACTOR=1; //scale factor to convert degrees of rotation to PWM value
const int LIGHT_SENSOR_THRESHOLD=200; //an ADC value above this indicates the light sensor is blocked
const int STEPPER_DIRECTION_FWD=1;
const int STEPPER_DIRECTION_REV=0;
/************************************************************************
*
* Global Objects and Variables
*
*************************************************************************/
Servo tilt_servo;
LIDARLite lidar_module;

/************************************************************************
*
* Function Declarations
*
*************************************************************************/
void step_azimuth(); //rotate around vertical axis
void rotate_azimuth_to_start();
void tilt_altitude(int degrees); //rotate around horizontal axis
void read_distance();


/************************************************************************
*
* Function Declaration Section
*
*
*************************************************************************/

/*************************************************************************
* Function name      : setup
* returns            : void
* Description        : 
* Notes              : 
**************************************************************************/
void setup() {
  //initialize serial port with 9600 Baud rate
  Serial.begin(9600);

  //initial servo object
  tilt_servo.attach(SERVO_PIN);

  //setup digitial pins for controlling stepper motor
  pinMode(STEPPER_STEP,OUTPUT);
  pinMode(STEPPER_DIR,OUTPUT);
  digitalWrite(STEPPER_STEP,LOW);

  //initialize lidar module
  lidar_module.begin(0,true);
  lidar_module.configure(0);      
}


/*************************************************************************
* Function name      : loop
* returns            : void
* Description        : main loop 
* Notes              : 
**************************************************************************/
void loop() {
  int azimuth_pos; //angle in degrees with x-axis when projected on horizontal plane, increases counter-clockwise
  int altitude_pos; //and with vertical in degrees
  double r,x,y,z;
  double theta, phi, radius;
  const double pi=3.142;
    
  //move to start position
  //rotate_azimuth_to_start();
  tilt_servo.write(0);
  delay(15);
  
  //send data start marker on serial port
  Serial.println("START");
  Serial.println("X Y Z"); //print header of xyz file in space delimited format
  
  //take readings at for 360 degrees of rotation at each tilt angle
  for(altitude_pos=0;altitude_pos<90; altitude_pos+=15){
    //set tilt
    tilt_servo.write(altitude_pos);
    delay(15);
      //rotate through 360 degreess  
      for(azimuth_pos=0;azimuth_pos<500;azimuth_pos++){  

        //measure distance and convert from spherical
        //to cartesian coordinates
        
        r=double(lidar_module.distance()); //distance in cm
        phi=double(90-altitude_pos)*pi/180.0; //convert angles to radians
        theta=double(azimuth_pos)*pi/250.0;
        z=r*cos(phi);
        x=r*sin(phi)*cos(theta);
        y=r*sin(phi)*sin(theta);
              
        //print readings on serial port in xyz file format (space delimited)
        Serial.print(x);
        Serial.print(" ");
        Serial.print(y);
        Serial.print(" ");
        Serial.println(z);

        //move to next azimuth position
        step_azimuth(STEPPER_DIRECTION_FWD);
        delay(TIME_BETWEEN_READINGS);
      }

    //go back to start position for rotatations
    //rotate_azimuth_to_start();
    //move to next altitude position  
  }

  //send data end marker on serial port
  Serial.println("STOP");  
}

/*************************************************************************
* Function name      : step_azimuth()
*
*  returns           : void
* 
*
* Description        : Steps the azimuth position
* Notes              : none
* Unit Test          : 
**************************************************************************/
void step_azimuth(int stepper_direction){
  //creating a rising edge on the STEP pin of the DRV8829 will result in a step motion
  //direction pin is set to HIGH, toggle to reverse step direction
  //all other connections to the DRV8829 board are assumed to be left disconnected and will hence assume their default values

  digitalWrite(STEPPER_DIR,stepper_direction); //set stepper motor direction
    
  //generate a pulse on the STEP pin
  digitalWrite(STEPPER_STEP,HIGH);
  delay(100);
  digitalWrite(STEPPER_STEP,LOW);
  delay(100);  
}


/*************************************************************************
* Function name      : rotate_azimuth_to_start()
*
*  returns           : void
* 
*
* Description        : returns the azimuth position to the start postion based on the light sensor
* Notes              : none
* Unit Test          : 
**************************************************************************/
void rotate_azimuth_to_start(){

  int step_count,i;
  const int MAX_ATTEMPTS_STEPS=400;

  //if the light sensor is blocked, reverse the vane 
  step_count=0;
  
  if(analogRead(LIGHT_SENSOR)>LIGHT_SENSOR_THRESHOLD){
    while((analogRead(LIGHT_SENSOR)>LIGHT_SENSOR_THRESHOLD)&& (step_count<MAX_ATTEMPTS_STEPS)){    
      step_azimuth(STEPPER_DIRECTION_REV);
      step_count++;
    }
    //ensure light sensor is cleared
    for(step_count=0;step_count<50;step_count++){
      step_azimuth(STEPPER_DIRECTION_REV);
    }
  }

  if(step_count==MAX_ATTEMPTS_STEPS){
    Serial.println("failed to find start azimuth position");
    exit;
  }
 
  //step to start position
  step_count=0;
  
  while((analogRead(LIGHT_SENSOR)<LIGHT_SENSOR_THRESHOLD)&& (step_count<MAX_ATTEMPTS_STEPS)){    
    step_azimuth(STEPPER_DIRECTION_FWD);
    step_count++;
  }
  
  if(step_count==MAX_ATTEMPTS_STEPS){
    Serial.println("failed to find start azimuth position");
    exit;
  } 
}
