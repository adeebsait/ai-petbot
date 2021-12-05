

/*
  Arduino Two Wheeled Self - balancing Robot

  The code :
  - Reads data from the MPU6050 and calculates the angle
  - Calculates the motor speed based on the angle error with a PID
  controller
  - Reads the serial port and changes the angular set point or
  rotational speed
  - Changes the angular set point if the robot is moving when no
  steering command is given

  The code runs at 250 Hz on a Arduino UNO R3 with a clock speed of MHz

  Created 2020 by :
  Fredrik Ihrfelt ( ihrfelt@kth . se )
  William Marin ( wmarin@kth . se )
*/

// --- INCLUDED LIBRARIES - - -//

#include <Wire.h>
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050
#define L_dirPin 6
#define L_stepPin 8
#define R_dirPin 7
#define R_stepPin 5

#include <SoftwareSerial.h>

SoftwareSerial s(10, 11);

MPU6050 mpu;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

/*********Tune these 4 values for your BOT*********/
double setpoint = 178 ; //set the value when the bot is perpendicular to ground using serial monitor.
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 27; //Set this first
double Kd = 0; //Set this secound
double Ki = 0; //Finally set this
/******End of values setting*********/
double pid_setpoint;
byte received_byte;
//#define BLYNK_DEVICE_NAME "El Chapo Joystick"

// --- DECLARING I2C ADDRESS OF MPU6050 - - -//
int gyro_address = 0x6A ;
int acc_calibration_value = 124;

// --- SETTING PID PARAMETERS - - -//
float pid_p_gain = Kp;//12
float pid_i_gain = Ki;//0.5
float pid_d_gain = Kd;//22

// Variables for the angle and PID controller
float angle_gyro , angle_acc , angle , self_balance_pid_setpoint ;
float pid_error_temp , pid_i_mem  , gyro_input , pid_output , pid_last_d_error ;
float pid_output_left , pid_output_right ;

double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

// --- DECLARING GLOBAL VARIABLES - - -//
byte start ;
int left_motor , throttle_left_motor , throttle_counter_left_motor ,
    throttle_left_motor_memory ;
int right_motor , throttle_right_motor , throttle_counter_right_motor ,
    throttle_right_motor_memory ;
int gyro_pitch_data_raw , gyro_yaw_data_raw , accelerometer_data_raw ;

long gyro_yaw_calibration_value , gyro_pitch_calibration_value ;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

unsigned long loop_timer ;


// Variables for steering
float spd = 1; // Stores the desired speed value
float rotation = 1; // Stores the rotation value
float turning_speed = 30;
float max_target_speed = 200;
float desired_rotation ;
float desired_speed ;

// Declaring constants for serial port reading
const byte buffSize = 40;
char inputBuffer [ buffSize ];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false ;
boolean newDataFromEsp = false ;

char messageFromEsp [ buffSize ] = {0};
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int16_t ax, ay, az;
int16_t gx, gy, gz;

int8_t indexofX, indexofY;
String xval;
String yval;
char c;
String dataIn;

int x;
int y;

void setup () {
  // Starting the serial port att 115200 kbps
  Serial.begin(115200) ;
  s.begin(9600);
  // Setting the I2C clock speed to 400 kHz
  TWBR = 12;

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  devStatus = mpu.dmpInitialize();


  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(174);
  mpu.setYGyroOffset(-57);
  mpu.setZGyroOffset(-80);
  mpu.setZAccelOffset(1812);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255);
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Creating variable pulse for stepper motor control , TIMER2_COMPA_vect
  TCCR2A = 0;
  TCCR2B = 0;
  TIMSK2 |= (1 << OCIE2A ) ;
  TCCR2B |= (1 << CS21 ) ;
  OCR2A = 39;
  TCCR2A |= (1 << WGM21 ) ;


  //   Defining outputs
  pinMode(L_stepPin, OUTPUT);
  pinMode(L_dirPin, OUTPUT);
  pinMode(R_stepPin, OUTPUT);
  pinMode(R_dirPin, OUTPUT);
  pinMode(13 , OUTPUT );


  gyro_pitch_calibration_value = -36;
  gyro_yaw_calibration_value = -78;

  // Creating loop timer to achieve 250 Hz frequency
  loop_timer = micros() + 4000;

}

// --- MAIN LOOP - - -//
void loop () {
  //Blynk.run();
  // Reading data from the serial port
  //getDataFromEsp() ;
  if (s.available()) {
    getRoot();
  }
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors
    pid.Compute();

    //Print the value of Input and Output on serial monitor to check how it is working.
    //Serial.print(input); Serial.print(" =>"); Serial.println(output);
  }
  
  
  // Calculate forvard speed from serial reading
  if ( spd > 1) {
    desired_speed = max_target_speed * ( spd - 1) ;
  }

  // Calculate backward speed from serial reading
  if ( spd < 1) {
    desired_speed = max_target_speed * (1 - spd ) ;
  }

  // Calculate clockwise rotation speed from serial reading
  if ( rotation > 1) {
    desired_rotation = turning_speed * ( rotation - 1) ;
  }

  // Calculate counter clockwise rotation speed from serial reading
  if ( rotation < 1) {
    desired_rotation = turning_speed * (1 - rotation );
  }

  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  //Serial.println(ax);
  accelerometer_data_raw = ax;                      //Combine the two bytes to make one integer
  accelerometer_data_raw += acc_calibration_value;                          //Add the accelerometer calibration value
  if (accelerometer_data_raw > 8200)accelerometer_data_raw = 8200;          //Prevent division by zero by limiting the acc data to +/-8200;
  if (accelerometer_data_raw < -8200)accelerometer_data_raw = -8200;        //Prevent division by zero by limiting the acc data to +/-8200;

  angle_acc = asin((float)accelerometer_data_raw / 8200.0) * 57.296;        //Calculate the current angle according to the accelerometer

  // Set the angle of the gyro to the angle of the accelerometer if the robot is vertical
  if ( start == 0 && angle_acc > -0.5 && angle_acc < 0.5) {
    angle_gyro = angle_acc ;
    start = 1;
  }


  gyro_yaw_data_raw = gz;
  gyro_pitch_data_raw = gy;

  // Compensate the angle data with the calibration value
  gyro_pitch_data_raw -= gyro_pitch_calibration_value ;
  Serial.println( setpoint ) ;
  //angle_gyro -= gyro_yaw_data_raw * 0.00031;

  gyro_yaw_data_raw -= gyro_yaw_calibration_value ;
  //Serial.println( gyro_yaw_data_raw ) ;

  angle_gyro -= gyro_yaw_data_raw * -0.000003;                            //Compensate the gyro offset when the robot is rotating
  //Serial.println(angle_gyro);
  // Corecting gyro drift with complementary filter
  angle_gyro = angle_gyro * 0.9996 + angle_acc * 0.0004;



  // Print the angle of the gyro for experiments
  //Serial.println( gyro_pitch_data_raw ) ;
  //Serial.println( gyro_yaw_data_raw ) ;
  //Serial.println( gyro_pitch_calibration_value ) ;
  //Serial.println( gyro_yaw_calibration_value ) ;
  //Serial.println( angle_acc ) ;
  //Serial.println(accelerometer_data_raw);
  pid_output = output;
  //Serial.println( angle_gyro ) ;

  Serial.println( output ) ;


  // --- PID CONTROLLER - - -// 
  // Calculating the angular error
  pid_error_temp = angle_gyro - self_balance_pid_setpoint - pid_setpoint ;
  if ( pid_output > 10 || pid_output < -10) pid_error_temp += pid_output * 0.015 ;

  // Calculating the value on the I - part and add it to i_mem
  pid_i_mem += pid_i_gain * pid_error_temp ;

  // Limit the maximum I - part value
  if ( pid_i_mem > 400) pid_i_mem = 400;
  else if ( pid_i_mem < -400) pid_i_mem = -400;

  // Calculating PID controller output
  //pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * ( pid_error_temp - pid_last_d_error ) ;
  if ( pid_output > 400) pid_output = 400;
  else if ( pid_output < -400) pid_output = -400;

  // Storing the error for the next loop
  pid_last_d_error = pid_error_temp ;

  // Creating a dead band for small PID outputs
  if ( output < 5 && output > -5) pid_output = 0;

  
  // --- CONTROLLER OUTPUTS - - -//

  // Copying the PID output to the right and left motors
  pid_output_left = pid_output ;
  pid_output_right = pid_output ;

  if(received_byte & B00000001){                                            //If the first bit of the receive byte is set change the left and right variable to turn the robot to the left
    Serial.println("R");
    pid_output_left += turning_speed;                                       //Increase the left motor speed
    pid_output_right -= turning_speed;                                      //Decrease the right motor speed
  }
  if(received_byte & B00000010){                                            //If the second bit of the receive byte is set change the left and right variable to turn the robot to the right
    Serial.println("L");
    pid_output_left -= turning_speed;                                       //Decrease the left motor speed
    pid_output_right += turning_speed;                                      //Increase the right motor speed
  }

  if(received_byte & B00000100){                                            //If the third bit of the receive byte is set change the left and right variable to turn the robot to the right
    Serial.println("R");
    if(setpoint < 180.5)setpoint += 0.05;                             //Slowly change the setpoint angle so the robot starts leaning backwards
    if(pid_output < desired_speed)setpoint += 0.05; 
                //Slowly change the setpoint angle so the robot starts leaning forwards
  }
  if(received_byte & B00001000){                                            //If the forth bit of the receive byte is set change the left and right variable to turn the robot to the right
    Serial.println("F");
    if(setpoint > 175.5)setpoint -= 0.05;                            //Slowly change the setpoint angle so the robot starts leaning forwards
    if(pid_output > desired_speed * -1)setpoint -= 0.05;                //Slowly change the setpoint angle so the robot starts leaning backwards
  }   

  if(!(received_byte & B00001100)){                                         //Slowly reduce the setpoint to zero if no forward or backward command is given
    Serial.println("S");
    //spd = 1;
    if(setpoint > 178.5)setpoint -=0.05;                              //If the PID setpoint is larger then 0.5 reduce the setpoint with 0.05 every loop
    else if(setpoint < 177.5)setpoint +=0.05;                        //If the PID setpoint is smaller then -0.5 increase the setpoint with 0.05 every loop
    else setpoint = 178;                                                  //If the PID setpoint is smaller then 0.5 or larger then -0.5 set the setpoint to 0
  }

  // Change the PID angle setpoint to compensate for off centered weight
  if ( pid_setpoint == 0) {
    if ( pid_output < 0) self_balance_pid_setpoint += 0.0115;
    if ( pid_output > 0) self_balance_pid_setpoint -= 0.0115;

  }
  Serial.println(pid_output);
  //Serial.println(spd);
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
    mpu.dmpGetGravity(&gravity, &q); //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr

    input = ypr[1] * 180 / M_PI + 180;
  }

  // --- CALCULATING THE STEPPER MOTOR PULSE - - -//
  // Linearize the stepper motors non - linear behavior
  if ( pid_output_left > 0) pid_output_left = 405 - (1 / ( pid_output_left + 9) ) * 5500;
  else if ( pid_output_left < 0) pid_output_left = -405 - (1 / (pid_output_left - 9) ) * 5500;

  if ( pid_output_right > 0) pid_output_right = 405 - (1 / (pid_output_right + 9) ) * 5500;
  else if ( pid_output_right < 0) pid_output_right = -405 - (1 / (pid_output_right - 9) ) * 5500;

  // Calculate the pulse time for the stepper motors
  if ( pid_output_left > 0) left_motor = 400 - pid_output_left ;
  else if ( pid_output_left < 0) left_motor = -400 - pid_output_left ;
  else left_motor = 0;

  if ( pid_output_right > 0) right_motor = 400 - pid_output_right ;
  else if ( pid_output_right < 0) right_motor = -400 - pid_output_right ;
  else right_motor = 0;

  // Copy the pulse time to the throttle variables so the interrupt subroutine can use them
  throttle_left_motor = left_motor ;
  throttle_right_motor = right_motor ;

  //Serial.println(throttle_left_motor);
  //Serial.println(throttle_right_motor);
  // Delay loop if the time is under 40 ms (250 Hz )
  while ( loop_timer > micros() ) ;
  loop_timer += 4000;
}


// --- INTERRUPT ROUTINE FOR TIMER2_COMPA_vect - - -//
ISR ( TIMER2_COMPA_vect ) {
  // Left motor pulse calculations
  throttle_counter_left_motor++;
  if ( throttle_counter_left_motor > throttle_left_motor_memory ) {
    throttle_counter_left_motor = 0;
    throttle_left_motor_memory = throttle_left_motor ;
    if ( throttle_left_motor_memory < 0) {
      PORTD &= 0b10111111 ;
      throttle_left_motor_memory *= -1;
    }
    else PORTD |= 0b01000000 ;
  }
  else if ( throttle_counter_left_motor == 1) PORTB &= 0b11111110;
  else if ( throttle_counter_left_motor == 2) PORTB |= 0b00000001 ;
  // right motor pulse calculations
  throttle_counter_right_motor++;
  if ( throttle_counter_right_motor > throttle_right_motor_memory ) {
    throttle_counter_right_motor = 0;
    throttle_right_motor_memory = throttle_right_motor ;
    if ( throttle_right_motor_memory < 0) {
      PORTD |= 0b00100000 ;
      throttle_right_motor_memory *= -1;
    }
    else PORTD &= 0b11011111  ;
  }
  else if ( throttle_counter_right_motor == 1) PORTD &= 0b01111111 ;
  else if ( throttle_counter_right_motor == 2) PORTD |= 0b10000000 ;
}

void getRoot() {
  while (s.available() > 0)
  {
    received_byte = s.read();
  }
}
