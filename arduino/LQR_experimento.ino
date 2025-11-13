/*
  ============================================
  UFMG DIA-Kit is placed under the MIT License
  Copyright (c) 2024 by GTI (UFMG)
  ============================================
  Hardware:
  * ESP32 WEMOS D1 R32 development board
  * 2 Nidec 24-H
  * 1 Module GY-521 MPU-6050
  * 1 Power Bank Box Charger, DC 12V output, 3x18650 batteries
*/

#include <math.h>

// I2C libray communication
#include <Wire.h>

// ENCODER library based on the built in counter hardware
#include <ESP32Encoder.h>

// ESP32 RED LED pin 
#define INTERNAL_LED 2

// IMU I2C address
#define MPU   0x68

// NIDEC PWM config
#define TIMER_BIT   8
#define BASE_FREQ   20000

// NIDEC pins: Reaction Wheel Motor
#define BRAKE1      18 //Yellow wire
#define PWM1        19 //Write wire 
#define DIR1        23 //Green wire
#define ENCA_1       5 //Brown wire // purple
#define ENCB_1      13 //Orange
#define PWM1_CH      0

#define BRAKE2      14 //Yellow wire
#define PWM2        27 //Write wire 
#define DIR2        16 //Green wire
#define ENCA_2      17 //Brown wire
#define ENCB_2      25 //Orange
#define PWM2_CH      1

// Encoder var
ESP32Encoder NIDEC1_ENC;
ESP32Encoder NIDEC2_ENC;

// Kalman Filter vars
float Q_angle = 0.001; // Angular data confidence
float Q_bias  = 0.005; // Angular velocity data confidence
float R_meas  = 1.0;
float pitch_angle = 0.0;
float bias = 0.0;
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float K[2] = {0, 0};

// Control vars
//       Th           th_dot        X             X_dot
// float K1 = -51,   K2 = -2.1,    K3 = -10,     K4 = -11;
float K1 = -53,   K2 = -3.5,    K3 = -3.1,     K4 = -9.3;


float theta = 0.0, theta_dot = 0.0;          // System states
float X = 0.0, X_dot = 0.0;
float X_erro = 0.0, X_erro_sum = 0.0;
float Psi = 0.0, Psi_dot = 0.0;
float Psi_erro = 0.0, Psi_erro_sum = 0.0;
float wheel1_speed = 0.0, wheel2_speed = 0.0;
float Ts = 0.01, currentT = 0.0, previousT = 0.0;      // Elapsed time in loop() function
float Tb = 0.1, currentTb = 0.0, previousTb = 0.0;
float count1 = 0, count2 = 0;
float U = 0;
int pwm = 0;
float disturbio = 0;
float Tc = 20, currentTc = 0.0, previousTc = 0.0;

float cnt2mps = 0.042/(Ts*67.3);

#define DADOSMAX 3000
float dados[DADOSMAX][7];
int count = 0;

// MAIN SETUP
void setup() { // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);      // make sure your Serial Monitor is also set at this baud rate.

  NIDECsetup();
  IMUsetup();
  
  pinMode(INTERNAL_LED,OUTPUT);
  digitalWrite(INTERNAL_LED,HIGH);  // Turn on red led
  delay(1000);             // Wait for the system to stabilize
  for (int i=1; i<= 200; i++){
    IMUread();
    delay(5);
  }
  currentT = millis();
}

// MAIN LOOP
void loop() {// put your main code here, to run repeatedly:
  equilibra();
  printa();
}

void equilibra() {
  
  currentT = millis();
   if ((currentT - previousT)/1000.0 >= Ts) {
    previousT = currentT;

    IMUread();
      
    if (abs(pitch_angle) < 40){
      digitalWrite(BRAKE1, HIGH);
      digitalWrite(BRAKE2, HIGH);

      theta += theta_dot * Ts;
      
      count1 += NIDEC1_ENC.getCount();
      wheel1_speed = NIDEC1_ENC.getCount();  //left wheel speed
      NIDEC1_ENC.clearCount();
    
      count2 -= NIDEC2_ENC.getCount();
      wheel2_speed = -NIDEC2_ENC.getCount();  //right wheel speed (oposite direction)
      NIDEC2_ENC.clearCount();

      X_dot = (wheel1_speed + wheel2_speed)*cnt2mps/2;
      X += X_dot*Ts;

      U = +K1*theta +K2*theta_dot +K3*X +K4*X_dot;

      if (currentT/1000.0 > 21) disturbio = 5;

      // if (currentT/1000.0 > 21 && currentT/1000.0 <21.5) disturbio = 5; // pulso
      // else disturbio = 0;

      pwm = (U + disturbio)*21.3;

      if (wheel1_speed==0 || wheel2_speed==0) pwm += random(20)-10; // sometimes the motor stops

      MOTOR1cmd(-pwm);
      MOTOR2cmd(pwm);

    } else {
      digitalWrite(BRAKE1, LOW); // stop reaction wheel
      digitalWrite(BRAKE2, LOW); // stop traction wheel
      digitalWrite(INTERNAL_LED,HIGH);  
      MOTOR1cmd(0);      // stop motor
      MOTOR2cmd(0);      // stop motor
      delay(6000);       // time to lift the robot
      for (int i=1; i<= 400; i++){ //Wait for the Kalman Filter stabilize
        IMUread();
        delay(5);
      }
      previousT = millis();
      theta = 0.0;
      NIDEC1_ENC.clearCount();
      NIDEC2_ENC.clearCount();
      X = 0.0;
      X_erro_sum = 0.0;
      count1 = 0;
      count2 = 0;
      digitalWrite(INTERNAL_LED,LOW);
    }

    if (currentT/1000.0 > 20) {
      dados[count][0] = currentT/1000.0;
      dados[count][1] = theta;
      dados[count][2] = theta_dot;
      dados[count][3] = X;
      dados[count][4] = X_dot;
      dados[count][5] = U;
      dados[count][6] = disturbio;
      if(count < DADOSMAX-1) count++;
    }


  }  

}

void printa(){
  currentTc = millis();
  if ((currentTc - previousTc)/1000.0 >= Tc && currentTc/1000.0 > 60) {
    previousTc = currentTc;

    for (int i = 0; i < DADOSMAX; i++) {
      for (int j = 0; j < 7; j++) { // Mostrando apenas os 10 primeiros elementos por linha
        Serial.print(dados[i][j], 5);
        if(j != 6) Serial.print(" ");      
      }
      Serial.println();
    }

    Serial.println("===");

  }
}

// SETUP functions
void IMUsetup(){
  // Initialize the MPU6050
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68)              
  Wire.write(0x6B);                      //make the reset (place a 0 into the 6B register)
  Wire.write(0);
  Wire.endTransmission(true);            //end the transmission
  //Gyro config
  Wire.beginTransmission(MPU);           //begin, Send the slave adress (in this case 68) 
  Wire.write(0x1B);                      //We want to write to the GYRO_CONFIG register (1B hex)
  // Wire.write(0x00000000);
  Wire.endTransmission();                //End the transmission with the gyro
  //Acc config
  Wire.beginTransmission(MPU);           //Start communication with the address found during search.
  Wire.write(0x1C);                      //We want to write to the ACCEL_CONFIG register
  Wire.write(0b00000000);                //Set the register bits as 00000000 (+/- 2g full scale range), 00010000 (+/- 8g full scale range)
  Wire.endTransmission(); 
}

void NIDECsetup(){
  pinMode(BRAKE1, OUTPUT);
  digitalWrite(BRAKE1, HIGH);

  pinMode(BRAKE2, OUTPUT);
  digitalWrite(BRAKE2, HIGH);
  
  pinMode(DIR1, OUTPUT);
  ledcSetup(PWM1_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM1, PWM1_CH);
  MOTOR1cmd(0);
  
  pinMode(DIR2, OUTPUT);
  ledcSetup(PWM2_CH, BASE_FREQ, TIMER_BIT);
  ledcAttachPin(PWM2, PWM2_CH);
  MOTOR2cmd(0);

  //s Encoder setup
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
	NIDEC1_ENC.attachFullQuad(ENCA_1, ENCB_1);
  NIDEC1_ENC.clearCount();

  NIDEC2_ENC.attachFullQuad(ENCA_2, ENCB_2);
  NIDEC2_ENC.clearCount();

}

// IMU function: Kalman Filter
void IMUread(){
  // read IMU
  int16_t ax,ay,az,temp,gx,gy,gz;
  Wire.beginTransmission(MPU);    // IMU address: 0x68
  Wire.write(0x3B);  
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14);        // IMU address: 0x68
  ax=Wire.read()<<8|Wire.read();   // X-axis value: 16384.0; 
  ay=Wire.read()<<8|Wire.read();   // Y-axis value: 16384.0;     
  az=Wire.read()<<8|Wire.read();   // Z-axis value: 16384.0;  
  temp=Wire.read()<<8|Wire.read();      
  gx=Wire.read()<<8|Wire.read();  
  gy=Wire.read()<<8|Wire.read();  
  gz=Wire.read()<<8|Wire.read();  
  //accelerometer angles in degrees (or rads)
  //float ax_angle = atan2(ay, sqrt(ax*ax + az*az)) * 57.3; // roll
  float ay_angle = atan2(-ax, sqrt(ay*ay + az*az)) * 57.3; // pitch
  // float az_angle = atan2(sqrt(ax*ax + az*az), az) * 57.3; // yaw (useless)
  // gyro measurements in degress (or rads)
  // gx =  gx / 65.5; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  gy =  gy / 131; //* 0.0174533; // Datasheet Sensitivity Scale Factor: 131, 65.5, 32.8, 16.4 for degrees/sec and Scale pi/180 = 0.0174533 for rad/sec
  
  // begin: Kalman filter - Pitch Axis (Y)
  pitch_angle += (gy - bias) * Ts;
  
  P[0][0] += (Q_angle - P[0][1] - P[1][0]) * Ts;
  P[0][1] += -P[1][1] * Ts;
  P[1][0] += -P[1][1] * Ts;
  P[1][1] += Q_bias * Ts;
  //
  K[0] = P[0][0] / (P[0][0] + R_meas);
  K[1] = P[1][0] / (P[0][0] + R_meas);  
  //
  pitch_angle += K[0] * (ay_angle - pitch_angle); 
  bias       += K[1] * (ay_angle - pitch_angle);
  //
  float P00_temp = P[0][0];
  float P01_temp = P[0][1];

  P[0][0] -= K[0] * P00_temp;
  P[0][1] -= K[0] * P01_temp;
  P[1][0] -= K[1] * P00_temp;
  P[1][1] -= K[1] * P01_temp;
  // end: Kalman filter 

  theta_dot = (gy - bias)/57.3; // Unbiased gyro speed

}

// NIDEC functions
void MOTOR1cmd(int sp) {
  if (sp < 0) {
    digitalWrite(DIR1, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR1, HIGH);
  }
  ledcWrite(PWM1_CH, int(sp > 255 ? 0 : 255 - sp));
  digitalWrite(BRAKE1, HIGH);
}

void MOTOR2cmd(int sp) {
  if (sp < 0) {
    digitalWrite(DIR2, LOW);
    sp = -sp;
  } else {
    digitalWrite(DIR2, HIGH);
  }
  ledcWrite(PWM2_CH, int(sp > 255 ? 0 : 255 - sp));
  digitalWrite(BRAKE2, HIGH);
}
