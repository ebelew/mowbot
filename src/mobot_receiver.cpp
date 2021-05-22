/* i2c to Teensy 3.2  Clock on 19 White, Data on 18 Yellow, GREEN IS 3V3, RED IS GND!!!
*/

#include <Arduino.h>

#include <Wire.h>
#include <Servo.h>
#include <math.h>

#include "wiinunchuk.h"

#define DeadZone 16
//#define DeadZoneMin CenterX-(DeadZone)
//#define DeadZoneMax CenterY+(DeadZone)

#define CenterX 127
#define CenterY 128

#define MaxSpd 100

#define LeftSpeedPin 3
#define LeftBrakePin 4
#define LeftDirPin 14

#define RightSpeedPin 6
#define RightBrakePin 7
#define RightDirPin 17

// Attached motors
Servo ServoX;
Servo ServoY;

unsigned int Mode = 1;
unsigned int ServoXPos = 90; //Nominal center location
unsigned int ServoYPos = 90; //Nominal center location

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.print("USB COMMs Stack initialized :) \r\n");

  //Set up GPIOs
  pinMode(2, OUTPUT);            // Voltage
  digitalWrite(2, HIGH);

  pinMode(LeftBrakePin, OUTPUT); // Brake (High = on)
  pinMode(LeftDirPin, OUTPUT);   // Direction control bit.

  Serial.print("i2c interface to WiiChuck open at 800KHz \r\n");
  nunchuk_init();

  // Set these to fix calibration
  joy_x = CenterX;joy_y = CenterY;
  nunchuk_calibrate_joy();

  nunchuk_calibrate_accelxy();
  nunchuk_calibrate_accelz();

}

void diag_output()
{
  Serial.print("joy_zerox: ");
  Serial.print(joy_zerox);
  Serial.print(" joy_zeroy: ");
  Serial.print(joy_zeroy);
//  Serial.println();

  Serial.print("Joy X: ");
  Serial.print(nunchuk_joy_x());

  Serial.print("  Joy X (calibrated): ");
  Serial.print(nunchuk_cjoy_x());

  Serial.print("  Joy Y: ");
  Serial.print(nunchuk_joy_y());

  Serial.print("  Joy Y (calibrated): ");
  Serial.print(nunchuk_cjoy_y());

//  Serial.print("  Accel X: ");
//  Serial.print(nunchuk_accelx());
//
//  Serial.print("  Accel Y: ");
//  Serial.print(nunchuk_accely());
//
//  Serial.print("  Accel Z: ");
//  Serial.print(nunchuk_accelz());
//
  Serial.print("  Z Button: ");
  Serial.print(nunchuk_zbutton());

  Serial.print("  C Button: ");
  Serial.print(nunchuk_cbutton());

//  Serial.print(" Mode: ");
//  Serial.print(Mode);
//
//  Serial.print(" ServoXPos: ");
//  Serial.print(ServoXPos);
//
//  Serial.print(" ServoYPos: ");
//  Serial.print(ServoYPos);

  Serial.println();  
}

/* Source: https://robotics.stackexchange.com/questions/2011/how-to-calculate-the-right-and-left-speed-for-a-tank-like-rover
 *  theta = atan2(y, x)
 *  
    45º = π/4 = M_PI_4
    90º = π/2 = M_PI_2
    180º = π  = M_PI
    360º = 2π = M_2_PI 
returns a tuple of percentages: (left_thrust, right_thrust)
// Given:
//  X = adjacent
//  Y = opposite
//  Z = Hypotenuse = sqrt(x^2 + y^2) = throttle
//  theta = arcsin( Y / Z )          = turn angle
*/
struct spd_pair {
  float left_spd;
  float right_spd;
};

spd_pair chuk_pos_to_movement(int x, int y) {
    spd_pair retval;

    int r = sqrt( pow((double)x, 2.0) + pow((double)y, 2.0) );
    double theta = atan2( x , y ); 

    // falloff of main motor
    double v_a = r * (M_PI_4 - fmod(theta, M_PI_2)) / M_PI_4;
    // compensation
    double v_b = fmin((double)MaxSpd, fmin(2.0 * r + v_a, 2.0 * r - v_a));

    if ( theta < -M_PI_2 )  { 
        retval.left_spd = -v_b;
        retval.right_spd = -v_a;
    }
    else if ( theta < 0 ) {
        retval.left_spd = -v_a;
        retval.right_spd = v_b;
    }
    else if ( theta < M_PI_2 ) {
        retval.left_spd = v_b;
        retval.right_spd = v_a;
    }
    else {
        retval.left_spd = v_a;
        retval.right_spd = -v_b;
    }
    return retval;
}


void loop()
{

  delay(2);
  nunchuk_get_data();
  diag_output();

  spd_pair lr_speed;
  if (abs(nunchuk_cjoy_y ()) >= DeadZone || abs(nunchuk_cjoy_x() >= DeadZone))
  {
    lr_speed = chuk_pos_to_movement(nunchuk_cjoy_x(), nunchuk_cjoy_y());

    // Left
    if(abs(lr_speed.left_spd) >= DeadZone) {
        digitalWrite(LeftBrakePin, LOW); // Brakes Off
        digitalWrite(LeftDirPin, 0 < lr_speed.left_spd ? HIGH : LOW); // Forward for 0 < spd
        analogWrite(LeftSpeedPin, map(abs(lr_speed.left_spd), DeadZone, 128, 0, MaxSpd));
    }

    // Right
    if(abs(lr_speed.right_spd) >= DeadZone) {
        digitalWrite(RightBrakePin, LOW); // Brakes Off
        digitalWrite(RightDirPin, 0 < lr_speed.right_spd ? HIGH : LOW); // Forward for 0 < spd
        analogWrite(RightSpeedPin, map(abs(lr_speed.right_spd), DeadZone, 128, 0, MaxSpd));
    }

    return;
  }

  // Default to speed 0, brake on
  analogWrite(3, 0); 
  digitalWrite(4, HIGH);

        digitalWrite(LeftBrakePin, HIGH); // Brakes On
        analogWrite(LeftSpeedPin, 0);

        digitalWrite(RightBrakePin, HIGH); // Brakes On
        analogWrite(RightSpeedPin, 0);

}
