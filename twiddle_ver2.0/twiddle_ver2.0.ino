/*
 * 1h45
 *  1. Thay doi he so ho tu 1.1 => 1.2, 0.9=>0.8
 *  2. Hien thi cac tham so nhan len 1000
 *  3. Thay doi thoi gian moi vong for chay nhanh hon 3000 => 2500mcs
 *  4. Thay doi tai dong ~89 N=100 => N=300, tang thoi gian co the dap ung
 *  5. Thay doi   float  params[3]  = {10, 10, 10};
                  float dparams[3]  = {5, 5, 5};
 */

#include "MotorHung.h"
#include "EncoderH.h"
#include "configure.h"

#include <AutoPID.h>

MotorHung M_1(PWM1, DIR1);
Encoder E_1(CA1, CB1);

//Time
typedef struct TIME_ {
  float dt = 0;
  long lastTime = 0;
} time_;
time_ timeRunPID;

//Run
double setPoint;
double measure;
float err;
float int_crosstrack_error;
float N;
float crosstrack_error;
float diff_crosstrack_error;
double PWM;
int i;

//Twiddle
float best_error;
float errTwiddle;
int n; //So lan lap cua Twiddle
//PID vector
typedef struct PIDVECTOR {
  float Kp;
  float Ki;
  float Kd;
} PID_vector;

PID_vector PID_params  = {0, 0, 0};
PID_vector PID_dparams = {1, 1, 1};

float  params[3]  = {0, 0, 0};
float dparams[3]  = {1, 1, 1};

AutoPID Motor_1(&measure, &setPoint, &PWM, -255, 255, 1, 1, 1);

float bestOfTheBest = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Motor_1.setTimeStep(8);
  M_1.setSpeedMotor(100);
}

void loop() {

  //  Serial.print  ("E_velocity: " + (String)E_1.getVelocity() + "\t\t");
  //  Serial.println();

  //  PID_params = {1, 0.5, 0};
  //  params[0]  = 1;
  //  params[1]  = 0.5;
  //  params[2]  = 0;

  //  best_error = run(params);
  //  Serial.print  ("best_error: " + (String)best_error + "\t\t");
  //  Serial.println();

  bestOfTheBest = twiddle(0.002);
  Serial.println("xxxxxx :" + (String)bestOfTheBest);
  while (1);

}


float run(float _params[3]) {
  setPoint = 100;
  err = 0.0;                             //Tong gia tri loi de danh gia bo PID
  int_crosstrack_error = 0.0;            //Tong sai so tich phan
  //  i = 0;
  N = 300;                                //So lan lap cho moi bo PID, tang so nay len de co dap ung tot hon
  measure = E_1.getVelocity();                             //Do luong
  crosstrack_error = measure - setPoint;  //Sai so

  for (int i = 1; i <= 2 * N; i++) {
    measure = E_1.getVelocity();

    diff_crosstrack_error = measure - crosstrack_error; //Do luong
    crosstrack_error = measure - setPoint;                                         //Sai so
    int_crosstrack_error += crosstrack_error;

    PWM = _params[0] * crosstrack_error + _params[1] * int_crosstrack_error + _params[2] * diff_crosstrack_error ;

    PWM = constrain(PWM, -255, 255);

    M_1.setSpeedMotor(-PWM);

    if (i >= N) {
      err += (crosstrack_error * crosstrack_error);
    }
    while (((float)micros() - timeRunPID.lastTime) < 2500);
    timeRunPID.lastTime = micros();
//    Serial.print  ("i: " + (String)i + "\t\t");
//    Serial.print  ("measure: " + (String)measure + "\t\t");
//    Serial.print  ("PWM: " + (String)PWM + "\t\t");
//    Serial.println();
  }
  return err / float(N);
}

float twiddle(float tol) {

  float  params[3]  = {1.05, 0.04, -0.13};
  float dparams[3]  = {1, 1, 1};

  best_error = run(params);
  n = 0;
  while (sum(dparams) > tol) {
    for (int i = 0; i < 3; i++) {
      params[i] += dparams[i];
      errTwiddle = run(params);
      if (errTwiddle < best_error) {
        best_error = errTwiddle;
        dparams[i] *= 1.2;
      }
      else {
        params[i] -= 2 * dparams[i];
        errTwiddle = run(params);
        if (errTwiddle < best_error) {
          best_error = errTwiddle;
          dparams[i] *= 1.2;
        }
        else {
          params[i] += dparams[i];
          dparams[i] *= 0.8;
        }
      }
    }
    n += 1;
    Serial.print  ("n : " + (String)n + "\t\t");
    Serial.print  ("best_error : " + (String)best_error + "\t\t");
    Serial.print  ("params : " + (String) params[0] + "__" + (String) params[1] + "__" + (String) params[2] + "__" + "\t\t");
    Serial.print  ("dparams: " + (String)(1000*dparams[0]) + "__" + (String)(1000*dparams[1]) + "__" + (String)(1000*dparams[2]) + "__" + "\t\t");
    Serial.println();
  }
  return run(params);
}

float sum(float _params[3]) {
  return (_params[0] + _params[1] + _params[2]);
}



