void setupMotors() {
  desire_rpm_right = 0;
  desire_rpm_left = 0;
  actual_rpm_right = 0;
  actual_rpm_left = 0;

  right_pwm = 0;
  left_pwm = 0;
  Release();
}
void getMotorData(unsigned long time_)  {
  actual_rpm_right = double((right_count - prev_right_count) * 60000) / double(time_ * enc_ticks);
  actual_rpm_left = double((left_count - prev_left_count) * 60000) / double(time_ * enc_ticks);

  prev_right_count = right_count;
  prev_left_count = left_count;
}
int updatePid(int id, int old_pwm, double targetValue, double currentValue,float dt) 
{
  double pidTerm = 0; 
  double error = 0;
  double new_pwm = 0;
  static double prev_right_err  = 0;
  static double prev_left_err  = 0;
  static double right_integral_err  = 0;
  static double left_integral_err  = 0;

  error = targetValue - currentValue;
  
  if (id == 1) 
  { 
    right_integral_err  += (error*dt);
    pidTerm = RKp * error + RKi * right_integral_err + RKd * ( (error - prev_right_err )/dt );
    prev_right_err  = error;
  }
  if (id == 2) 
  {
    left_integral_err  += (error*dt);
    pidTerm = LKp * error + LKi * left_integral_err + LKd * ( (error - prev_left_err )/dt );
    prev_left_err  = error;
  }
  new_pwm = constrain(double(old_pwm) + pidTerm, -255, 255); 
  return int(new_pwm);
}

void Forward() {
  digitalWrite(Right_in1, LOW);
  digitalWrite(Right_in2, HIGH);
  digitalWrite(Left_in1, LOW);
  digitalWrite(Left_in2, HIGH);
  analogWrite(ENA1, abs(right_pwm));
  analogWrite(ENA2, abs(left_pwm));
}

void Backward() {
  digitalWrite(Right_in1, HIGH);
  digitalWrite(Right_in2, LOW);
  digitalWrite(Left_in1, HIGH);
  digitalWrite(Left_in2, LOW);
  analogWrite(ENA1, abs(right_pwm));
  analogWrite(ENA2, abs(left_pwm));
}

void Left() {
  digitalWrite(Right_in1, HIGH);
  digitalWrite(Right_in2, LOW);
  digitalWrite(Left_in1, LOW);
  digitalWrite(Left_in2, HIGH);
  analogWrite(ENA1, abs(right_pwm));
  analogWrite(ENA2, abs(left_pwm));
}

void Right() {
  digitalWrite(Right_in1, LOW);
  digitalWrite(Right_in2, HIGH);
  digitalWrite(Left_in1, HIGH);
  digitalWrite(Left_in2, LOW);
  analogWrite(ENA1, abs(right_pwm));
  analogWrite(ENA2, abs(left_pwm));
}

void Release() {
  digitalWrite(Right_in1, LOW);
  digitalWrite(Right_in2, LOW);
  digitalWrite(Left_in1, LOW);
  digitalWrite(Left_in2, LOW);
  analogWrite(ENA1, 0);
  analogWrite(ENA2, 0);
}
//void setPWM() {
  //analogWrite(ENA1, abs(right_pwm));
  //analogWrite(ENA2, abs(left_pwm));
//}
