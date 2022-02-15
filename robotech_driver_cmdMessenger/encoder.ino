void setupEncoders() {

  right_count = 0;
  left_count = 0;
  prev_right_count = 0;
  prev_left_count = 0;

  pinMode(right_encoderA, INPUT);
  pinMode(right_encoderB, INPUT);
  pinMode(left_encoderA, INPUT);
  pinMode(left_encoderB, INPUT);

  digitalWrite(right_encoderA, LOW);
  digitalWrite(right_encoderB, LOW);
  digitalWrite(left_encoderA, LOW);
  digitalWrite(left_encoderB, LOW);

  attachInterrupt(digitalPinToInterrupt(right_encoderA), encoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(right_encoderB), encoder1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoderA), encoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(left_encoderB), encoder2B, CHANGE);
}

void encoder1A() {
  if (digitalRead(right_encoderA) == digitalRead(right_encoderB)) {
    right_count = right_count - 1;
  }
  else                                                     {
    right_count = right_count + 1;
  }
}
void encoder1B() {
  if (digitalRead(right_encoderA) == digitalRead(right_encoderB)) {
    right_count = right_count + 1;
  }
  else                                                     {
    right_count = right_count - 1;
  }
}

void encoder2A() {
  if (digitalRead(left_encoderA) == digitalRead(left_encoderB)) {
    left_count = left_count - 1;
  }
  else                                                     {
    left_count = left_count + 1;
  }
}
void encoder2B() {
  if (digitalRead(left_encoderA) == digitalRead(left_encoderB)) {
    left_count = left_count + 1;
  }
  else                                                     {
    left_count = left_count - 1;
  }
}
