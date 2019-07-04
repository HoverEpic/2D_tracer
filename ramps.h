struct Motor {

  // use https://reprap.org/wiki/RAMPS_1.4#Firmware_and_Pin_Assignments

  // set to low for activate stepper
  int enablePin;
  // move
  int stepPin;
  int dirPin;
  // steps per revolution, usually 200 (1.8° per step)
  int stepsPerRev = 200;
  // delay between moves, default 100
  int delayMicro = 100;
  // Steps per millimeter, depends on transmition
  float steps_per_millimeter = 50;
  // case of reversed connections
  boolean reversed = false;
  // pin to listen endstops
  int minEndStopPin;
  int maxEndStopPin;

  Motor(int p_EnablePin, int p_StepPin, int p_DirPin, int p_minEndStopPin, int p_maxEndStopPin) {
    enablePin = p_EnablePin;
    stepPin = p_StepPin;
    dirPin = p_DirPin;
    minEndStopPin = p_minEndStopPin;
    maxEndStopPin = p_maxEndStopPin;
    init();
  }

  void init() {
    pinMode(enablePin, OUTPUT);
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    pinMode(minEndStopPin, INPUT);
    pinMode(maxEndStopPin, INPUT);
  }

  void enable() {
    digitalWrite(enablePin, LOW);
  }

  void disable() {
    digitalWrite(enablePin, HIGH);
  }

  void reverse(boolean p_Reversed) {
    reversed = p_Reversed;
  }

  void setStepsPerMillimeter(float steps_per_millimeter) {
    this->steps_per_millimeter = steps_per_millimeter;
  }

  // mm/min
  void setSpeed(int mm_per_minute) {
    delayMicro = (1000 / (steps_per_millimeter / 2000 / 60))/mm_per_minute;
  }

  // µs
  void setDelay(int delay) {
    delayMicro = delay;
  }

  // mm/min
  float getSpeed() {
    return (1000 / (steps_per_millimeter / 2000 / 60))/delayMicro;
  }

  boolean isMinEndStopped() {
    return digitalRead(minEndStopPin);
  }

  boolean isMaxEndStopped() {
    return digitalRead(maxEndStopPin);
  }

  void step(long steps, boolean forward) {
    digitalWrite(dirPin, forward xor reversed ? HIGH : LOW);
    for (long x = 0; x < steps; x++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(delayMicro);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(delayMicro);
    }
  }

  void onestep(int dir) {
    digitalWrite(dirPin, dir > 0 xor reversed ? HIGH : LOW);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delayMicro);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delayMicro);
  }
};
