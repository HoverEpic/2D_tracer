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
  // revolution per minute
  int revPerMin;
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

  void setSpeed(int rpm) {
    revPerMin = rpm;
    // calculate step per second
    // calculate rev per second
    // calculate rev per minute
  }

  void setDelay(int delay) {
    delayMicro = delay;
  }

  int getSpeed() {
    return revPerMin;
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
