

#define ENCODER_PULSE_PER_ROTATION (75*3)
#define ENCODER_OC_OUT_PIN 2

#define PWM_MOTOR_DRIVER 5
#define DIR_MOTOR_DRIVER 4



unsigned int baudrate = 115200;

volatile unsigned long int encoderCounts = 0;

unsigned int intervalInMilliseconds = 1000;
unsigned long int previousMillis = 0;
unsigned long int currentMillis = 0;
unsigned int rpm = 0;






void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_OC_OUT_PIN, INPUT);
  pinMode(PWM_MOTOR_DRIVER, OUTPUT);
  pinMode(DIR_MOTOR_DRIVER, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_OC_OUT_PIN), updateEncoderCount, RISING);
  setMotor(-1,100,PWM_MOTOR_DRIVER,DIR_MOTOR_DRIVER);
}

void loop() {
  
  currentMillis = millis();
  
  if (currentMillis - previousMillis >= intervalInMilliseconds) {
    previousMillis = currentMillis; 


    noInterrupts();
    rpm = (float)((encoderCounts*60)/ENCODER_PULSE_PER_ROTATION);
    encoderCounts = 0;
    interrupts();
    Serial.print("RPM: ");
    Serial.println(rpm);
  }
  

}

void setMotor(int dirValue, int pwmValue, int pwmPin, int dirPin) {
  
  if (dirValue == 1) {
    digitalWrite(dirPin, LOW); // Clockwise
    analogWrite(pwmPin, pwmValue);
  }
  else if (dirValue == -1) {
    digitalWrite(dirPin, HIGH); // Counter clockwise
    analogWrite(pwmPin, pwmValue);
  }
  else {
    Serial.print("Please set dirValue either as 1 og -1");
  }
}


void updateEncoderCount(){
  encoderCounts++;
}
