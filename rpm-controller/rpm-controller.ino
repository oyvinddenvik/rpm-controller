

#define ENCODER_PULSE_PER_ROTATION (75*3)
#define ENCODER_OC_OUT_PIN 2

#define PWM_MOTOR_DRIVER 5
#define DIR_MOTOR_DRIVER 4



volatile unsigned long int encoderCounts = 0;

unsigned int intervalInMilliseconds = 1000;
unsigned long int previousMillis = 0;
unsigned long int currentMillis = 0;
int measuredRPM = 0;
int targetRPM = 30;

// Controller
//unsigned int targetRPM = 0;
long int previousTime = 0;
float eIntegral = 0;
float ePrevious = 0;

#define K_p 0.5
#define K_i 1.5
#define K_d 0


void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_OC_OUT_PIN, INPUT);
  pinMode(PWM_MOTOR_DRIVER, OUTPUT);
  pinMode(DIR_MOTOR_DRIVER, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_OC_OUT_PIN), updateEncoderCount, RISING);
}

void loop() {
  
  currentMillis = millis();
  
  if (currentMillis - previousMillis >= intervalInMilliseconds) {
    previousMillis = currentMillis; 


    noInterrupts();
    measuredRPM = (float)((encoderCounts*60)/ENCODER_PULSE_PER_ROTATION);
    encoderCounts = 0;
    interrupts();
    
    double eCurrent = targetRPM - measuredRPM;

    long currentTime = micros();

    float deltaTime = (float)((currentTime - previousTime)/1.0e6);
    previousTime = currentTime;

    // Derivative
    float dedt = (eCurrent - ePrevious)/deltaTime;

    // Integral
    eIntegral = eIntegral + (eCurrent*deltaTime);
    
    // Control signal
    float u = (K_p*eCurrent) + (K_i*eIntegral) + (K_d*dedt);

    float controlledPWMToMotor = fabs(u);
    if(controlledPWMToMotor > 255) {
      controlledPWMToMotor = 255;
    }

    setMotor(1,controlledPWMToMotor,PWM_MOTOR_DRIVER,DIR_MOTOR_DRIVER);

    ePrevious = eCurrent;

    Serial.print("Target RPM: ");
    Serial.print(targetRPM);
    Serial.print(" ");
    Serial.print("Measured RPM: ");
    Serial.print(measuredRPM);
    Serial.print(" ");
    /*
    Serial.print("Integral term: ");
    Serial.print(eIntegral);
    Serial.print(" ");
    Serial.print("Derivative term: ");
    Serial.print(dedt);
    Serial.print(" ");
    Serial.print("Error: ");
    Serial.print(eCurrent);
    Serial.print("Control signal u: ");
    Serial.print(u);
    */
    
   // Serial.print("  ");
    //Serial.print("Controlled PWM to motor: ");
    //Serial.print(controlledPWMToMotor);
    Serial.println();

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
