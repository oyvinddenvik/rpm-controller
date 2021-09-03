

#define ENCODER_PULSE_PER_ROTATION (75*3)
#define ENCODER_OC_OUT_PIN 2

#define PWM_MOTOR_DRIVER 6
#define DIR_MOTOR_DRIVER 5

#define BUTTON_PIN 3


// Encoder
volatile unsigned long int encoderCounts = 0;

unsigned int intervalInMilliseconds = 1000;
unsigned long int previousMillis = 0;
unsigned long int currentMillis = 0;


// Controller
int measuredRPM = 0;
int targetRPM = 40;
long int previousTime = 0;
float eIntegral = 0;
float ePrevious = 0;

#define K_p 0.1
#define K_i 1.5
#define K_d 0.0001

// Button
int buttonState = 0;
int previousButtonState = 0;
int buttonCounter = 0;

void setup() {
  Serial.begin(115200);
  pinMode(ENCODER_OC_OUT_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT); 
  pinMode(PWM_MOTOR_DRIVER, OUTPUT);
  pinMode(DIR_MOTOR_DRIVER, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_OC_OUT_PIN), updateEncoderCount, RISING);
  
  // Store button state
  buttonState = digitalRead(BUTTON_PIN);
  previousButtonState = buttonState;
}

void loop() {
    
    buttonState = digitalRead(BUTTON_PIN);

     if (buttonState != previousButtonState) {
        buttonCounter++;
        delay(1000);
      }
  
    previousButtonState = buttonState;

    
    if (buttonCounter == 1) {
      targetRPM = 30;
 
    } else if (buttonCounter == 2) {
      targetRPM  = 40;
      
    } else {
      targetRPM = 0;
      setMotor(1,0,PWM_MOTOR_DRIVER,DIR_MOTOR_DRIVER);
      buttonCounter = 0;
    } 
    
        currentMillis = millis();
    
        if (currentMillis - previousMillis >= intervalInMilliseconds) {
        previousMillis = currentMillis; 
  
  
        noInterrupts();
        measuredRPM = (float)((encoderCounts*60)/ENCODER_PULSE_PER_ROTATION);
        encoderCounts = 0;
        interrupts();

      
        double eCurrent = targetRPM- measuredRPM;
    
        long currentTime = micros();
    
        float deltaTime = (float)((currentTime - previousTime)/1.0e6);
        previousTime = currentTime;
    
        // Derivative
        float dedt = (eCurrent - ePrevious)/deltaTime;
    
        // Integral
        eIntegral = eIntegral + (eCurrent*deltaTime);
      
        if (buttonCounter == 0){
          eIntegral = 0;
          dedt = 0;
          deltaTime = 0;
          eCurrent = 0;
        }
        
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
