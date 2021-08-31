

#define ENCODER_PULSE_PER_ROTATION (75*3)
#define ENCODER_OC_OUT_PIN 2



unsigned int baudrate = 115200;

volatile unsigned long int encoderCounts = 0;

unsigned int intervalInMilliseconds = 1000;
unsigned long int previousMillis = 0;
unsigned long int currentMillis = 0;
unsigned int rpm = 0;


void updateEncoderCount(){
  encoderCounts++;
}



void setup() {
  Serial.begin(baudrate);
  pinMode(ENCODER_OC_OUT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCODER_OC_OUT_PIN), updateEncoderCount, RISING);
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
