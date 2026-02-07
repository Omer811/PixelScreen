// testing a stepper motor with a Pololu A4988 driver board
// on an Uno the onboard led will flash with each step
// as posted on Arduino Forum at http://forum.arduino.cc/index.php?topic=208905.0

byte directionPin = 9;
byte stepPin = 8;
int numberOfSteps = 5000;
byte ledPin = 13;
int pulseWidthMicros = 50;  // microseconds
int millisbetweenSteps = 50; // milliseconds

void setup() 
{ 

  Serial.begin(9600);
  Serial.println("Starting StepperTest");
  digitalWrite(ledPin, LOW);
  
  delay(2000);

  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  
 
  digitalWrite(directionPin, HIGH);
  for(int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepPin, LOW);
    
    delay(millisbetweenSteps);
    
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
  
  delay(3000);
  

  digitalWrite(directionPin, LOW);
  for(int n = 0; n < numberOfSteps; n++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseWidthMicros);
    digitalWrite(stepPin, LOW);
    
    delay(millisbetweenSteps);
    
    digitalWrite(ledPin, !digitalRead(ledPin));
  }
  
}

void loop() 
{ 

}