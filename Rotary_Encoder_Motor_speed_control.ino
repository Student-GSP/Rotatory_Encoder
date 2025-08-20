// DC Motor Control with Rotary Encoder and L298N Driver
// Rotary encoder controls speed (CW = increase, CCW = decrease)
// Button press reverses motor direction

// L298N Motor Driver Pins
#define ENA 9     // Enable pin (PWM) for motor speed control
#define IN1 7     // Motor direction pin 1
#define IN2 8     // Motor direction pin 2

// Rotary Encoder Pins
#define CLK 2     // Encoder CLK pin (must be interrupt pin)
#define DT 3      // Encoder DT pin (must be interrupt pin)
#define SW 5      // Encoder switch pin

// Variables
volatile int encoderPos = 0;    // Encoder position counter
int lastCLK = HIGH;             // Last CLK state
int motorSpeed = 0;             // Motor speed (0-255)
bool motorDirection = true;     // true = forward, false = reverse
bool lastSwitchState = HIGH;    // Last switch state
bool switchPressed = false;     // Switch press flag

// Debounce variables
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 50;
bool switchState = false;
bool lastStableSwitchState = false;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  
  // Set pin modes
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(CLK, INPUT_PULLUP);
  pinMode(DT, INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  
  // Attach interrupts for encoder
  //attachInterrupt(digitalPinToInterrupt(CLK), readEncoder, CHANGE);
  
  // Initialize motor (stopped)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);
  
  // Read initial states
  lastCLK = digitalRead(CLK);
  lastSwitchState = digitalRead(SW);
  
  Serial.println("DC Motor Control Ready");
  Serial.println("Rotate encoder: CW = increase speed, CCW = decrease speed");
  Serial.println("Press button: Reverse direction");
  
  // Test switch at startup
  Serial.print("Switch initial state: ");
  Serial.println(digitalRead(SW) == HIGH ? "Released" : "Pressed");
}

void loop() {
  // Handle switch press for direction change
  readEncoderNoInterrupt();
  handleSwitchPress();
  
  // Update motor control based on encoder position
  updateMotorControl();
  
  // Print status every 500ms
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) {
    printStatus();
    lastPrint = millis();
  }
  
  delay(10); // Small delay for stability
}
/*
void readEncoder() {
  // Read current state of CLK and DT
  int currentCLK = digitalRead(CLK);
  int currentDT = digitalRead(DT);
  
  // If CLK state has changed
  if (currentCLK != lastCLK) {
    // Check direction based on DT state
    if (currentCLK == LOW) {
      if (currentDT == HIGH) {
        encoderPos--; // Clockwise
      } else {
        encoderPos++; // Counter-clockwise
      }
    }
  }
  lastCLK = currentCLK;
}*/

void handleSwitchPress() {
  int switchReading = digitalRead(SW);
  
  // If switch reading changed, reset debounce timer
  if (switchReading != lastSwitchState) {
    lastDebounceTime = millis();
  }
  
  // If enough time has passed, check for actual state change
  if ((millis() - lastDebounceTime) > debounceDelay) {
    // Update stable switch state
    switchState = (switchReading == LOW);
    
    // Check for press (transition from not pressed to pressed)
    if (switchState && !lastStableSwitchState) {
      // Switch was just pressed
      motorDirection = !motorDirection;
      Serial.println("* DIRECTION REVERSED! *");
      Serial.print("New direction: ");
      Serial.println(motorDirection ? "Forward" : "Reverse");
      
      // Add a brief delay to prevent multiple triggers
      delay(200);
    }
    
    lastStableSwitchState = switchState;
  }
  
  lastSwitchState = switchReading;
}

void updateMotorControl() {
  // Convert encoder position to motor speed (0-255)
  // Limit encoder position to reasonable range
  encoderPos = constrain(encoderPos, 0, 255);
  
  motorSpeed = encoderPos;
  
  // Set motor direction
  if (motorDirection) {
    // Forward direction
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    // Reverse direction
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  
  // Set motor speed (PWM)
  analogWrite(ENA, motorSpeed);
}

void printStatus() {
  Serial.print("Speed: ");
  Serial.print(motorSpeed);
  Serial.print(" | Direction: ");
  Serial.print(motorDirection ? "Forward" : "Reverse");
  Serial.print(" | Encoder: ");
  Serial.print(encoderPos);
  Serial.print(" | Switch: ");
  Serial.println(digitalRead(SW) == HIGH ? "Released" : "Pressed");
}

// Alternative version without interrupts (if needed)

void readEncoderNoInterrupt() {
  int currentCLK = digitalRead(CLK);
  
  if (currentCLK != lastCLK && currentCLK==1 ) {
    if (digitalRead(DT) == currentCLK  ) {
      encoderPos++; // Clockwise
    } else {
      encoderPos--; // Counter-clockwise
    }
  }
  lastCLK = currentCLK;
}