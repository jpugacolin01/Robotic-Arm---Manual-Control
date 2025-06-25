#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// ===== PWM SERVO DRIVER CONFIGURATION =====
const uint8_t PWM_I2C_ADDRESS = 0x40;
const uint16_t SERVO_FREQ = 50;  // 50Hz for analog servos
const uint32_t OSCILLATOR_FREQ = 27000000;  // Internal oscillator frequency
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(PWM_I2C_ADDRESS, Wire);

// ===== SERVO PIN ASSIGNMENTS =====
const uint8_t PAN_PIN = 0;
const uint8_t TILT_PIN = 1;
const uint8_t ROLL_PIN = 2;
const uint8_t CRUNCH_PIN = 3;
const uint8_t TILT_PIN_2 = 4;
const uint8_t ROLL_PIN_2 = 5;

// ===== SERVO PULSE LENGTH LIMITS =====
struct ServoLimits {
  uint16_t min;
  uint16_t max;
};

const ServoLimits SERVO_LIMITS[] = {
  {107, 437},  // Pan servo limits
  {90, 380},   // Tilt servo limits
  {90, 356},   // Roll servo limits
  {0, 400},    // Crunch servo limits
  {90, 500},   // Tilt 2 servo limits
  {90, 500}    // Roll 2 servo limits
};

// ===== SERVO CONTROL PARAMETERS =====
const uint16_t SERVO_STEP = 10;  // Step size for servo movements
const uint16_t USMIN = 600;      // Minimum microsecond pulse length
const uint16_t USMAX = 2400;     // Maximum microsecond pulse length

// ===== SERVO POSITIONS =====
struct ServoPositions {
  int16_t pan = 90;
  int16_t tilt = 150;
  int16_t roll = 35;
  int16_t crunch = 90;
  int16_t tilt2 = 90;
  int16_t roll2 = 120;
};

ServoPositions servos;

// ===== FUNCTION DECLARATIONS =====
void initializeServos();
void setServoPosition(uint8_t pin, int16_t position);
void setServoPulse(uint8_t pin, double pulseSeconds);
void handleSerialInput();
void updateServoPosition(int16_t& position, int16_t delta, uint8_t servoIndex, uint8_t pin, const char* name);
void printServoPosition(const char* name, int16_t position);

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for native USB)
  }
  
  Serial.println("Multi-Servo Controller Initialized");
  initializeServos();
}

void loop() {
  handleSerialInput();
}

void initializeServos() {
  pwm.begin();
  pwm.setOscillatorFrequency(OSCILLATOR_FREQ);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  
  // Set initial positions
  setServoPosition(PAN_PIN, servos.pan);
  setServoPosition(TILT_PIN, servos.tilt);
  setServoPosition(ROLL_PIN, servos.roll);
  setServoPosition(CRUNCH_PIN, servos.crunch);
  setServoPosition(TILT_PIN_2, servos.tilt2);
  setServoPosition(ROLL_PIN_2, servos.roll2);
  
  Serial.println("Servos initialized to default positions");
}

void setServoPosition(uint8_t pin, int16_t position) {
  pwm.setPWM(pin, 0, position);
}

void setServoPulse(uint8_t pin, double pulseSeconds) {
  double pulseLength = 1000000.0 / SERVO_FREQ;  // Microseconds per PWM period
  pulseLength /= 4096.0;  // 12-bit resolution
  double pulse = pulseSeconds * 1000000.0;  // Convert to microseconds
  pulse /= pulseLength;  // Convert to PWM counts
  pwm.setPWM(pin, 0, (uint16_t)pulse);
}

void handleSerialInput() {
  if (Serial.available() > 0) {
    char input = Serial.read();
    
    switch (input) {
      // Pan (Yaw) control
      case 'a':
      case 'd':
        updateServoPosition(servos.pan, (input == 'a') ? SERVO_STEP : -SERVO_STEP, 
                          0, PAN_PIN, "Pan");
        break;
        
      // Tilt control
      case 'r':
      case 'f':
        updateServoPosition(servos.tilt, (input == 'r') ? SERVO_STEP : -SERVO_STEP, 
                          1, TILT_PIN, "Tilt");
        break;
        
      // Roll control
      case 'q':
      case 'e':
        updateServoPosition(servos.roll, (input == 'q') ? SERVO_STEP : -SERVO_STEP, 
                          2, ROLL_PIN, "Roll");
        break;
        
      // Crunch control
      case 'z':
      case 'c':
        updateServoPosition(servos.crunch, (input == 'z') ? SERVO_STEP : -SERVO_STEP, 
                          3, CRUNCH_PIN, "Crunch");
        break;
        
      // Tilt 2 control
      case 't':
      case 'g':
        updateServoPosition(servos.tilt2, (input == 't') ? SERVO_STEP : -SERVO_STEP, 
                          4, TILT_PIN_2, "Tilt2");
        break;
        
      // Roll 2 control
      case 'y':
      case 'h':
        updateServoPosition(servos.roll2, (input == 'y') ? SERVO_STEP : -SERVO_STEP, 
                          5, ROLL_PIN_2, "Roll2");
        break;
        
      // Center all servos
      case 'x':
        centerAllServos();
        break;
        
      // Print current positions
      case 'p':
        printAllPositions();
        break;
        
      // Help menu
      case '?':
        printHelpMenu();
        break;
        
      default:
        Serial.println("Unknown command. Press '?' for help.");
        break;
    }
  }
}

void updateServoPosition(int16_t& position, int16_t delta, uint8_t servoIndex, uint8_t pin, const char* name) {
  int16_t newPosition = position + delta;
  newPosition = constrain(newPosition, SERVO_LIMITS[servoIndex].min, SERVO_LIMITS[servoIndex].max);
  
  if (newPosition != position) {
    position = newPosition;
    setServoPosition(pin, position);
    printServoPosition(name, position);
  } else {
    Serial.print(name);
    Serial.println(" at limit!");
  }
}

void printServoPosition(const char* name, int16_t position) {
  Serial.print(name);
  Serial.print(" = ");
  Serial.println(position);
}

void centerAllServos() {
  Serial.println("Centering all servos...");
  
  // Calculate center positions for each servo
  servos.pan = (SERVO_LIMITS[0].min + SERVO_LIMITS[0].max) / 2;
  servos.tilt = (SERVO_LIMITS[1].min + SERVO_LIMITS[1].max) / 2;
  servos.roll = (SERVO_LIMITS[2].min + SERVO_LIMITS[2].max) / 2;
  servos.crunch = (SERVO_LIMITS[3].min + SERVO_LIMITS[3].max) / 2;
  servos.tilt2 = (SERVO_LIMITS[4].min + SERVO_LIMITS[4].max) / 2;
  servos.roll2 = (SERVO_LIMITS[5].min + SERVO_LIMITS[5].max) / 2;
  
  // Apply positions
  setServoPosition(PAN_PIN, servos.pan);
  setServoPosition(TILT_PIN, servos.tilt);
  setServoPosition(ROLL_PIN, servos.roll);
  setServoPosition(CRUNCH_PIN, servos.crunch);
  setServoPosition(TILT_PIN_2, servos.tilt2);
  setServoPosition(ROLL_PIN_2, servos.roll2);
  
  printAllPositions();
}

void printAllPositions() {
  Serial.println("=== Current Servo Positions ===");
  printServoPosition("Pan   ", servos.pan);
  printServoPosition("Tilt  ", servos.tilt);
  printServoPosition("Roll  ", servos.roll);
  printServoPosition("Crunch", servos.crunch);
  printServoPosition("Tilt2 ", servos.tilt2);
  printServoPosition("Roll2 ", servos.roll2);
  Serial.println("==============================");
}

void printHelpMenu() {
  Serial.println("=== Servo Control Commands ===");
  Serial.println("Pan:    a/d (left/right)");
  Serial.println("Tilt:   r/f (up/down)");
  Serial.println("Roll:   q/e (left/right)");
  Serial.println("Crunch: z/c (in/out)");
  Serial.println("Tilt2:  t/g (up/down)");
  Serial.println("Roll2:  y/h (left/right)");
  Serial.println("Center: x (center all)");
  Serial.println("Status: p (print positions)");
  Serial.println("Help:   ? (this menu)");
  Serial.println("==============================");
}
