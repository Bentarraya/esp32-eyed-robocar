#include <DabbleESP32.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>

// Pin motor TB6612
#define AIN1 14
#define AIN2 12
#define BIN1 18
#define BIN2 19
#define PWMA 27
#define PWMB 33
#define STBY 13

// Infrared sensor (mengganti HC-SR04)
#define IR_SENSOR_PIN 4

// OLED Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// RoboEyes instance
RoboEyes<Adafruit_SSD1306> roboEyes(display);

// PWM
const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int PWM_CH_A = 0;
const int PWM_CH_B = 1;

// Speeds
int BASE_SPEED = 180;
int MANUAL_SPEED = 200;
int BACK_SPEED = 150;
int TURN_SPEED = 150;

// Trim
int TRIM_A = 0; //-35
int TRIM_B = 0;

// Infrared sensor
bool lastObstacleDetected = false;
unsigned long obstacleStartTime = 0;
const unsigned long OBSTACLE_CONFIRM_TIME = 50; // ms untuk konfirmasi obstacle
unsigned long lastConfusedAnim = 0;
const unsigned long CONFUSED_COOLDOWN = 1500; // Cooldown animasi confused

// Mode
bool modeManual = true;
unsigned long lastModeToggle = 0;
const unsigned long MODE_DEBOUNCE = 250;

// Autonomous state machine
enum AutoState { AUTO_FORWARD, AUTO_TURN_LEFT, AUTO_TURN_RIGHT, AUTO_BACKUP, AUTO_PAUSE, AUTO_SCAN };
AutoState astate = AUTO_FORWARD;
unsigned long stateUntil = 0;
int scanStep = 0;

// Eye state tracking
enum EyeDirection { EYE_FORWARD, EYE_LEFT, EYE_RIGHT, EYE_UP, EYE_CONFUSED };
EyeDirection currentEyeDirection = EYE_FORWARD;
EyeDirection lastEyeDirection = EYE_FORWARD;
unsigned long lastEyeUpdate = 0;
const unsigned long EYE_UPDATE_INTERVAL = 50;

// Forward declarations
bool checkObstacle();
void updateEyesBasedOnMovement();
void setEyeDirection(EyeDirection dir);
void handleObstacleAnimation();
void emergencyBrakeAndReverse();
bool forward_with_safety(int pwm);
void motorA_forward(int speed);
void motorA_backward(int speed);
void motorB_forward(int speed);
void motorB_backward(int speed);
void stopAll();
void forward_raw(int speed);
void backward_raw(int speed);
void turnLeft_raw(int speed);
void turnRight_raw(int speed);
void manualControl();
void autonomousStep();
void handleModeToggle();
void printDebugInfo();

// -------------------- SETUP --------------------
void setup() {
  Serial.begin(115200);

  // Setup motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  // Setup IR sensor
  pinMode(IR_SENSOR_PIN, INPUT);

  // Setup PWM
  ledcSetup(PWM_CH_A, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMA, PWM_CH_A);
  ledcSetup(PWM_CH_B, PWM_FREQ, PWM_RES);
  ledcAttachPin(PWMB, PWM_CH_B);

  digitalWrite(STBY, HIGH);

  // Initialize Dabble
  Dabble.begin("ESP32_RC");

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.setRotation(2);
  display.clearDisplay();
  display.display();

  // Initialize RoboEyes
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100);
  
  // Configure eyes behavior
  roboEyes.setAutoblinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  
  // Set eye size and shape
  roboEyes.setWidth(36, 36);
  roboEyes.setHeight(36, 36);
  roboEyes.setBorderradius(15, 15);
  roboEyes.setSpacebetween(10);
  
  // Default to forward looking
  roboEyes.setPosition(DEFAULT);
  roboEyes.setMood(DEFAULT);

  randomSeed(analogRead(0));
  astate = AUTO_FORWARD;
  stateUntil = millis() + random(800, 2000);

  Serial.println("READY: Select toggles Manual/Auto | X=MAJU | O=MUNDUR | Left/Right steer");
  Serial.println("Eyes system initialized");
}

// -------------------- IR SENSOR --------------------
bool checkObstacle() {
  // IR sensor aktif LOW saat mendeteksi obstacle
  return digitalRead(IR_SENSOR_PIN) == LOW;
}

// -------------------- EYE ANIMATION CONTROL --------------------
void setEyeDirection(EyeDirection dir) {
  if (dir == currentEyeDirection) return;
  
  currentEyeDirection = dir;
  
  switch(dir) {
    case EYE_FORWARD:
      roboEyes.setPosition(DEFAULT);
      roboEyes.setMood(DEFAULT);
      roboEyes.setCuriosity(OFF);
      break;
      
    case EYE_LEFT:
      roboEyes.setPosition(E);  // Look West/left
      roboEyes.setCuriosity(ON);
      break;
      
    case EYE_RIGHT:
      roboEyes.setPosition(W);  // Look East/right
      roboEyes.setCuriosity(ON);
      break;
      
    case EYE_UP:
      // Untuk mundur, mata melihat ke atas
      roboEyes.setPosition(N);  // Look North/up
      roboEyes.setCuriosity(OFF);
      break;
      
    case EYE_CONFUSED:
      // Animasi confused akan di-trigger di handleObstacleAnimation
      break;
  }
}

void updateEyesBasedOnMovement() {
  unsigned long now = millis();
  if (now - lastEyeUpdate < EYE_UPDATE_INTERVAL) return;
  lastEyeUpdate = now;

  // Cek obstacle terlebih dahulu
  if (checkObstacle()) {
    handleObstacleAnimation();
    return;
  }

  // Update mata berdasarkan input manual atau autonomous
  if (modeManual) {
    bool X = GamePad.isCrossPressed();
    bool O = GamePad.isCirclePressed();
    bool L = GamePad.isLeftPressed();
    bool R = GamePad.isRightPressed();
    
    if (X && L && !R) {
      // Maju belok kiri (arc left)
      setEyeDirection(EYE_LEFT);
    } else if (X && R && !L) {
      // Maju belok kanan (arc right)
      setEyeDirection(EYE_RIGHT);
    } else if (X) {
      // Maju lurus
      setEyeDirection(EYE_FORWARD);
    } else if (O) {
      // Mundur
      setEyeDirection(EYE_UP);
    } else if (L && !R && !X && !O) {
      // Putar kiri di tempat
      setEyeDirection(EYE_LEFT);
    } else if (R && !L && !X && !O) {
      // Putar kanan di tempat
      setEyeDirection(EYE_RIGHT);
    } else {
      // Diam
      setEyeDirection(EYE_FORWARD);
    }
  } else {
    // Autonomous mode
    switch(astate) {
      case AUTO_FORWARD:
        setEyeDirection(EYE_FORWARD);
        break;
      case AUTO_TURN_LEFT:
      case AUTO_SCAN:
        if (scanStep % 4 == 0) setEyeDirection(EYE_LEFT);
        else setEyeDirection(EYE_FORWARD);
        break;
      case AUTO_TURN_RIGHT:
        setEyeDirection(EYE_RIGHT);
        break;
      case AUTO_BACKUP:
        setEyeDirection(EYE_UP);
        break;
      case AUTO_PAUSE:
        setEyeDirection(EYE_FORWARD);
        break;
    }
  }
  
  // Update mata
  roboEyes.update();
}

void handleObstacleAnimation() {
  unsigned long now = millis();
  
  // Cek cooldown
  if (now - lastConfusedAnim < CONFUSED_COOLDOWN) {
    roboEyes.update();
    return;
  }
  
  // Trigger animasi confused
  roboEyes.anim_confused();
  roboEyes.setMood(ANGRY);
  lastConfusedAnim = now;
  currentEyeDirection = EYE_CONFUSED;
  
  Serial.println("OBSTACLE DETECTED: Playing confused animation");
}

// -------------------- MOTOR PRIMITIVES --------------------
void motorA_forward(int speed) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  ledcWrite(PWM_CH_A, constrain(speed + TRIM_A, 0, 255));
}

void motorA_backward(int speed) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  ledcWrite(PWM_CH_A, constrain(speed + TRIM_A, 0, 255));
}

void motorB_forward(int speed) {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(PWM_CH_B, constrain(speed + TRIM_B, 0, 255));
}

void motorB_backward(int speed) {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWM_CH_B, constrain(speed + TRIM_B, 0, 255));
}

void stopAll() {
  ledcWrite(PWM_CH_A, 0);
  ledcWrite(PWM_CH_B, 0);
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

void forward_raw(int speed) {
  motorA_forward(speed);
  motorB_forward(speed);
}

void backward_raw(int speed) {
  motorA_backward(speed);
  motorB_backward(speed);
}

void turnLeft_raw(int speed) {
  motorA_forward(speed);
  motorB_backward(speed);
}

void turnRight_raw(int speed) {
  motorA_backward(speed);
  motorB_forward(speed);
}

// -------------------- SAFETY --------------------
void emergencyBrakeAndReverse() {
  Serial.println("EMERGENCY: BRAKE + REVERSE BURST");
  
  // Immediate brake
  stopAll();
  delay(30);

  // Reverse burst
  backward_raw(BACK_SPEED);
  delay(180);

  // Final stop
  stopAll();
  delay(80);
}

bool forward_with_safety(int pwm) {
  // Check obstacle using IR sensor
  if (checkObstacle()) {
    emergencyBrakeAndReverse();
    return false;
  }
  
  forward_raw(pwm);
  return true;
}

// -------------------- MANUAL CONTROL --------------------
void manualControl() {
  bool X = GamePad.isCrossPressed();
  bool O = GamePad.isCirclePressed();
  bool L = GamePad.isLeftPressed();
  bool R = GamePad.isRightPressed();

  // Safety: check obstacle for forward movement
  if (X) {
    if (L && !R) {
      // Arc left
      int arcPWM = (int)(MANUAL_SPEED * 0.75);
      if (!forward_with_safety(arcPWM)) return;
      motorA_forward(arcPWM);
      motorB_forward((int)(arcPWM * 0.5));
      return;
    } else if (R && !L) {
      // Arc right
      int arcPWM = (int)(MANUAL_SPEED * 0.75);
      if (!forward_with_safety(arcPWM)) return;
      motorA_forward((int)(arcPWM * 0.5));
      motorB_forward(arcPWM);
      return;
    } else {
      // Straight forward with safety
      forward_with_safety(MANUAL_SPEED);
      return;
    }
  }

  if (O) {
    // Backward (no obstacle check for backward)
    backward_raw(MANUAL_SPEED);
    return;
  }

  if (L && !R) {
    // Spin left
    turnLeft_raw(TURN_SPEED);
    return;
  }

  if (R && !L) {
    // Spin right
    turnRight_raw(TURN_SPEED);
    return;
  }

  // No input
  stopAll();
}

// -------------------- AUTONOMOUS --------------------
void autonomousStep() {
  unsigned long now = millis();

  // Emergency reaction before deciding
  if (checkObstacle()) {
    Serial.println("AUTO: obstacle detected - backup+rotate");
    
    // Trigger eye animation
    handleObstacleAnimation();
    
    // Emergency maneuver
    backward_raw(BACK_SPEED);
    delay(400);
    stopAll();
    delay(80);
    
    if (random(0, 2) == 0) {
      turnLeft_raw(TURN_SPEED);
      delay(random(350, 900));
    } else {
      turnRight_raw(TURN_SPEED);
      delay(random(350, 900));
    }
    
    stopAll();
    stateUntil = now + 250;
    astate = AUTO_FORWARD;
    return;
  }

  // If current state expired, pick next action
  if (now >= stateUntil) {
    int r = random(0, 100);
    if (r < 55) { 
      astate = AUTO_FORWARD; 
      stateUntil = now + random(1200, 3000); 
    }
    else if (r < 75) { 
      astate = AUTO_TURN_LEFT; 
      stateUntil = now + random(600, 1600); 
    }
    else if (r < 95) { 
      astate = AUTO_TURN_RIGHT; 
      stateUntil = now + random(600, 1600); 
    }
    else { 
      astate = AUTO_PAUSE; 
      stateUntil = now + random(300, 900); 
    }
  }

  // Execute current state
  switch (astate) {
    case AUTO_FORWARD:
      forward_with_safety(BASE_SPEED);
      break;

    case AUTO_TURN_LEFT:
      turnLeft_raw((int)(BASE_SPEED * 0.7));
      break;

    case AUTO_TURN_RIGHT:
      turnRight_raw((int)(BASE_SPEED * 0.7));
      break;

    case AUTO_BACKUP:
      backward_raw(BACK_SPEED);
      break;

    case AUTO_PAUSE:
      stopAll();
      break;

    case AUTO_SCAN:
      if (scanStep % 4 == 0) turnLeft_raw(TURN_SPEED);
      else if (scanStep % 4 == 1) stopAll();
      else if (scanStep % 4 == 2) turnRight_raw(TURN_SPEED);
      else stopAll();
      if (millis() >= stateUntil) { 
        scanStep++; 
        stateUntil = millis() + 350; 
      }
      break;
  }
}

// -------------------- MODE TOGGLE --------------------
void handleModeToggle() {
  static bool lastSelect = false;
  static unsigned long lastToggleTime = 0;
  unsigned long now = millis();
  
  bool currentSelect = GamePad.isSelectPressed();
  
  if (currentSelect && !lastSelect) {
    if (now - lastToggleTime > MODE_DEBOUNCE) {
      modeManual = !modeManual;
      lastToggleTime = now;

      stopAll();
      astate = AUTO_FORWARD;
      stateUntil = now + 400;
      scanStep = 0;
      
      Dabble.processInput();

      Serial.println("\n========================================");
      Serial.print("MODE CHANGED TO: ");
      Serial.println(modeManual ? "MANUAL" : "AUTONOMOUS");
      Serial.println("========================================");
      
      // Reset eye position ketika ganti mode
      setEyeDirection(EYE_FORWARD);
    }
  }

  lastSelect = currentSelect;
}

// -------------------- DEBUG --------------------
void printDebugInfo() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < 1500) return;
  last = now;
  
  Serial.print("Mode=");
  Serial.print(modeManual ? "MANUAL" : "AUTO");
  Serial.print(" | Eye=");
  switch(currentEyeDirection) {
    case EYE_FORWARD: Serial.print("FWD"); break;
    case EYE_LEFT: Serial.print("LEFT"); break;
    case EYE_RIGHT: Serial.print("RIGHT"); break;
    case EYE_UP: Serial.print("UP"); break;
    case EYE_CONFUSED: Serial.print("CONFUSED"); break;
  }
  Serial.print(" | Obstacle=");
  Serial.print(checkObstacle() ? "YES" : "NO");
  Serial.print(" | State=");
  switch(astate) {
    case AUTO_FORWARD: Serial.print("FWD"); break;
    case AUTO_TURN_LEFT: Serial.print("TLEFT"); break;
    case AUTO_TURN_RIGHT: Serial.print("TRIGHT"); break;
    case AUTO_BACKUP: Serial.print("BACK"); break;
    case AUTO_PAUSE: Serial.print("PAUSE"); break;
    case AUTO_SCAN: Serial.print("SCAN"); break;
  }
  Serial.println();
}

// -------------------- MAIN LOOP --------------------
void loop() {
  Dabble.processInput();
  handleModeToggle();

  updateEyesBasedOnMovement();

  if (modeManual) {
    manualControl();
  } else {
    autonomousStep();
  }

  printDebugInfo();
  delay(10);
}