#include <SPI.h>
#include <RH_RF95.h>
#include <ezButton.h>

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4
#endif

#define RF95_FREQ 915.0
char received_data[RH_RF95_MAX_MESSAGE_LEN];

// Define motor control pins
const int voltA = 5;
const int voltB = 11;
const int pwm_port = 9;
const int duty_cycle = 255;

// Define switch pins
const int topSwitchPin = 12;
const int bottomSwitchPin = A3;

// Initialize switches with debounce
ezButton topSwitch(topSwitchPin);
ezButton bottomSwitch(bottomSwitchPin);

// Timers for submerging and surfacing
const unsigned long WAIT_TIME = 5000; // Wait for 5 secs
unsigned long state_timer = 0;

// Enum for motor direction
enum Motor_Direction {
  CLOCKWISE,
  COUNTERCLOCKWISE,
  STALLED
};

Motor_Direction motor_direction = STALLED; 

enum Motor_State {
  IDLE,
  MOVING_DOWN,
  WAITING_BOTTOM,
  MOVING_UP,
  WAITING_TOP
};

Motor_State motor_state = IDLE;

// Declare RF95 object
RH_RF95 rf95(RFM95_CS, RFM95_INT);

void setup() {
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  digitalWrite(RFM95_RST, LOW);
  digitalWrite(RFM95_RST, HIGH);

  topSwitch.setDebounceTime(50);
  bottomSwitch.setDebounceTime(50);

  pinMode(voltA, OUTPUT);
  pinMode(voltB, OUTPUT);
  pinMode(pwm_port, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(topSwitchPin, INPUT_PULLUP);
  pinMode(bottomSwitchPin, INPUT_PULLUP);

  bottomSwitch.loop(); 

  if (!bottomSwitch.isPressed()) {
    motor_state = MOVING_DOWN;
    motor_direction = CLOCKWISE;
  } else {
    motor_state = IDLE;
    motor_direction = STALLED;
  }

  // Initialize RF95 module
  while (!rf95.init()) {
    while (1); // hang forever if radio doesn't init
  }

  if (!rf95.setFrequency(RF95_FREQ)) {
    while (1); // hang forever if freq not set
  }
  rf95.setTxPower(23, false);
}

void loop() {
  topSwitch.loop();
  bottomSwitch.loop();

  bool topPressed = topSwitch.isPressed();
  bool bottomPressed = bottomSwitch.isPressed();

  unsigned long current_time = millis();

  if (rf95.waitAvailableTimeout(1000)) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.recv(buf, &len)) {
      strncpy(received_data, (char*)buf, len);
    }

    // If command is received
    if (strcmp(received_data, "initiate") == 0) {
      switch (motor_state) {
        case IDLE:
          // if bottom switch is not pressed, start moving down
          if (!bottomPressed) {
            motor_state = MOVING_DOWN;
            motor_direction = CLOCKWISE;
          }
          break;

        case MOVING_DOWN:
          // stop when the bottom switch is pressed
          if (bottomPressed) {
            motor_state = WAITING_BOTTOM;
            motor_direction = STALLED;
            state_timer = current_time;
          }
          break;

        case WAITING_BOTTOM:
          // wait 5 seconds before moving up
          if (current_time - state_timer >= WAIT_TIME) {
            motor_state = MOVING_UP;
            motor_direction = COUNTERCLOCKWISE;
          }
          break;

        case MOVING_UP:
          // stop when the top switch is pressed
          if (topPressed) {
            motor_state = WAITING_TOP;
            motor_direction = STALLED;
            state_timer = current_time;
          }
          break;

        case WAITING_TOP:
          // wait 5 seconds before moving down
          if (current_time - state_timer >= WAIT_TIME) {
            motor_state = MOVING_DOWN;
            motor_direction = CLOCKWISE;
          }
          break;
      }

      // Control the motor based on the direction
      controlMotor();
    }
  }
}

void controlMotor() {
  switch (motor_direction) {
    case CLOCKWISE:
      digitalWrite(voltA, HIGH);
      digitalWrite(voltB, LOW);
      analogWrite(pwm_port, duty_cycle); 
      break;

    case COUNTERCLOCKWISE:
      digitalWrite(voltA, LOW);
      digitalWrite(voltB, HIGH);
      analogWrite(pwm_port, duty_cycle);
      break;

    case STALLED:
    default:
      digitalWrite(voltA, LOW);
      digitalWrite(voltB, LOW);
      analogWrite(pwm_port, 0); 
      break;
  }
}
