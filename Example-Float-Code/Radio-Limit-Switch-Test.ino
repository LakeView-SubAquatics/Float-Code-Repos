#include <SPI.h>
#include <RH_RF95.h>
#include <ezButton.h>

// First 3 here are boards w/radio BUILT-IN. Boards using FeatherWing follow.

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4

#endif

// Define motor control pins
const int voltA = 5;
const int voltB = 11;
const int pwm_port = 9;
const int duty_cycle = 255; // Maximum speed (0-255)

// Define switch pins
const int topSwitchPin = 12;
const int bottomSwitchPin = A3;

// Initialize switches with debounce
ezButton topSwitch(topSwitchPin);
ezButton bottomSwitch(bottomSwitchPin);

// Float Motor Setup
bool setup_motor = false;

// Hold Motor Timer
unsigned long motor_hold_timer = 0;
const long MOTOR_HOLD_INTERVAL = 20000

// Enum for motor direction
enum Motor_Direction {
  CLOCKWISE,
  COUNTERCLOCKWISE,
  STALLED
};

// Variable to track motor direction
Motor_Direction motor_direction = STALLED;

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Radio REcieve Tring
char received_data[RH_RF95_MAX_MESSAGE_LEN];

void setup() {
  // Configure switches with debouncing
  topSwitch.setDebounceTime(50);
  bottomSwitch.setDebounceTime(50);

  // Set motor control pins as outputs
  pinMode(voltA, OUTPUT);
  pinMode(voltB, OUTPUT);
  pinMode(pwm_port, OUTPUT);

  // Configure built-in LED for status indication
  pinMode(LED_BUILTIN, OUTPUT);

  // Set switch pins as inputs with pull-up resistors
  pinMode(topSwitchPin, INPUT_PULLUP);
  pinMode(bottomSwitchPin, INPUT_PULLUP);
  
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    while (1);
  }

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    while (1);
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);
}

int16_t packetnum = 0;  // packet counter, we increment per xmission

void loop() {
  unsigned long current_millis = millis();
  float radiopacket[10] = {3.33, 3.33, 3.33, 5.43, 1.97, 8.21, 0.21, 9.54, 7.43, 5.01};
  String arrayString = floatToString(radiopacket, 10, ',');

  char charBuffer[arrayString.length() + 1]; // +1 for null terminator
  arrayString.toCharArray(charBuffer, arrayString.length() + 1);

  // Send the data over LoRa
  rf95.send((uint8_t *)charBuffer, arrayString.length());

  rf95.waitPacketSent();

  if (rf95.waitAvailableTimeout(1000)) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        strncpy(received_data, (char*)buf, len);
      }

      if (strcmp(received_data, "initiate") == 0){
        // Update switch states (handles debounce)
      // Send the data over LoRa
      rf95.send((uint8_t *)charBuffer, arrayString.length());
          
      topSwitch.loop();
      bottomSwitch.loop();
    
      // Read switch states
      bool topPressed = topSwitch.isPressed();
      bool bottomPressed = bottomSwitch.isPressed();

      while (!setup_motor){
        if (!bottomPressed){
          motor_direction = CLOCKWISE;
        } else if(bottomPressed){
          setup_motor = true;
          motor_direction = STALLED;
        }
      }
    
      // Change motor direction only when a button is pressed
      if (topPressed && bottomPressed) {
        motor_direction = STALLED; // Stop motor if both are pressed
      } else if (topPressed && current_millis - motor_hold_timer >= MOTOR_HOLD_INTERVAL) {
        motor_direction = CLOCKWISE;
        motor_hold_timer = current_millis;
      } else if (bottomPressed) {
        motor_direction = COUNTERCLOCKWISE;
        motor_hold_timer = current_millis;
      }
    
      // Control motor direction
      switch (motor_direction) {
        case CLOCKWISE:
          digitalWrite(LED_BUILTIN, LOW);
          digitalWrite(voltA, HIGH);
          digitalWrite(voltB, LOW);
          analogWrite(pwm_port, duty_cycle); // Apply PWM
          break;
        
        case COUNTERCLOCKWISE:
          digitalWrite(LED_BUILTIN, HIGH);
          digitalWrite(voltA, LOW);
          digitalWrite(voltB, HIGH);
          analogWrite(pwm_port, duty_cycle); // Apply PWM
          break;
        
        case STALLED:
        default:
          digitalWrite(voltA, LOW);
          digitalWrite(voltB, LOW);
          analogWrite(pwm_port, 0); // Stop motor
          break;
      }
    }
  }
}

String floatToString(float *array, int length, char delimiter) {
  String result = "";
  // Loop through each element in the array
  for (int i = 0; i < length; i++) {
    // Convert the float value to a string and append it to the result
    result += String(array[i]);
    // Add delimiter unless it's the last element
    if (i < length - 1) {
      result += delimiter;
    }
  }
  return result;
}
