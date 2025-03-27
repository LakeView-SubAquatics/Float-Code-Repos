/* 
  Author(s): Tyerone Chen, Danny Henningfield, Adam Palma
  Innit Create: 6/30/2024
  Last update: 3/27/2025
*/

#include <SPI.h>
#include <RH_RF95.h>
#include <ezButton.h>
#include <List.hpp>

#if defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
#define RFM95_CS    8
#define RFM95_INT   3
#define RFM95_RST   4
#endif

#define RF95_FREQ 915.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);

#pragma region Variable_Definement
List<float> psiList;
List<float> depthList;
List<unsigned long> timeList;

float depth = 0;
const String COMPANY_NAME = "LakeView SubAquatics-8";
const int MAX_LIST_SIZE = 2000;
char received_data[RH_RF95_MAX_MESSAGE_LEN];
float psi_half_sec = 0;
float psi_full_sec = 0;

const int OUT_A = 5;
const int DIAG_PORT_A = 6;
const int OUT_B = 11;
const int DIAG_PORT_B = 10;
const int PWM_PORT = 9;
const int DUTY_CYCLE = 255;

ezButton switch_top(12);
ezButton switch_bottom(A3);

enum Float_State { SURFACED, SUBMURSED, MOVING, MAINTAIN, FLOORED };
volatile Float_State float_curr_state = SURFACED;

enum Switch_State { ACTIVE, INACTIVE };
float const MAX_MAINTAIN_DEPTH = 2.6;
float const MIN_MAINTAIN_DEPTH = 2.4;
volatile Switch_State switch_top_state = INACTIVE;
volatile Switch_State switch_bottom_state = INACTIVE;

enum Motor_Direction { CLOCKWISE, COUNTERCLOCKWISE, STALLED };
#pragma endregion

#pragma region Setup
void setup() {
  analogWrite(PWM_PORT, DUTY_CYCLE);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  switch_top.setDebounceTime(50);
  switch_bottom.setDebounceTime(50);
  attachInterrupt(digitalPinToInterrupt(12), switchTopDetect, CHANGE);
  attachInterrupt(digitalPinToInterrupt(A3), switchBottomDetect, CHANGE);

  pinMode(OUT_A, OUTPUT);
  pinMode(OUT_B, OUTPUT);
  pinMode(DIAG_PORT_A, INPUT_PULLUP);
  pinMode(DIAG_PORT_B, INPUT_PULLUP);
  pinMode(PWM_PORT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(DIAG_PORT_A, HIGH);
  digitalWrite(DIAG_PORT_B, HIGH);

  pinMode(12, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);

  int pressure_pin = analogRead(A1);
  float psi = (0.0374 * pressure_pin) - 3.3308;
  float pascal_pi = (psi - 14.7) * 6894.76;
  depth = (pascal_pi / (1000 * 9.81));
  float_curr_state = psiCompare(psi_half_sec, psi_full_sec, depth, switch_bottom_state, switch_top_state, switch_bottom.isPressed(), switch_top.isPressed());

  digitalWrite(RFM95_RST, LOW);
  digitalWrite(RFM95_RST, HIGH);

  while (!rf95.init()) {
    handleNoResponse();
  }
  if (!rf95.setFrequency(RF95_FREQ)) {
    handleNoResponse();
  }
  rf95.setTxPower(23, false);
}
#pragma endregion

int16_t packetnum = 0;
unsigned long radio_task_millis = 0;
unsigned long psi_task_half_millis = 0;
unsigned long psi_task_full_millis = 0;
unsigned long psi_change_check_millis = 0;
unsigned long list_updater_millis = 0;
unsigned long maintain_depth_millis = 0;

const long RADIO_TASK_INTERVAL = 1001;
const long PSI_TASK_HALF_INTERVAL = 1001;
const long PSI_TASK_FULL_INTERVAL = 1250;
const long PSI_CHANGE_CHECK_INTERVAL = 500;
const long LIST_UPDATER_INTERVAL = 5000;
const long MAINTAIN_DEEPTH_DURATION = 60000;

#pragma region Main_Program/Loop
void loop() {
  unsigned long current_millis = millis();

  int pressure_pin = analogRead(A1);
  float psi = (0.0374 * pressure_pin) - 3.3308;
  float pascal_pi = (psi - 14.7) * 6894.76;
  depth = (pascal_pi / (1000 * 9.81));

  switch_top.loop();
  switch_bottom.loop();
  bool ez_switch_top = switch_top.isPressed();
  bool ez_switch_bottom = switch_bottom.isPressed();

  if (current_millis - radio_task_millis >= RADIO_TASK_INTERVAL) {
    radio_task_millis = current_millis;
    if (rf95.waitAvailableTimeout(1000)) {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.recv(buf, &len)) {
        strncpy(received_data, (char*)buf, len);
      } else {
        handleNoResponse();
      }
    } else {
      handleNoResponse();
    }
  }

  if (strcmp(received_data, "initiate") == 0) {
    float_curr_state = psiCompare(psi_half_sec, psi_full_sec, depth, switch_bottom_state, switch_top_state, ez_switch_bottom, ez_switch_top);
    motorDirection(float_curr_state, depth);
    if (float_curr_state == MAINTAIN) {
      maintainDepth(current_millis, depth);
    }

    if (current_millis - list_updater_millis >= LIST_UPDATER_INTERVAL) {
      list_updater_millis = current_millis;
      if (psiList.getSize() >= MAX_LIST_SIZE) {
        psiList.remove(0);
        depthList.remove(0);
        timeList.remove(0);
      }
      psiList.add(psi);
      depthList.add(depth);
      timeList.add(current_millis);
    }
    if (current_millis - psi_task_half_millis >= PSI_TASK_HALF_INTERVAL) {
      psi_task_half_millis = current_millis;
      psi_half_sec = psi;
    }
    if (current_millis - psi_task_full_millis >= PSI_TASK_FULL_INTERVAL) {
      psi_task_full_millis = current_millis;
      psi_full_sec = psi;
    }
    if (current_millis - psi_change_check_millis >= PSI_CHANGE_CHECK_INTERVAL) {
      psi_change_check_millis = current_millis;
      float_curr_state = psiCompare(psi_half_sec, psi_full_sec, depth, switch_bottom_state, switch_top_state, ez_switch_bottom, ez_switch_top);
    }

    if (float_curr_state == SURFACED) {
      sendData();
    }
  } else {
    digitalWrite(OUT_A, LOW);
    digitalWrite(OUT_B, LOW);
    pinMode(12, INPUT_PULLUP);
    pinMode(A3, INPUT_PULLUP);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
#pragma endregion

#pragma region Functions
void sendData() {
  String data = COMPANY_NAME + ": ";
  data += "PSI: ";
  for (int i = 0; i < psiList.getSize(); i++) {
    data += String(psiList.get(i)) + ", ";
  }
  data += "DEPTH: ";
  for (int i = 0; i < depthList.getSize(); i++) {
    data += String(depthList.get(i)) + ", ";
  }
  data += "TIME: ";
  for (int i = 0; i < timeList.getSize(); i++) {
    data += String(timeList.get(i)) + ", ";
  }

  if (data.length() >= RH_RF95_MAX_MESSAGE_LEN) {
    data = data.substring(0, RH_RF95_MAX_MESSAGE_LEN - 1);
  }

  char send_buffer[RH_RF95_MAX_MESSAGE_LEN];
  data.toCharArray(send_buffer, RH_RF95_MAX_MESSAGE_LEN);
  rf95.send((uint8_t *)send_buffer, strlen(send_buffer));
  rf95.waitPacketSent();
}

void handleNoResponse() {
  float_curr_state = MOVING;
  strncpy(received_data, "", sizeof(received_data));
}

Float_State psiCompare(float half_time_psi, float full_time_psi, float curr_depth, 
    Switch_State switch_bottom_state, Switch_State switch_top_state, bool ez_switch_bottom, bool ez_switch_top) {
  float calc_psi_diff = abs(full_time_psi - half_time_psi);

  if (calc_psi_diff <= 1.0) {
    if (curr_depth <= MAX_MAINTAIN_DEPTH && curr_depth >= MIN_MAINTAIN_DEPTH) {
      return MAINTAIN;
    } else if ((switch_bottom_state == ACTIVE && switch_top_state == INACTIVE) || (ez_switch_bottom == true && ez_switch_top == false)) {
      return SURFACED;
    } else if ((switch_bottom_state == INACTIVE && switch_top_state == ACTIVE) || (ez_switch_bottom == false && ez_switch_top == true)) {
      return FLOORED;
    } else if ((switch_bottom_state == INACTIVE && switch_top_state == INACTIVE) || (ez_switch_bottom == false && ez_switch_top == false)) {
      return SUBMURSED;
    }
  }
  return MOVING;
}

void motorDirection(Float_State float_state, float curr_depth) {
  switch (float_state) {
    case SURFACED:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, HIGH);
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case SUBMURSED:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case FLOORED:
      digitalWrite(OUT_A, HIGH);
      digitalWrite(OUT_B, LOW);
      digitalWrite(LED_BUILTIN, HIGH);
      break;
    case MOVING:
      if (curr_depth < 2.5) {
        digitalWrite(OUT_A, HIGH);
        digitalWrite(OUT_B, LOW);
      } else {
        digitalWrite(OUT_A, LOW);
        digitalWrite(OUT_B, HIGH);
      }
      digitalWrite(LED_BUILTIN, LOW);
      break;
    case MAINTAIN:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      maintain_depth_millis = millis();
      break;
    default:
      digitalWrite(OUT_A, LOW);
      digitalWrite(OUT_B, LOW);
      digitalWrite(LED_BUILTIN, LOW);
      break;
  }
  pinMode(12, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
}

void maintainDepth(unsigned long current_millis, float curr_depth) {
  static unsigned long maintain_start_time = 0;

  if (float_curr_state == MAINTAIN && maintain_start_time == 0) {
    maintain_start_time = current_millis;
  }

  if (current_millis - maintain_start_time >= MAINTAIN_DEEPTH_DURATION) {
    float_curr_state = MOVING;
    maintain_start_time = 0;
  } else if (curr_depth < MIN_MAINTAIN_DEPTH) {
    digitalWrite(OUT_A, HIGH);
    digitalWrite(OUT_B, LOW);
  } else if (curr_depth > MAX_MAINTAIN_DEPTH) {
    digitalWrite(OUT_A, LOW);
    digitalWrite(OUT_B, HIGH);
  } else {
    digitalWrite(OUT_A, LOW);
    digitalWrite(OUT_B, LOW);
  }
  pinMode(12, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
}

void switchBottomDetect() {
  if (digitalRead(A3) == HIGH) {
    switch_bottom_state = ACTIVE;
    switch_top_state = INACTIVE;
  }
}

void switchTopDetect() {
  if (digitalRead(12) == HIGH) {
    switch_top_state = ACTIVE;
    switch_bottom_state = INACTIVE;
  }
}
#pragma endregion
