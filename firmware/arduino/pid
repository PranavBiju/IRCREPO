#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <nav_msgs/msg/odometry.h>

#include "cytrons.h"    
#include <Arduino.h>
#include "driver/pcnt.h"

//PINS AND PID THINGS
#define M1_PWM 4
#define M1_DIR 16
#define ENC1_A 25
#define ENC1_B 26

#define M2_PWM 17
#define M2_DIR 21
#define ENC2_A 27
#define ENC2_B 14

#define M3_PWM 22
#define M3_DIR 23
#define ENC3_A 33
#define ENC3_B 32

#define M4_PWM 5
#define M4_DIR 15
#define ENC4_A 18
#define ENC4_B 19

//PCNT SETUP
pcnt_unit_t encUnits[4] = {PCNT_UNIT_0, PCNT_UNIT_1, PCNT_UNIT_2, PCNT_UNIT_3};
volatile int32_t encoderCount[4] = {0, 0, 0, 0};

const float ENCODER_PPR = 240.0f; // <-- EDIT THIS

//ROBOT DATA
const float WHEEL_RADIUS = 0.08f;
const float WHEEL_BASE   = 0.30f;
const float MAX_VEL      = 1.0f;

// boilerplate
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_publisher_t odom_publisher;
rclc_executor_t executor;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
geometry_msgs__msg__Twist cmd_vel_msg;
std_msgs__msg__Int32MultiArray enc_msg;
nav_msgs__msg__Odometry odom_msg;

// MOTOR OBJECTS
MDD10A motorLF(M1_PWM, M1_DIR, ENC1_A, ENC1_B);
MDD10A motorLR(M2_PWM, M2_DIR, ENC2_A, ENC2_B);
MDD10A motorRF(M3_PWM, M3_DIR, ENC3_A, ENC3_B);
MDD10A motorRR(M4_PWM, M4_DIR, ENC4_A, ENC4_B);

//PID STATE 
struct PIDState {
  long prevPos = 0;
  long prevT_us = 0;
  float eprev = 0.0f;
  float eintegral = 0.0f;
};

PIDState pidLF, pidLR, pidRF, pidRR;

const float KP_LF = 0.0f; const float KI_LF = 0.0f; const float KD_LF = 0.00f;
const float KP_LR = 0.3f; const float KI_LR = 0.6f; const float KD_LR = 0.0002f;
const float KP_RF = 0.3f; const float KI_RF = 0.6f; const float KD_RF = 0.0002f;
const float KP_RR = 0.3f; const float KI_RR = 0.7f; const float KD_RR = 0.0015f;

float x = 0.0f, y = 0.0f, theta = 0.0f;
long prevPosLF = 0, prevPosLR = 0, prevPosRF = 0, prevPosRR = 0;
unsigned long last_odom_time = 0;

// CMD_VEL variables
volatile float cmd_v = 0.0f;  // linear.x
volatile float cmd_w = 0.0f;  // angular.z

// Timeout protection
volatile unsigned long last_cmd_time = 0;
const unsigned long CMD_TIMEOUT_MS = 500;

void setupEncoder(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t pcnt_config = {
    .pulse_gpio_num = pinA,
    .ctrl_gpio_num  = pinB,
    .lctrl_mode     = PCNT_MODE_REVERSE,
    .hctrl_mode     = PCNT_MODE_KEEP,
    .pos_mode       = PCNT_COUNT_INC,
    .neg_mode       = PCNT_COUNT_DEC,
    .counter_h_lim  = 32767,
    .counter_l_lim  = -32768,
    .unit           = unit,
    .channel        = PCNT_CHANNEL_0
  };
  pcnt_unit_config(&pcnt_config);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void updateEncoderCounts() {
  for (int i = 0; i < 4; ++i) {
    int16_t val = 0;
    pcnt_get_counter_value(encUnits[i], &val);
    encoderCount[i] = val;
  }
}

inline float linearVelToTicksPerSec(float v_mps) {
  float revs_per_sec = v_mps / (2.0f * PI * WHEEL_RADIUS);
  return revs_per_sec * ENCODER_PPR;
}
inline float ticksPerSecToLinearVel(float ticks_per_sec) {
  float revs_per_sec = ticks_per_sec / ENCODER_PPR;
  float linear_vel = revs_per_sec * (2.0f * PI * WHEEL_RADIUS); // m/s
  return linear_vel;
}

// ---------- PID WITH ANTI-WINDUP ----------
int computePIDandOutput(MDD10A &motor, PIDState &state, long currentPos, long currentTimeUs, float target_ticks_per_sec,
                        float KP, float KI, float KD, bool invertVelocity=false) {
  const float INTEGRAL_MAX = 75.0f; // anti-windup cap

  if (state.prevT_us == 0) {
    state.prevT_us = currentTimeUs;
    state.prevPos = currentPos;
    return 0;
  }
  float deltaT = (currentTimeUs - state.prevT_us) / 1.0e6f;
  if (deltaT <= 0.0f) deltaT = 1e-6f;

  float measured_ticks_per_sec = (float)(currentPos - state.prevPos) / deltaT;
  if (invertVelocity) measured_ticks_per_sec *= -1;

  float e = target_ticks_per_sec - measured_ticks_per_sec;
  float dedt = (e - state.eprev) / deltaT;

  state.eintegral += e * deltaT;

  // 🔹 Anti-windup clamp
  if (state.eintegral > INTEGRAL_MAX) state.eintegral = INTEGRAL_MAX;
  if (state.eintegral < -INTEGRAL_MAX) state.eintegral = -INTEGRAL_MAX;

  float pwr = KP * e + KI * state.eintegral + KD * dedt;

  if (pwr > 255.0f) pwr = 255.0f;
  if (pwr < -255.0f) pwr = -255.0f;

  int out = (int)round(pwr);
  motor.run(out);

  state.prevPos = currentPos;
  state.prevT_us = currentTimeUs;
  state.eprev = e;

  return (int)measured_ticks_per_sec;
}

geometry_msgs__msg__Quaternion yawToQuaternion(float yaw) {
  geometry_msgs__msg__Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = sin(yaw / 2.0);
  q.w = cos(yaw / 2.0);
  return q;
}

// ---------------- CMD_VEL CALLBACK ----------------
void cmd_vel_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_v = msg->linear.x;
  cmd_w = msg->angular.z;
  last_cmd_time = millis(); // update timestamp
  Serial.print("[cmd_vel] v: ");
  Serial.print(cmd_v);
  Serial.print(", w: ");
  Serial.println(cmd_w);
}

void setup() {
  Serial.begin(115200);
  delay(50);

  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_DIR, OUTPUT);

  setupEncoder(encUnits[0], ENC1_A, ENC1_B);
  setupEncoder(encUnits[1], ENC2_A, ENC2_B);
  setupEncoder(encUnits[2], ENC3_A, ENC3_B);
  setupEncoder(encUnits[3], ENC4_A, ENC4_B);

  // ---------------- micro-ROS init ----------------
  set_microros_transports();
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_rover_node", "", &support);

  // Subscriber for /cmd_vel
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel"
  );

  // Encoder publisher
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/encoders"
  );

  // Executor setup
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

  enc_msg.data.data = (int32_t*)malloc(4 * sizeof(int32_t));
  enc_msg.data.size = 4;
  enc_msg.data.capacity = 4;

  Serial.println("System Ready — micro-ROS /cmd_vel subscriber active");
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Timeout safety
  if (millis() - last_cmd_time > CMD_TIMEOUT_MS) {
    cmd_v = 0.0f;
    cmd_w = 0.0f;
    pidLF.eintegral = pidLR.eintegral = pidRF.eintegral = pidRR.eintegral = 0.0f;
  }

  // 🔹 Reset PID if command is near zero (prevents residual jitter)
  if (fabs(cmd_v) < 0.01 && fabs(cmd_w) < 0.01) {
    pidLF.eintegral = pidLR.eintegral = pidRF.eintegral = pidRR.eintegral = 0.0f;
  }

  updateEncoderCounts();
  long currTimeUs = micros();

  long posLF = encoderCount[0];
  long posLR = encoderCount[1];
  long posRF = encoderCount[2];
  long posRR = encoderCount[3];

  float v = cmd_v;
  float w = cmd_w;

  float v_left  = v - (w * WHEEL_BASE / 2.0);
  float v_right = v + (w * WHEEL_BASE / 2.0);

  float target_ticks_LF = linearVelToTicksPerSec(v_left);
  float target_ticks_LR = linearVelToTicksPerSec(v_left);
  float target_ticks_RF = linearVelToTicksPerSec(v_right); 
  float target_ticks_RR = linearVelToTicksPerSec(v_right);

  int velLF = computePIDandOutput(motorLF, pidLF, posLF, currTimeUs, target_ticks_LF, KP_LF, KI_LF, KD_LF, true);
  int velLR = computePIDandOutput(motorLR, pidLR, posLR, currTimeUs, target_ticks_LR, KP_LR, KI_LR, KD_LR);
  int velRF = computePIDandOutput(motorRF, pidRF, posRF, currTimeUs, -target_ticks_RF, KP_RF, KI_RF, KD_RF,true);
  int velRR = computePIDandOutput(motorRR, pidRR, posRR, currTimeUs, target_ticks_RR, KP_RR, KI_RR, KD_RR);

  // Publish encoders
  enc_msg.data.data[0] = posLF;
  enc_msg.data.data[1] = posLR;
  enc_msg.data.data[2] = posRF;
  enc_msg.data.data[3] = posRR;
  rcl_publish(&publisher, &enc_msg, NULL);

  delay(50);
}
