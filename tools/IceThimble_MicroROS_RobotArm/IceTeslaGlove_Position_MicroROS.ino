
#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include "TinyPICO.h"'

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/float32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Initial led object
TinyPICO tp = TinyPICO();

// Initialize servo code
// Define struct containing force sensed data
typedef struct force_data {
  float force_sensed_index = 0;
  float force_sensed_thumb = 0;
  float force_sensed_middle = 0;
  float force_sensed_ring = 0;
  float force_sensed_little = 0;
  float force_sensed_palm = 0;
};

// Initialize force_data struct
struct force_data posData;

// Set load cell pin number
int thumb_pin = 14;
int index_pin = 14;
int middle_pin = 15;
int ring_pin = 27;
int little_pin = 26;
int palm_pin = 25;

// Initialize other auxiliary variables
int limit = 254;
int offset[6] = {0, 0, 0, 0, 0, 0}; // Offset values [thumb, index, middle, ring, little, palm]

void error_loop(){
  while(1){
    tp.DotStar_SetPixelColor(255, 0, 0);
    delay(100);
  }
}

void read_volt()
{
  posData.force_sensed_thumb = analogRead(thumb_pin);
  posData.force_sensed_index = analogRead(index_pin);
  posData.force_sensed_middle = analogRead(middle_pin);
  posData.force_sensed_ring = analogRead(ring_pin);
  posData.force_sensed_little = analogRead(little_pin);


  //Serial.println(posData.force_sensed_thumb);
  //Serial.println(analogRead(thumb_pin));
}

void setup() {
  //Serial.begin(115200);

  set_microros_wifi_transports("JM2_nett", "autyn33369", "192.168.0.107", 8888);

  delay(2000);
  
  // Set Blu light for status
  tp.DotStar_SetPixelColor(0, 0, 255);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "tinypico_finger_position", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "ice_glove_pos_id1"));
  

  // Hardware setup
  // Set pins to input
  pinMode(thumb_pin, INPUT);
  pinMode(index_pin, INPUT);
  pinMode(middle_pin, INPUT);
  pinMode(ring_pin, INPUT);
  pinMode(little_pin, INPUT);

}

void loop() {
  // Status green for ready
  tp.DotStar_SetPixelColor(0, 255, 0);

  // Updater finger values
  read_volt();

  // Write new message
  msg.linear.x = posData.force_sensed_thumb;
  msg.linear.y = posData.force_sensed_index;
  msg.linear.z = posData.force_sensed_middle;
  msg.angular.x = posData.force_sensed_ring;
  msg.angular.y = posData.force_sensed_little;
  msg.angular.z = 42;

  // Publish message
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  //Serial.println(analogRead(thumb_pin));
  //Serial.println(posData.force_sensed_index);

  delay(10);

}
