// #include <micro_ros_arduino.h>

// #include <stdio.h>
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>

// #include <Servo.h>

// #include <std_msgs/msg/float32_multi_array.h>
// #include <std_msgs/msg/int32.h>
// #include <EEPROM.h>
// //extern "C" void NVIC_SystemReset(void);


// //https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/handle_reconnections/handle_reconnections.html
// //https://docs.vulcanexus.org/en/latest/rst/tutorials/micro/memory_management/memory_management.html#id1
// //https://micro.ros.org/docs/tutorials/advanced/microxrcedds_rmw_configuration/

// bool micro_ros_initialized = false;
// bool connection_active = false; // Track connection state

// unsigned long lastConnectionLossTime = 0;
// #define WATCHDOG_TIMEOUT 10000 //10 sec timeout

// // Define EEPROM flag parameters.
// #define RESET_FLAG_ADDR 0      // EEPROM address to store the flag.
// #define RESET_FLAG_VALUE 0x55  // Arbitrary non-zero value.


// // Global micro-ROS objects for subscription (servo_commands)
// rcl_subscription_t subscriber;
// std_msgs__msg__Float32MultiArray servo_msg;

// // Global micro-ROS objects for publisher (youssef)
// rcl_publisher_t publisher;
// std_msgs__msg__Int32 pub_msg;

// rclc_executor_t executor;
// rclc_support_t support;
// rcl_allocator_t allocator;
// rcl_node_t node;
// rcl_timer_t timer;

// Servo servo1, servo2;

// #define SERVO1_PIN 7
// #define SERVO2_PIN 8     //CHANGE THESE VALUES

// //from ping pong
// #define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
// #define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

// // Forward declarations of our init and cleanup routines.
// void init_micro_ros();
// void cleanup_micro_ros();

// void resetBoard() {
//   // Function pointer to address 0 (the reset vector)
//   void (*resetFunc)(void) = 0;
//   resetFunc();
// }

// // Call this in setup to force one reset on first run.
// void maybeResetOnce() {
//   // Initialize EEPROM with a size of 1 byte (if not already done).
//   //EEPROM.begin(1); // For boards that require this (like ESPs); Teensy may not need it.
//   uint8_t flag = EEPROM.read(RESET_FLAG_ADDR);
//   if (flag != RESET_FLAG_VALUE) {
//     // Set the flag so we know a reset was triggered.
//     EEPROM.write(RESET_FLAG_ADDR, RESET_FLAG_VALUE);
//     delay(100);  // Give a moment for the serial message to go out.
//     resetBoard();
//   } else {
//     // Clear the flag so subsequent runs don't trigger a reset.
//     EEPROM.write(RESET_FLAG_ADDR, 0);
//   }
// }

// void subscription_callback(const void * msg_in)
// {  
//   const std_msgs__msg__Float32MultiArray *msg = (const std_msgs__msg__Float32MultiArray *)msg_in;

//   if (msg->data.size >= 2) {  // Ensure at least 2 values exist
//     int servo1_angle = static_cast<int>(msg->data.data[0]); // First value for Servo 1
//     int servo2_angle = static_cast<int>(msg->data.data[1]); // Second value for Servo 2

//     printf(servo1_angle);
//     printf(servo2_angle);
//     // servo1.write(servo1_angle);  // Move Servo 1
//     // servo2.write(servo2_angle);  // Move Servo 2

//     // Increment and publish counter on topic "youssef"
//     static int counter = 0;
//     counter++;
//     pub_msg.data = counter;
//     RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
//   }
// }

// void init_micro_ros() {
//   // Initialize transports and wait for the agent.
//   set_microros_transports();
//   while (!rmw_uros_ping_agent(100, 5)) {
//     // Busy-wait until the agent is reachable.
//     delay(100);
//   }
//   connection_active = true;
  
//   // Initialize the message (and its sequence) properly.
//   std_msgs__msg__Float32MultiArray__init(&servo_msg);
//   // For example, we expect 2 values for two servos:
//   servo_msg.data.capacity = 256;
//   servo_msg.data.size = 256;
//   static float data_buffer[256];
//   servo_msg.data.data = data_buffer;
  
//   allocator = rcl_get_default_allocator();

//   RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
//   RCCHECK(rclc_node_init_default(&node, "micro_ros", "", &support));
//   RCCHECK(rclc_subscription_init_best_effort(
//     &subscriber,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
//     "servo_commands"));

//   // Initialize the publisher to "youssef"
//   RCCHECK(rclc_publisher_init_best_effort(
//     &publisher,
//     &node,
//     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
//     "youssef"));
//   // Initialize the publisher message. (This simply sets pub_msg.data to 0.)
//   pub_msg.data = 0;

//   RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
//   RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &servo_msg, &subscription_callback, ON_NEW_DATA));

//   micro_ros_initialized = true;
//   printf("micro-ROS initialized successfully.\n");
// }

// void cleanup_micro_ros() {
//   // Finalize/cleanup all micro-ROS objects if they have been initialized.
//   if (micro_ros_initialized) {
//     rclc_executor_fini(&executor);
//     rcl_publisher_fini(&publisher, &node);
//     rcl_node_fini(&node);
//     rclc_support_fini(&support);
//     // Optionally, also call subscription fini if available.
//     micro_ros_initialized = false;
//     printf("micro-ROS cleaned up.\n");
//   }
// }

// void setup() {
//   //maybeResetOnce();

//   init_micro_ros();

//   servo1.attach(SERVO1_PIN);
//   servo2.attach(SERVO2_PIN);
// }

// void loop() {
//   if (!rmw_uros_ping_agent(100, 5)) {
//     if (connection_active) {
//       printf("Connection lost! Cleaning up micro-ROS...\n");
//       connection_active = false;
//       lastConnectionLossTime = millis();

//       //set_microros_transports();
//       cleanup_micro_ros();
//     }
//     else {
//       // If already lost, check if we've exceeded the watchdog timeout.
//       if (millis() - lastConnectionLossTime > WATCHDOG_TIMEOUT) {
//         printf("Watchdog timeout reached. System resetting...\n");
//         // Reset the system (for ARM-based controllers like Teensy)
//         //NVIC_SystemReset();
//         //resetBoard();
//       }
//     }
//   } else {
//     if (!connection_active) {
//       printf("Connection reestablished. Reinitializing micro-ROS...\n");
//       init_micro_ros();
//       connection_active = true;
//     }
//   }
//    if (micro_ros_initialized) {
//     // Spin the executor to handle subscriptions.
//     rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//   }
  
//   delay(100);
// }
