#include "vesc_can.h"
#include "buffer.h"
#include "datatypes.h"
#include <FlexCAN_T4.h>



// Global CANBUS variables
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CAN;

//Can processing examples
int can_idle = 0;

// We have lots of memory on a teensy 4, so eh
node_status nodes_status[MAX_NODE_ID + 1];

// Helper function
bool can_send_packet(uint32_t id, uint8_t packet[], int32_t len) {
  CAN_message_t  msg;
  msg.flags.extended = true;
  msg.id = id;
  msg.len = len;
  memcpy(msg.buf, packet, len);
  //msg.timeout = 0;
  return (bool) CAN.write(msg);
}

void vesc_can_begin() {
  CAN.begin();
  CAN.setBaudRate(250000);
}

bool vesc_can_read() {
  CAN_message_t  msg;
  if (!CAN.read(msg)) {
    return false; // if the message cant be read return 0
  }


  if (msg.flags.extended) {
    uint8_t id = msg.id & 0xFF; //take the lower 8 bits for the ID
    CAN_PACKET_ID cmd = (CAN_PACKET_ID) (msg.id >> 8); // Take the upper bits as the comand

    int32_t ind;
    switch (cmd) {
    case CAN_PACKET_STATUS: 
        ind = 0;
        nodes_status[id].rpm = (float)buffer_get_int32(msg.buf, &ind);
        nodes_status[id].current = (float)buffer_get_int16(msg.buf, &ind) / 10.0;
        nodes_status[id].duty = (float)buffer_get_int16(msg.buf, &ind) / 1000.0;
        break;
    default:
        break;
    }
  }
  return true;
}

float vesc_can_get_erpm(uint8_t controller_id) {
    return nodes_status[controller_id].rpm;
}

float vesc_can_get_rpm(uint8_t controller_id) {
    return nodes_status[controller_id].rpm / MOTOR_POLES;
}

float vesc_can_get_current(uint8_t controller_id) {
    return nodes_status[controller_id].current;
}

float vesc_can_get_duty(uint8_t controller_id) {
    return nodes_status[controller_id].duty;
}

bool vesc_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  return can_send_packet(controller_id | ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}

bool vesc_can_set_current(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  return can_send_packet(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
}

bool vesc_can_set_current_brake(uint8_t controller_id, float current) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
  return can_send_packet(controller_id | ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
}

bool vesc_can_set_erpm(uint8_t controller_id, float erpm) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)erpm, &send_index);
  return can_send_packet(controller_id | ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
}

bool vesc_can_set_rpm(uint8_t controller_id, float rpm) {
    return vesc_can_set_erpm(controller_id, rpm * MOTOR_POLES);
}

bool vesc_can_set_pos(uint8_t controller_id, float pos) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
  return can_send_packet(controller_id | ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
}




