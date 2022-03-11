#include "vesc_can.h"
#include "buffer.h"
#include "datatypes.h"
#include <FlexCAN_T4.h>



// Global CANBUS variables
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> CAN;

//Can processing examples
int can_idle = 0;

can_status_msg status_msg;
can_status_msg_2 status_msg_2;
can_status_msg_3 status_msg_3;
can_status_msg_4 status_msg_4;
can_status_msg_5 status_msg_5;

// Helper function
bool can_send_packet(uint32_t id, uint8_t packet[], int32_t len) {
  CAN_message_t  msg;
  msg.flags.extended = true;
  msg.id = id;
  msg.len = len;
  memcpy(msg.buf, packet, len * sizeof(uint8_t));
  //msg.timeout = 0;
  return (bool) CAN.write(msg);
}

void vesc_can_begin() {
  CAN.begin();
  CAN.setBaudRate(250000);
}

bool vesc_can_read() {
  CAN_message_t  inMsg;
  unsigned int rxbuf_len;
  unsigned int rxbuf_ind;
  uint8_t crc_low;
  uint8_t crc_high;

  int32_t ind;
  if (!CAN.read(inMsg)) {
    return false; // if the message cant be read return 0
  }
  if (inMsg.flags.extended) {
    uint8_t id = inMsg.id & 0xFF; //take the lower 8 bits for the ID
    CAN_PACKET_ID cmd = (CAN_PACKET_ID) (inMsg.id >> 8); // Take the upper bits as the comand
    switch (cmd) {
        ind = 0;
        status_msg.id = id;
        status_msg.rpm = (float)buffer_get_int32(inMsg.buf, &ind);
        status_msg.current = (float)buffer_get_int16(inMsg.buf, &ind) / 10.0;
        status_msg.duty = (float)buffer_get_int16(inMsg.buf, &ind) / 1000.0;
        break;
      default:
        break;
    }
  }
  return true;
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




