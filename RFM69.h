#ifndef RFM69_H
#define RFM69_H

#ifdef __AVR__
#include <Arduino.h>
#else
#include <c_types.h>
#endif

#include "RFM69_registers.h"

#ifndef RFM69_MAX_PACKET_LENGTH
#define RFM69_MAX_PACKET_LENGTH 60
#endif // RFM69_MAX_PACKET_LENGTH

#define RFM69_HISTORY_SIZE 3
#define DEBUGLEVEL 1

typedef void (*RFM69_receive_handler)(uint8_t sender, int8_t rssi, uint8_t * packet, uint8_t len);
enum RFM69_STATE { STBY, RECV, ACK_WAIT, SEND_LBT, SEND_WAIT, SEND_WAIT_ACK };
enum RFM69_MODE { RFM69_STANDBY, RFM69_LISTEN, RFM69_RECEIVE };

struct RFM69ReceiveHistory {
  uint8_t sender;
  uint8_t seq_num;
  unsigned long received;
};

class RFM69
{
  private:
  
  uint16_t m_long_preamble_millis = 1005;
  uint32_t m_frequency = 867500032; // Hz
  uint16_t m_channel_step = 50020; // Hz

  uint8_t m_rst_pin;
  uint8_t m_ss_pin;
  uint8_t m_dio0_pin;
  RFM69_STATE m_state = STBY;
  RFM69_MODE m_default_mode = RFM69_STANDBY;
  
  uint8_t m_recipient;
  uint8_t m_send_seq_num;
  uint8_t m_send_retries;
  uint8_t m_ack_recipient;
  uint8_t m_ack_seq_num;

  uint8_t m_tx_success = false;
  unsigned long m_tx_start = 0;
  unsigned long m_ack_wait_start = 0;

  uint8_t m_ack_buffer[5];
  uint8_t m_packet_buffer[RFM69_MAX_PACKET_LENGTH];
  uint8_t m_packet_length = 0;

  struct RFM69ReceiveHistory m_history[RFM69_HISTORY_SIZE];
  uint8_t m_history_length = 0;
  uint8_t m_history_end = 0;

  int8_t m_rssi_threshold = -105; // dBm
  int8_t m_recv_rssi;
  uint8_t m_rssi;

  uint8_t m_network_id = 100;
  uint8_t m_node_id = 0;
  uint8_t m_channel = 0;

  bool m_high_power = false;
  bool m_ready = false;
  uint8_t m_seq_num = 0;

  RFM69_receive_handler m_receive_handler = NULL;

  public:

  RFM69(uint8_t dio0_pin, uint8_t ss_pin, uint8_t rst_pin);
  
  void begin(uint8_t network_id, uint8_t node_id, uint8_t channel);
  void update();
  
  uint8_t getNetworkId();
  uint8_t getNodeId();
  uint8_t getBroadcastId();
  void setHighPower(bool high_power = true);
  bool isHighPower();
  void setReceiveHandler(RFM69_receive_handler handler);
  void sendMessage(uint8_t recipient, uint8_t * buffer, uint8_t len);
  bool isSending();
  void waitForSend();
  void receive();
  void listen();
  
  private:
  
  void reset();
  void setChannel(uint8_t channel);
  void setFrequency(uint32_t frequency);
  void calibrateOsc();
  void setMode(uint8_t mode);
  uint8_t getMode();
  void writeReg(uint8_t addr, uint8_t val);
  uint8_t readReg(uint8_t addr);
  void select();
  void unselect();
  void setPacketSentDio0Interrupt();
  void setPayloadReadyDio0Interrupt();
  void setRssiDio0Interrupt();
  void sendQueueWriteCommand();
  void sendQueueReadCommand();
  void enableHighPower();
  void disableHighPower();
  bool isModeReady();
  bool isPayloadReady();
  void waitForModeReady();
  void updateRssi();
  int8_t getRssi();
  void transmitMessage();
  void transmitAck();
  bool isChannelFree();
  void beginTransmission();
  bool isTransmissionFinished();
  void endTransmission();
  void receivePacket(uint8_t * buffer, uint8_t * length, uint8_t maxLength, int8_t * rssi);
  void doReceiveCallback();
  bool isInHistory(uint8_t sender, uint8_t seq_num);
  void putInHistory(uint8_t sender, uint8_t seq_num);
  void restoreDefaultMode();
};

#endif // RFM69_H
