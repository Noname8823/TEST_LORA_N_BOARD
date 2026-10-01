#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

#define PROTO_SOF1              0xAA
#define PROTO_SOF2              0x55

#define PROTO_MAX_PAYLOAD       32

#define DEVICE_ADDR_MASTER      0x00
#define DEVICE_ADDR_STM32       0x01

#define CMD_PING                0x01
#define CMD_GET_INPUTS          0x10

#define CMD_PING_RESPONSE       0x81
#define CMD_INPUTS_RESPONSE     0x90


typedef struct
{
    uint8_t dst;
    uint8_t src;

    uint8_t cmd;
    uint8_t seq;

    uint8_t len;

    uint8_t payload[PROTO_MAX_PAYLOAD];

} ProtocolFrame;


uint16_t Protocol_CRC16(const uint8_t *data,
                        uint16_t len);


uint16_t Protocol_BuildFrame(uint8_t *buffer,
                             uint8_t dst,
                             uint8_t src,
                             uint8_t cmd,
                             uint8_t seq,
                             const uint8_t *payload,
                             uint8_t payload_len);


uint8_t Protocol_DecodeFrame(const uint8_t *buffer,
                             uint16_t length,
                             ProtocolFrame *frame);

#endif
