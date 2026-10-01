#include "protocol.h"
#include <string.h>


uint16_t Protocol_CRC16(const uint8_t *data,
                        uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}


uint16_t Protocol_BuildFrame(uint8_t *buffer,
                             uint8_t dst,
                             uint8_t src,
                             uint8_t cmd,
                             uint8_t seq,
                             const uint8_t *payload,
                             uint8_t payload_len)
{
    if (payload_len > PROTO_MAX_PAYLOAD)
    {
        return 0;
    }

    buffer[0] = PROTO_SOF1;
    buffer[1] = PROTO_SOF2;

    buffer[2] = dst;
    buffer[3] = src;

    buffer[4] = cmd;
    buffer[5] = seq;

    buffer[6] = payload_len;

    if ((payload != NULL) && (payload_len > 0))
    {
        memcpy(&buffer[7],
               payload,
               payload_len);
    }

    /*
     * CRC calculate from DST:
     *
     * DST
     * SRC
     * CMD
     * SEQ
     * LEN
     * DATA
     */

    uint16_t crc =
        Protocol_CRC16(&buffer[2],
                       5 + payload_len);

    buffer[7 + payload_len] =
        (uint8_t)(crc & 0xFF);

    buffer[8 + payload_len] =
        (uint8_t)((crc >> 8) & 0xFF);

    return 9 + payload_len;
}


uint8_t Protocol_DecodeFrame(const uint8_t *buffer,
                             uint16_t length,
                             ProtocolFrame *frame)
{
    if (length < 9)
    {
        return 0;
    }

    if ((buffer[0] != PROTO_SOF1) ||
        (buffer[1] != PROTO_SOF2))
    {
        return 0;
    }

    uint8_t payload_len = buffer[6];

    if (payload_len > PROTO_MAX_PAYLOAD)
    {
        return 0;
    }

    uint16_t expected_length =
        9 + payload_len;

    if (length != expected_length)
    {
        return 0;
    }

    uint16_t received_crc;

    received_crc =
        buffer[7 + payload_len];

    received_crc |=
        ((uint16_t)buffer[8 + payload_len] << 8);


    uint16_t calculated_crc =
        Protocol_CRC16(&buffer[2],
                       5 + payload_len);


    if (received_crc != calculated_crc)
    {
        return 0;
    }


    frame->dst = buffer[2];
    frame->src = buffer[3];

    frame->cmd = buffer[4];
    frame->seq = buffer[5];

    frame->len = payload_len;


    if (payload_len > 0)
    {
        memcpy(frame->payload,
               &buffer[7],
               payload_len);
    }

    return 1;
}
