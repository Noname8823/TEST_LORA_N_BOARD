
#include "protocol.h"

#include <string.h>

/* =========================================================
 * CRC16 MODBUS
 * ========================================================= */

uint16_t Protocol_CRC16(
    const uint8_t *data,
    uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL)
    {
        return crc;
    }

    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0U; j < 8U; j++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

/* =========================================================
 * BUILD PROTOCOL FRAME
 *
 * AA 55 DST SRC CMD SEQ LEN PAYLOAD CRC_L CRC_H
 * ========================================================= */

uint16_t Protocol_BuildFrame(
    uint8_t *buffer,
    uint8_t dst,
    uint8_t src,
    uint8_t cmd,
    uint8_t seq,
    const uint8_t *payload,
    uint8_t payload_len)
{
    uint16_t crc;

    if (buffer == NULL)
    {
        return 0U;
    }

    if (payload_len > PROTO_MAX_PAYLOAD)
    {
        return 0U;
    }

    if ((payload_len > 0U) && (payload == NULL))
    {
        return 0U;
    }

    buffer[0] = PROTO_SOF1;
    buffer[1] = PROTO_SOF2;

    buffer[2] = dst;
    buffer[3] = src;
    buffer[4] = cmd;
    buffer[5] = seq;
    buffer[6] = payload_len;

    if (payload_len > 0U)
    {
        memcpy(
            &buffer[7],
            payload,
            payload_len);
    }

    /* CRC covers DST SRC CMD SEQ LEN PAYLOAD */
    crc = Protocol_CRC16(
        &buffer[2],
        (uint16_t)(5U + payload_len));

    buffer[7U + payload_len] =
        (uint8_t)(crc & 0xFFU);

    buffer[8U + payload_len] =
        (uint8_t)((crc >> 8U) & 0xFFU);

    return (uint16_t)(9U + payload_len);
}

/* =========================================================
 * DECODE PROTOCOL FRAME
 * ========================================================= */

uint8_t Protocol_DecodeFrame(
    const uint8_t *buffer,
    uint16_t length,
    ProtocolFrame *frame)
{
    uint8_t payload_len;

    uint16_t expected_length;
    uint16_t received_crc;
    uint16_t calculated_crc;

    if ((buffer == NULL) || (frame == NULL))
    {
        return 0U;
    }

    if (length < 9U)
    {
        return 0U;
    }

    if ((buffer[0] != PROTO_SOF1) ||
        (buffer[1] != PROTO_SOF2))
    {
        return 0U;
    }

    payload_len = buffer[6];

    if (payload_len > PROTO_MAX_PAYLOAD)
    {
        return 0U;
    }

    expected_length =
        (uint16_t)(9U + payload_len);

    if (length != expected_length)
    {
        return 0U;
    }

    received_crc =
        (uint16_t)buffer[7U + payload_len];

    received_crc |=
        (uint16_t)(
            (uint16_t)buffer[8U + payload_len] << 8U);

    calculated_crc = Protocol_CRC16(
        &buffer[2],
        (uint16_t)(5U + payload_len));

    if (received_crc != calculated_crc)
    {
        return 0U;
    }

    frame->dst = buffer[2];
    frame->src = buffer[3];

    frame->cmd = buffer[4];
    frame->seq = buffer[5];

    frame->len = payload_len;

    memset(
        frame->payload,
        0,
        sizeof(frame->payload));

    if (payload_len > 0U)
    {
        memcpy(
            frame->payload,
            &buffer[7],
            payload_len);
    }

    return 1U;
}
