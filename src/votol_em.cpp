#include "votol_em.h"

HardwareSerial VotolSerial(VOTOL_USART);
VOTOL_ResponseData_t VOTOL_Response;
uint8_t VOTOL_Buffer[VOTOL_BUFFER_SIZE];

void VOTOL_send_request (const char* req) // TODO: convert this to VOTOL_Request_t? If so make it also a union
{
    // send the serial request
    // then wait for the response in the main loop by polling the serial port
    // TODO: detect a timeout on this? 
    VotolSerial.write (req);
}

void VOTOL_flush_rx (int maxBytes)
{
    int cnt = 0;
        
    delayMicroseconds(100); // need to wait a little for first byte .. note 1 byte takes ~87us @ 115200

    // flush the buffer
    // 'Soft' flush: just read until no more bytes
    while (VotolSerial.available() && (cnt < maxBytes))
    {
        //VotolSerial.readBytes(VOTOL_Buffer, sizeof (VOTOL_Buffer)); // stash in buffer, or ..
        VotolSerial.read(); // just discard NOTE only seems to read one byte?
        delayMicroseconds(100); // need to wait a little for next byte .. note 1 byte takes ~87us @ 115200
        cnt++;
    }

    #ifdef DEBUG
        DebugSerial.printf ("Flushed %d of %d\r\n", cnt, maxBytes);
    #endif

    // 'HARD' flush; stop the packets getting out of synch if junk data is coming in
    //VotolSerial.end ();
    //VotolSerial.begin (VOTOL_UART_BAUD);
}

// Check a response to see if it is valid before using data
// TODO: calculate checksum? XOR first 22 bytes, should equal checksum (byte 23)
// TODO: Check against a const header array not hard coded here?
// Packet with weird temps:  c0 14 d 59 42 2 e8 0 0 0 0 0 0 0 0 0 92 4b 0 0 2 0 f3 d  (96C, 25C)
// Valid packet - bug in Votol presumably
bool VOTOL_check_response (uint8_t *data)
{
    return ((data[0] == 0xC0) && (data[1] == 0x14) && (data[2] == 0x0D) && (data[3] == 0x59) && (data[4] == 0x42));
}

bool VOTOL_check_external_read (uint8_t *data)
{
    return ((data[0] == 0xC9) && (data[1] == 0x14) && (data[2] == 0x02) && (data[3] == 0x4C) && (data[4] == 0x44));
}

uint16_t VOTOL_get_volts (VOTOL_Response_t *resp)
{
    return ((uint16_t)(resp->voltage_h << 8) + resp->voltage_l);
}

int16_t VOTOL_get_amps (VOTOL_Response_t *resp)
{
    return ((int16_t)(resp->current_h << 8) + resp->current_l);
}

/*
float VOTOL_get_volts (VOTOL_Response_t *resp)
{
    return ((float)((resp->voltage_h << 8) + resp->voltage_l) / 10);
}

float VOTOL_get_amps (VOTOL_Response_t *resp)
{
    return ((float)((resp->current_h << 8) + resp->current_l) / 10);
}
*/

int16_t VOTOL_get_contr_temp (VOTOL_Response_t *resp)
{
    return ((int16_t)resp->temp_cont - 50);
}

// on startup, the controller sends junk temp data for a little while
// usually this shows up as controller temp 96C
bool VOTOL_check_valid_temp (VOTOL_Response_t *resp)
{
    return (resp->temp_cont != (96 + 50));
}

int16_t VOTOL_get_motor_temp (VOTOL_Response_t *resp)
{
    return ((int16_t)resp->temp_ext - 50);
}

uint16_t VOTOL_get_rpm (VOTOL_Response_t *resp)
{
    return ((uint16_t)(resp->rpm_h << 8) + resp->rpm_l);
}

bool VOTOL_get_sport_mode (VOTOL_Response_t *resp)
{
    return ((resp->status_1 & VOTOL_STATUS_1_MASK_SPORT) == VOTOL_STATUS_1_MASK_SPORT);
}

bool VOTOL_get_fault (VOTOL_Response_t *resp)
{
    return (resp->status_2 == Fault);
}

bool VOTOL_get_regen_status (VOTOL_Response_t *resp)
{
    return ((resp->status_1 & VOTOL_STATUS_1_MASK_REGEN) == VOTOL_STATUS_1_MASK_REGEN);
}


