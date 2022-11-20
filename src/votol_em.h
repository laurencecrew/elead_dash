#ifndef VOTOL_EM_H
#define VOTOL_EM_H

#include <Arduino.h>
#include "pins.h"

#define VOTOL_UART_BAUD 115200

#define VOTOL_BUFFER_SIZE 24 // bytes; min is normal packet size (24 bytes) but a larger buffer helps to clear junk data

// Request packet
// note this is when in local mode (controller inputs, not serial input, controls parameters)
// this is set in byte [12] AA for local, 55 for remote
const char VOTOL_Request_Local[] = {0xC9, 0x14, 0x02, 0x53, 0x48, 0x4F, 0x57, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0xAA, 0x00, 0x00, 0x00,
                                    0x09, 0xAA, 0x05, 0xA1, 0x00, 0x2C, 0x5D, 0x0D};

// Disarm packet
// use remote mode to disable movement, e.g. when in charging state
// this is set in byte [12] 55 for remote; 2x in byte [15] for lock/park
// TODO: Capture correct packet
const char VOTOL_Request_Disarm[] = {0xC9, 0x14, 0x02, 0x53, 0x48, 0x4F, 0x57, 0x00,
                                    0x00, 0x00, 0x00, 0x00, 0x55, 0x10, 0xF4, 0x22,
                                    0x09, 0xAA, 0x05, 0xA1, 0x00, 0x2C, 0x64, 0x0D};

// Sent when the Votol software 'Connect' function is executed
// retrieves approx 3 packets of generaal data / settings
// not required to initiate communication with the controller
/*const char VOTOL_Request_Connect[] = {0xC9, 0x14, 0x02, 0x4C, 0x44, 0x47, 0x45, 0x54,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                      0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x81, 0x0D};
*/

// values for initalising the data buffer on startup
// temperature values are 50 which is the offset from 0
const char VOTOL_Buffer_init[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                                  50, 50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*
typedef struct
{
    const uint8_t header_0 = 0xC9;    // Header, request
    const uint8_t header_l = 0x14;    // 
    const uint8_t header_2 = 0x02;    // Mode? 02 53 48 = Display (Tab in software)?
    const uint8_t header_3 = 0x53;    // 
    const uint8_t header_4 = 0x48;    //
    uint8_t showerror_h = 0x4F;       // 4F 57 = Show error? or 45 31, 45 32
    uint8_t showerror_l = 0x57;       // 
    const uint8_t pad_0 = 0x00;       // Padding / null
    const uint8_t pad_1 = 0x00;       // Padding / null
    const uint8_t pad_2 = 0x00;       // Padding / null
    const uint8_t pad_3 = 0x00;       // Padding / null
    const uint8_t pad_4 = 0x00;       // Padding / null
    uint8_t control = 0xAA;           // AA = Local, 55 = Remote
    
    uint8_t throttle_h = 0x11;        // Throttle ratio = throttle value * 5.52 / 32768
    uint8_t throttle_l = 0xE2;        // 
    uint8_t status_1 = 0x00;          // Brake[7] Reverse[6] Lock[5] Switch[3-4] ??[2] Gear[0-1] L/M/H/S
    uint8_t field_weaken = 0x09;      // Field weaking
    uint8_t calibration = 0xAA;       // 55 = Calibration mode; AA = not?
    uint8_t volcal_h = 0x05;          // Voltage Calibration Hi byte
    uint8_t volcal_l = 0xA1;          // Voltage Calibration Lo byte
    uint8_t curcal_h = 0x00;          // Current Calibration Hi byte
    uint8_t curcal_l = 0x2C;          // Current Calibration Lo byte
    uint8_t checksum = 0xAE;          // Checksum (XOR of first 22 bytes)
    uint8_t footer = 0x0D;		      // Terminator
} VOTOL_Request_t;
*/

// Response packet
typedef struct
{
    uint8_t header_0;		// C0 Header, response
    uint8_t header_1;		// 14
    uint8_t header_2;       // 0D
    uint8_t header_3;       // 59
    uint8_t header_4;       // 42
    uint8_t voltage_h;      // Motor voltage, fixed pt dec, Hi byte
    uint8_t voltage_l;      // Motor voltage, fixed pt dec, Lo byte
    int8_t current_h;       // Motor current, signed fixed pt dec, Hi byte
    uint8_t current_l;      // Motor current, signed fixed pt dec, Lo byte NOTE only hi byte should be signed!!
    uint8_t unknown;        // ?? 02 Example shows 00
    uint8_t fault_0;        // Fault code? .. 00
    uint8_t fault_1;        // Fault code? .. 00
    uint8_t fault_2;        // Fault code? .. 00
    uint8_t fault_3;        // Fault code? Brake off = 00, Brake on = 80 - is this a fault? Check software checkboxes
    uint8_t rpm_h;          // Motor RPM, int, Hi byte
    uint8_t rpm_l;          // Motor RPM, int, Low byte
    uint8_t temp_cont;      // Controller temp, dec+50, degrees C
    uint8_t temp_ext;       // External temp, dec+50, degrees C // Motor temp??
    uint8_t temp_coeff_h;   // Temperature coefficient? Hi byte
    uint8_t temp_coeff_l;   // Temperature coefficient? Lo byte
    uint8_t status_1;       // gear[0-1] Reverse[2] Park[3] Brake[4] antitheft[5] side stand[6] regen[7] note 'gear' = L/M/H/Sport
    uint8_t status_2;       // 0=idle 1=init 2=start 3=run 4=stop 5=brake 6=wait 7=fault
    uint8_t checksum;		// Checksum (XOR of first 22 bytes)
    uint8_t footer;		    // 0D; Terminator
} VOTOL_Response_t;

typedef union 
{
    VOTOL_Response_t resp;
    char data[sizeof resp];
} VOTOL_ResponseData_t;

// Status 1 bit fields
#define VOTOL_STATUS_1_MASK_SPORT   0b00000011
#define VOTOL_STATUS_1_MASK_REGEN   0b10000000

// status 2 enum
enum VOTOL_Status_2 {Idle = 0, Init = 1, Start = 2, Run = 3, Stop = 4, Brake = 5, Wait = 6, Fault = 7}; 

void VOTOL_send_request (const char*);
void VOTOL_flush_rx (void);
bool VOTOL_check_response (uint8_t*);
bool VOTOL_check_external_read (uint8_t *);
uint16_t VOTOL_get_volts (VOTOL_Response_t*);
int16_t VOTOL_get_amps (VOTOL_Response_t*);
//float VOTOL_get_volts (VOTOL_Response_t*);
//float VOTOL_get_amps (VOTOL_Response_t*);
int16_t VOTOL_get_contr_temp (VOTOL_Response_t*);
int16_t VOTOL_get_motor_temp (VOTOL_Response_t*);
uint16_t VOTOL_get_rpm (VOTOL_Response_t*);
bool VOTOL_get_sport_mode (VOTOL_Response_t*);
bool VOTOL_get_fault (VOTOL_Response_t*);
bool VOTOL_get_regen_status (VOTOL_Response_t*);
bool VOTOL_check_valid_temp (VOTOL_Response_t*);

extern HardwareSerial VotolSerial;
extern VOTOL_ResponseData_t VOTOL_Response;
extern uint8_t VOTOL_Buffer[VOTOL_BUFFER_SIZE];


#endif