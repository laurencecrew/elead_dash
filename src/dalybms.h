#ifndef DALYBMS_H
#define DALYBMS_H

#include <Arduino.h>

#define BMS_CAN_PRIORITY        0x18
#define BMS_HOST_ID             0x40
#define BMS_CAN_ID_1            0x01 // first BMS unit in system
#define BMS_CAN_ID_2            0x02 // second BMS unit in system

#define BMS_DATA_ID_SOC         0x90
#define BMS_DATA_ID_MINMAXTEMP  0x92
#define BMS_DATA_ID_CHARGING    0x93
#define BMS_DATA_ID_STATUS      0x94
#define BMS_DATA_ID_TEMP        0x96
#define BMS_DATA_ID_FAULT       0x98


// CAN ID for data request
typedef struct
{
  uint8_t priority;           // CAN priority level, fixed
  uint8_t data_id;            // Set the data ID you want to query
  uint8_t dest_id;            // ID of message receiver e.g. 0x01 for BMS, 0x40 for host
  uint8_t source_id;          // ID of message sender e.g. 0x01 for BMS, 0x40 for host
} BMS_CAN_Id_t;

// Data report for SOC (ID 0x90)
typedef struct
{
  uint8_t pressure_h;         // Discharge voltage? (0.1V)
  uint8_t pressure_l;         // Discharge voltage? (0.1V)
  uint8_t acquisition_h;      // Charge voltage? (0.1V)
  uint8_t acquisition_l;      // Charge voltage? (0.1V)
  int8_t current_h;           // Current, 0.1A, 30000 offset NOTE Hi byte signed(?)
  uint8_t current_l;          // Current, 0.1A, 30000 offset
  uint8_t soc_h;              // State of charge (0.1%)
  uint8_t soc_l;              // State of charge (0.1%)
} BMS_SOC_Report_t;

// Data report for max / min temperature (ID 0x92)
typedef struct
{
  uint8_t max_temp;           // Maximum temperature, deg C, 40 deg offset
  uint8_t max_temp_no;        // Temperature sensor number for maximum e.g. 1
  uint8_t min_temp;           // Minimum temperature, deg C, 40 deg offset
  uint8_t min_temp_no;        // Temperature sensor number for minimum e.g. 1
  uint8_t byte_4;             // unused
  uint8_t byte_5;             // unused
  uint8_t byte_6;             // unused
  uint8_t byte_7;             // unused
} BMS_MaxMinTemp_Report_t;

// Data report for charge / discharge / MOS status (ID 0x93)
typedef struct
{
  uint8_t charge_status;        // 0 stationary, 1 charged, 2 discharged
  uint8_t charge_MOS_status;    // 'charging MOS tube status'
  uint8_t discharge_MOS_status; // 'discharge MOS tube status'
  uint8_t BMS_life;             // BMS life(0~255 cycles)
  uint8_t capacity_3;           // Residual capacity of pack (mA, 4 bytes)
  uint8_t capacity_2;           // 
  uint8_t capacity_1;           // 
  uint8_t capacity_0;           // 
} BMS_Charging_Report_t;

#define BMS_CHARGE_STATUS_STATIONARY  0
#define BMS_CHARGE_STATUS_CHARGE      1
#define BMS_CHARGE_STATUS_DISCHARGE   2

// Data report for Status information (ID 0x94)
typedef struct
{
  uint8_t no_of_cells;        // 'battery string'
  uint8_t no_of_temps;        // 'temperatures'
  uint8_t charger_status;     // 0 disconnected, 1 connected
  uint8_t load_status;        // 0 disconnected, 1 access
  uint8_t dio_state;          // bits 0-3: DI1-4 state; bits 4-7: DO 1-4 state: Date in/out??
  uint8_t byte_5;             // unused
  uint8_t byte_6;             // unused
  uint8_t byte_7;             // unused
} BMS_Status_Report_t;

// Data report for all temperatures (ID 0x96)
typedef struct
{
  uint8_t frame_no;           // Frame number (up to 3 frames) but generally '1' in this instance! (only 1 temp to report)
  uint8_t temp_1;             // Temperature number 1, deg C, 40 deg offset
  uint8_t temp_2;             // Temperature number 2, deg C, 40 deg offset
  uint8_t temp_3;             // Temperature number 3, deg C, 40 deg offset
  uint8_t temp_4;             // Temperature number 4, deg C, 40 deg offset
  uint8_t temp_5;             // Temperature number 5, deg C, 40 deg offset
  uint8_t temp_6;             // Temperature number 6, deg C, 40 deg offset
  uint8_t temp_7;             // Temperature number 7, deg C, 40 deg offset
} BMS_Temp_Report_t;

// Data report for faults (ID 0x98)
typedef struct
{
  uint8_t byte_0;
  uint8_t byte_1;
  uint8_t byte_2;
  uint8_t byte_3;
  uint8_t byte_4;
  uint8_t byte_5;
  uint8_t byte_6;
  uint8_t byte_7;
} BMS_Fault_Report_t;

/*
Fault report bit fields:
Byte 0
Bit 0: one stage warning of unit over voltage
Bit 1: one stage warning of unit over voltage
Bit 2: one stage warning of unit over voltage
Bit 3: two stage warning of unit over voltage
Bit 4: Total voltage is too high One alarm
Bit 5: Total voltage is too high Level two alarm
Bit 6: Total voltage is too low One alarm
Bit 7: Total voltage is too low Level two alarm

Byte 1
Bit 0: Charging temperature too high. One alarm
Bit 1: Charging temperature too high. Level two alarm
Bit 2: Charging temperature too low. One alarm
Bit 3: Charging temperature's too low. Level two alarm
Bit 4: Discharge temperature is too high. One alarm
Bit 5: Discharge temperature is too high. Level two alarm
Bit 6: Discharge temperature is too low. One alarm
Bit 7: Discharge temperature is too low. Level two alarm

Byte 2
Bit 0: Charge over current. Level one alarm
Bit 1: Charge over current, level two alarm
Bit 2: Discharge over current. Level one alarm
Bit 3: Discharge over current, level two alarm
Bit 4: SOC is too high an alarm
Bit 5: SOC is too high. Alarm Two
Bit 6: SOC is too low. level one alarm
Bit 7: SOC is too low. level two alarm

Byte 3
Bit 0: Excessive differential pressure level one alarm
Bit 1: Excessive differential pressure level two alarm
Bit 2: Excessive temperature difference level one alarm
Bit 3: Excessive temperature difference level two alarm

Byte 4
Bit 0: charging MOS overtemperature warning
Bit 1: discharge MOS overtemperature warning
Bit 2: charging MOS temperature detection sensor failure
Bit 3: discharge MOS temperature detection sensor failure
Bit 4: charging MOS adhesion failure
Bit 5: discharge MOS adhesion failure
Bit 6: charging MOS breaker failure
Bit 7: discharge MOS breaker failure

Byte 5
Bit 0: AFE acquisition chip malfunction
Bit 1: monomer collect drop off
Bit 2: Single Temperature Sensor Fault
Bit 3: EEPROM storage failures
Bit 4: RTC clock malfunction
Bit 5: Precharge Failure
Bit 6: vehicle communications malfunction
Bit 7: intranet communication module malfunction

Byte 6
Bit 0: Current Module Failure
Bit 1: main pressure detection module
Bit 2: Short circuit protection failure
Bit 3: Low Voltage No Charging
Bit 4: MOS GPS or soft switch MOS off
Bit 5~Bit7: Reserved

Byte 7：fault code (if 0 x 03, show "fault code 3", 0 do not show)
*/


const BMS_CAN_Id_t BMS_soc_id_1 = {BMS_CAN_PRIORITY, BMS_DATA_ID_SOC, BMS_HOST_ID, BMS_CAN_ID_1};
//const BMS_CAN_Id_t BMS_soc_id_2 = {BMS_CAN_PRIORITY, BMS_DATA_ID_SOC, BMS_HOST_ID, BMS_CAN_ID_2};
const BMS_CAN_Id_t BMS_charging_id_1 = {BMS_CAN_PRIORITY, BMS_DATA_ID_CHARGING, BMS_HOST_ID, BMS_CAN_ID_1};
//const BMS_CAN_Id_t BMS_charging_id_2 = {BMS_CAN_PRIORITY, BMS_DATA_ID_CHARGING, BMS_HOST_ID, BMS_CAN_ID_2};
const BMS_CAN_Id_t BMS_status_id_1 = {BMS_CAN_PRIORITY, BMS_DATA_ID_STATUS, BMS_HOST_ID, BMS_CAN_ID_1};
//const BMS_CAN_Id_t BMS_status_id_2 = {BMS_CAN_PRIORITY, BMS_DATA_ID_STATUS, BMS_HOST_ID, BMS_CAN_ID_2};
const BMS_CAN_Id_t BMS_temp_id_1 = {BMS_CAN_PRIORITY, BMS_DATA_ID_TEMP, BMS_HOST_ID, BMS_CAN_ID_1};
//const BMS_CAN_Id_t BMS_temp_id_2 = {BMS_CAN_PRIORITY, BMS_DATA_ID_TEMP, BMS_HOST_ID, BMS_CAN_ID_2};
const BMS_CAN_Id_t BMS_fault_id_1 = {BMS_CAN_PRIORITY, BMS_DATA_ID_FAULT, BMS_HOST_ID, BMS_CAN_ID_1};
//const BMS_CAN_Id_t BMS_fault_id_2 = {BMS_CAN_PRIORITY, BMS_DATA_ID_FAULT, BMS_HOST_ID, BMS_CAN_ID_2};


void BMS_request_data (uint8_t, uint8_t);
uint32_t BMS_ParseId (BMS_CAN_Id_t);
uint16_t BMS_get_soc (BMS_SOC_Report_t*);
uint16_t BMS_get_volts (BMS_SOC_Report_t*);
int16_t BMS_get_amps (BMS_SOC_Report_t*);
//float BMS_get_soc (BMS_SOC_Report_t*);
//float BMS_get_volts (BMS_SOC_Report_t*);
//float BMS_get_amps (BMS_SOC_Report_t*);
int BMS_get_temp (BMS_Temp_Report_t*);
bool BMS_get_fault (BMS_Fault_Report_t*);
bool BMS_get_charge_status (BMS_Charging_Report_t*);

#endif