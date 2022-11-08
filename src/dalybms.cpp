
#include "dalybms.h"
#include "can.h"
#include "debug.h"

void BMS_request_data (uint8_t dest_id, uint8_t data_id)
{
    BMS_CAN_Id_t id;
    CAN_msg_t msg;

    id.priority = BMS_CAN_PRIORITY;
    id.data_id = data_id;
    id.dest_id = dest_id;
    id.source_id = BMS_HOST_ID;

    //msg.id = ;
    msg.id = BMS_ParseId (id);
    msg.data[0] = 0x00;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    msg.len = 8;
    msg.format = EXTENDED_FORMAT; // STANDARD_FORMAT; // 
    msg.type = DATA_FRAME; // REMOTE_FRAME;
    
    #ifdef DEBUG
        DebugSerial.print ("Send ID: ");
        DebugSerial.println (msg.id, HEX);
    #endif
    
    CANSend (&msg);
}


uint32_t BMS_ParseId (BMS_CAN_Id_t id)
{
    return ((id.priority << 24) + (id.data_id << 16) + (id.dest_id << 8) + id.source_id);
}

uint16_t BMS_get_soc (BMS_SOC_Report_t *rpt)
{
    return ((uint16_t)((rpt->soc_h << 8) + rpt->soc_l) / 10);
}

uint16_t BMS_get_volts (BMS_SOC_Report_t *rpt)
{
    return ((uint16_t)((rpt->pressure_h << 8) + rpt->pressure_l));
}

int16_t BMS_get_amps (BMS_SOC_Report_t *rpt)
{
    return ((int16_t)((rpt->current_h << 8) + rpt->current_l - 30000));
}

/*
float BMS_get_soc (BMS_SOC_Report_t *rpt)
{
    return ((float)((rpt->soc_h << 8) + rpt->soc_l) / 10);
}

float BMS_get_volts (BMS_SOC_Report_t *rpt)
{
    return ((float)((rpt->pressure_h << 8) + rpt->pressure_l) / 10);
}

float BMS_get_amps (BMS_SOC_Report_t *rpt)
{
    return ((float)((rpt->current_h << 8) + rpt->current_l - 30000) / -10);
}
*/

int BMS_get_temp (BMS_Temp_Report_t *rpt)
{
    return ((int)rpt->temp_1 - 40);
}

bool BMS_get_fault (BMS_Fault_Report_t *rpt)
{
    return (rpt->byte_7 != 0); // fault code is 0 when there is no fault
}

bool BMS_get_charge_status (BMS_Charging_Report_t* rpt)
{
    return (rpt->charge_status == BMS_CHARGE_STATUS_CHARGE);
}

