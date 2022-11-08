
#include "can_lights.h"
#include "can.h"
#include "debug.h"

void LIGHTS_send_data (LIGHTS_Data_t *lights)
{
    CAN_msg_t msg;

    //msg.id = ;
    msg.id = LIGHTS_CAN_ID;
    msg.data[0] = lights->data;
    msg.data[1] = 0x00;
    msg.data[2] = 0x00;
    msg.data[3] = 0x00;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;
    msg.len = LIGHTS_DATA_LEN;
    msg.format =  STANDARD_FORMAT; // EXTENDED_FORMAT; //
    msg.type = DATA_FRAME; // REMOTE_FRAME;
    
    #ifdef DEBUG
    //    DebugSerial.print ("Send ID: ");
    //    DebugSerial.println (msg.id, HEX);
    //    DebugSerial.print ("Send data[0]: ");
    //    DebugSerial.println (msg.data[0]);
    #endif
    
    CANSend (&msg);
}
