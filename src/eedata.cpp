#include "eedata.h"

/*

EEPROM wear leveling

Adapted from: https://ww1.microchip.com/downloads/en/Appnotes/doc2526.pdf

This scheme uses 2 buffer areas in the (emulated) EEPROM memory space:
- address buffer ("status buffer" in app note)
- data buffer ("paramater buffer" in app note)

Each time data is written:
- The address is incremented and a 1:0 sequence is written to the address buffer at the new index
- The data is written at the corresponding index in the data buffer

When reading the data
- First, the last write address is found by looking for 1:0 sequence in the address buffer
- Then the data is read from the corresponding address
- Subsequent writes start from that address
- Address wraps within the available buffer size

NOTE:
- total EEPROM size is 1024 bytes on the STM32F103

*/

int find_last_address (void);

int address = 0;
uint8_t data_size = sizeof (dashData_t);

void EEDATA_load (dashData_t *data)
{
    // find the last written address
    address = find_last_address();

    #ifdef DEBUG
        DebugSerial.printf ("found address: %d\n", address);
    #endif

    // read the data
    EEPROM.get (DATA_BUFFER_START + address * data_size, *data);

    #ifdef DEBUG
        DebugSerial.printf ("read: %d from address %d at offset %d\n", data->soc, address, DATA_BUFFER_START + address * data_size);
    #endif
}

// Save the data and update the address buffer
// TODO: Make sure this only writes if data has changed 
void EEDATA_save (dashData_t *data)
{
    int next_addr;

    // increment & wrap the address
    address = ++address % BUFFER_SIZE;
    next_addr = (address + 1) % BUFFER_SIZE;

    // set EEPROM to only write on commit
    // only really needs to be set once but doing it here saves an init() function
    EEPROM.setCommitASAP (false);

    // write the data
    EEPROM.put (DATA_BUFFER_START + address * data_size, *data);

    #ifdef DEBUG
        DebugSerial.printf ("wrote: %d to address %d at offset %d\n", data->soc, address, DATA_BUFFER_START + address * data_size);
    #endif

    // write the 1:0 sequence to the address buffer
    EEPROM.write (ADDR_BUFFER_START + address, 1);
    EEPROM.write (ADDR_BUFFER_START + next_addr, 0);

    // commit to EEPROM
    EEPROM.commit();

    #ifdef DEBUG
        //DebugSerial.printf ("wrote 1:0 to %d:%d\n", ADDR_BUFFER_START + address, ADDR_BUFFER_START + next_addr);
    #endif

}

// search through the address buffer to find the last data write address
// this is marked by a 1 representing the last data address, followed by a 0
int find_last_address ()
{
    int idx = 0;
    uint8_t this_addr, next_addr;

    this_addr = EEPROM.read (ADDR_BUFFER_START + idx);
    next_addr = EEPROM.read (ADDR_BUFFER_START + idx + 1);

    while (!((this_addr == 1) && (next_addr  == 0)))
    {
        #ifdef DEBUG
            //DebugSerial.printf ("idx: %d this: %d next: %d\n", idx, this_addr, next_addr);
        #endif

        // escape hatch: no previous address found; use the beginning of the buffer
        if (++idx >= BUFFER_SIZE)
            return (ADDR_BUFFER_START);

        this_addr = next_addr; // shortcut for EEPROM.read(idx) as idx has incremented;
        next_addr = EEPROM.read (ADDR_BUFFER_START + (idx + 1) % BUFFER_SIZE); // wrap at end of buffer
    }
    
    return (ADDR_BUFFER_START + idx);
}
