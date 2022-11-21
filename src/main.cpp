/* 
  Honda Lead Electric conversion dash module
  - Read data from Daly BMS and Votol controller, show on OLED displays and fuel gauge
  - Control indicator, brake and tail lights via CANBus
  - CAN driver code adapted from https://github.com/nopnop2002/Arduino-STM32-CAN
  - Use included version of library adafruit/Adafruit SSD1306@^2.5.1 as it has bug fixes / adaptations
  laurence@laurencecrew.com 2022
*/

#include <Arduino.h>
#include "pins.h"
#include "debug.h"
#include "timers.h"
#include "can.h"
#include "display.h"
#include "dalybms.h"
#include "votol_em.h"
#include "can_lights.h"
#include "gauge.h"
#include "trip_stats.h"

// blink states for turn indicators
#define BLINK_MODE_NONE       0
#define BLINK_MODE_LEFT       1
#define BLINK_MODE_RIGHT      2

// operational states
#define STATE_STARTING        0 // no real data yet
#define STATE_IDLE            1 // controller off
#define STATE_WAKEUP          2 // controller waking up
#define STATE_ACTIVE          3 // controller on
#define STATE_CHARGING        4 // charger connected
#define STATE_EXT_CONTROL     5 // external control software connected

// display modes
#define MODE_AMPS             0 // motor amps, real time
#define MODE_TRIP             1 // distance and time TODO
#define MODE_STATS            2 // Wh/km, avg km/h TODO

// timing
// NOTES Votol request / read takes ~5ms
//  BMS read takes 20/40/outlier 60ms to respond, generally <= 15ms for 2 packets
//  Screen update ~ 5ms
//  Votol parameter change 5 successive reads / 500ms
//  Votol software reads every ~100ms; RPM updates fast but current also ~500ms
#define T_TICK            50 // ms = base time / timer interrupt interval; also VOTOL spamming rate in IDLE
#define T_READ            250 / T_TICK // ticks; interval between reading BMS data and VOTOL data in ACTIVE state ALSO updating lights
#define T_VOTOL_TIMEOUT   750 / (T_READ * T_TICK) // Read cycles. If Votol does not respond (e.g. off), go to IDLE state
#define T_BLINK           500 / T_TICK // ticks; on-off blink time for turn indicators
#define T_DEBOUNCE        1000 // us; wait before reading inputs after interrupt 
#define T_CHARGE_TIMEOUT  10 // ticks in IDLE; successful reads in ACTIVE. If charge is detected, wait a few cycles before changing state to make sure


// Globals
volatile bool tick_flag = true; // true on start and every timer cycle
volatile uint8_t read_cnt = 0; // read every n tick cycles
volatile bool read_flag = true; // true on start and whenever a read is due
volatile bool debounce_flag = false; // whenever an input state changes
volatile bool update_lights_flag = true; // true on start and whenever lights are due for update
bool display_init = false; // false on start - make true if displays successfully initialise

volatile uint8_t state = STATE_STARTING;
uint8_t display_mode = MODE_AMPS;
//uint8_t display_mode = MODE_TRIP;
//uint8_t display_mode = MODE_STATS;

// Trip meter and stats
// TODO: Still implementing
int read_t;                   // time (ms) since last read
int last_read_t;              // time (ms) of last read

// keep track of which data has been updated on each read cycle
typedef union
{
  struct
  {
    bool votol:1;
    bool bms_soc_1:1;
    bool bms_temp_1:1;
    bool bms_fault_1:1;
    bool bms_charging_1:1;
    //bool bms_soc_2:1;
    //bool bms_temp_2:1;
    //bool bms_fault_2:1;
  } flags;

  uint8_t all_flags;

} Update_flags_t;
  
#define ALL_UPDATED 0b00011111 // votol and BMS 1
//#define ALL_UPDATED 0b01111111 // votol and BMS 1&2
#define BMS_UPDATED 0b0001110

Update_flags_t update_flags;

// Response packets and data buffers
uint8_t votol_timeout_cnt = 0;
uint8_t charge_timeout_cnt = 0;

// turn signal blinking mode & states
uint8_t blink_mode = BLINK_MODE_NONE, last_blink_mode = BLINK_MODE_NONE;
volatile bool blink_state = false;
volatile uint8_t blink_count = 0;

// function prototypes
void init_dash ();
void update_lights ();

// interrupt handlers
void input_ISR ();
void tick ();

// Get started .. 
void setup ()
{

  // Initialise all pins
  // TODO: put gauge output pin into a neutral(?) state prior to starting PWM later
  pinMode (LIGHTS_IN, INPUT);
  pinMode (TURN_L_IN, INPUT);
  pinMode (TURN_R_IN, INPUT);
  pinMode (BRAKE_IN, INPUT);
  pinMode (MODE_1_IN, INPUT);
  pinMode (MODE_2_IN, INPUT);

  // Input pin interrupts
  //attachInterrupt (digitalPinToInterrupt (LIGHTS_IN), input_ISR, CHANGE); // NOTE: Can't use PB12 and PA12 interrupts as they are on the same 'line' :/
  attachInterrupt (digitalPinToInterrupt (TURN_L_IN), input_ISR, CHANGE);
  attachInterrupt (digitalPinToInterrupt (TURN_R_IN), input_ISR, CHANGE);
  attachInterrupt (digitalPinToInterrupt (BRAKE_IN), input_ISR, CHANGE);
  //attachInterrupt (digitalPinToInterrupt (MODE_1_IN), input_ISR, CHANGE); // For future mode switch
  //attachInterrupt (digitalPinToInterrupt (MODE_2_IN), input_ISR, CHANGE); // For future mode switch

  pinMode (TURN_L_OUT, OUTPUT);
  pinMode (TURN_R_OUT, OUTPUT);
  pinMode (TURN_IND_OUT, OUTPUT);

  digitalWrite (TURN_L_OUT, LOW);
  digitalWrite (TURN_R_OUT, LOW);
  digitalWrite (TURN_IND_OUT, LOW);
  
  // clear data buffers - no junk data
  // may not need this now with state machine? Doesn't hurt
  memcpy (&VOTOL_Response.data, &VOTOL_Buffer_init, sizeof (VOTOL_Buffer_init));

  // TODO: Move this to an init function in votol module?
  VotolSerial.begin (VOTOL_UART_BAUD);

  #ifdef DEBUG
    DebugSerial.begin (115200);
    DebugSerial.println ("Starting ..");
  #endif

  // initialise hardware timer - SetInterval etc
  TMR_Init();

  // initialise dash (start OLEDs, gauge)
  init_dash ();

  #ifdef DEBUG
    DebugSerial.println ("Init CAN ..");
  #endif

  // initialise CAN
  bool ret = CANInit(CAN_250KBPS, CAN_REMAP);  // CAN_RX mapped to PB8, CAN_TX mapped to PB9
  if (!ret) while(true);

  #ifdef DEBUG
    DebugSerial.println ("Done!");
  #endif

  update_flags.all_flags = 0; // reset flags

  // update the lights on start
  // note; need to do this after CAN is initialised
  update_lights ();

  // initialise trip stats
  // in particular, start the RTC
  // note time is set to 0 on entering ACTIVE state
  TRIP_STATS_init ();

  delay (500); // wait a bit for everthing to settle

  // start tick timer
  TMR_SetInterval (tick, T_TICK);

}


void loop()
{
  CAN_msg_t CAN_RX_msg;
  
  // lights need to be updated ..
  if (update_lights_flag)
  {
    update_lights_flag = false; // reset
    update_lights();
  }

  // debounce = input state has changed e.g. light switches
  if (debounce_flag)
  {
      debounce_flag = false; // reset

      // TODO: Need to disable interrupts while debouncing, etc?
      // TODO: Save return state after debounce?
      delayMicroseconds (T_DEBOUNCE);
      update_lights();
  }

  // Check for data on VOTOL "CAN" UART
  // If in External Control state, just ignore it
  // otherwise hard buffer flush disturbs communications
  if (state != STATE_EXT_CONTROL && VotolSerial.available())
  { 
    VotolSerial.readBytes(VOTOL_Buffer, sizeof VOTOL_Response.data);

    // if successful / valid packet
    if (VOTOL_check_response (VOTOL_Buffer))
    {
      memcpy (&VOTOL_Response.data, &VOTOL_Buffer, sizeof (VOTOL_Response.data));
      update_flags.flags.votol = true;
      votol_timeout_cnt = 0;

      #ifdef DEBUG
      /*
        DebugSerial.print ("Controller Volts: ");
        DebugSerial.println (VOTOL_get_volts (&VOTOL_Response.resp));
        DebugSerial.print ("Controller Amps: ");
        DebugSerial.println (VOTOL_get_amps (&VOTOL_Response.resp));
        DebugSerial.print ("Motor Temp: ");
        DebugSerial.println (VOTOL_get_motor_temp (&VOTOL_Response.resp));
        DebugSerial.print ("RPM: ");
        DebugSerial.println (VOTOL_get_rpm (&VOTOL_Response.resp));
        DebugSerial.print ("Cont status 2: ");
        DebugSerial.println (VOTOL_Response.resp.status_2, HEX);
        DebugSerial.print ("Controller status 1: ");
        DebugSerial.println (VOTOL_Response.resp.status_1, HEX);
        DebugSerial.print ("Regen: ");
        DebugSerial.println (VOTOL_get_regen_status (&VOTOL_Response.resp));
        DebugSerial.print ("Controller Temp: ");
        DebugSerial.println (VOTOL_get_contr_temp (&VOTOL_Response.resp));
      */
      #endif
    }
    else if (VOTOL_check_external_read (VOTOL_Buffer))
    {
      // check for external software trying to connect
      // if so, suppress data reading until next restart

      // Enter external control state until next power cycle
      state = STATE_EXT_CONTROL;

      // update display to show connect mode
      draw_display (display1, DISPLAY_MODE_CONNECT, NULL, NULL, NULL, NULL, NULL);
      draw_display (display2, DISPLAY_MODE_CONNECT, NULL, NULL, NULL, NULL, NULL);

      #ifdef DEBUG
        DebugSerial.print ("To state: EXT_CONTROL");
      #endif
    }
    else
    {
      #ifdef DEBUG

        DebugSerial.println ("Bad Votol packet! Flushing ..");

        //dump the data & clear as I go
        DebugSerial.print ("Data: ");
        for (int j=0; j < sizeof (VOTOL_Buffer); j++)
        {
          DebugSerial.printf ("%x ", VOTOL_Buffer[j]);
          VOTOL_Buffer[j] = 0;
        }
        DebugSerial.println("");
              
      #endif

      // flush any remaining data
      VOTOL_flush_rx(sizeof (VOTOL_Buffer));

    }
  }

  // Check for CAN data from BMS(s)
  if(CANMsgAvail())
  {
    CANReceive(&CAN_RX_msg);

    if (CAN_RX_msg.id == BMS_ParseId (BMS_soc_id_1))
    {
      // get data from the CAN packet into the report structure
      memcpy(&BMS_rpt_soc_1, CAN_RX_msg.data, sizeof BMS_rpt_soc_1);
      update_flags.flags.bms_soc_1 = true;

      /*
      #ifdef DEBUG
        DebugSerial.print ("SOC 1: ");
        DebugSerial.println (BMS_get_soc (&BMS_rpt_soc_1));
        DebugSerial.print ("Volts 1: ");
        DebugSerial.println (BMS_get_volts (&BMS_rpt_soc_1));
        DebugSerial.print ("Amps 1: ");
        DebugSerial.println (BMS_get_amps (&BMS_rpt_soc_1));
      #endif
      */
    }
    else if (CAN_RX_msg.id == BMS_ParseId (BMS_temp_id_1))
    {
      // get data from the CAN packet into the report structure
      memcpy(&BMS_rpt_temp_1, CAN_RX_msg.data, sizeof BMS_rpt_temp_1);
      update_flags.flags.bms_temp_1 = true;

      /*
      #ifdef DEBUG
        DebugSerial.print ("Temp 1: ");
        DebugSerial.println (BMS_get_temp(&BMS_rpt_temp_1));
      #endif
      */
    }
    /* don't need this one for now
    else if (CAN_RX_msg.id == BMS_ParseId (BMS_status_id_1))
    {
      // get data from the CAN packet into the report structure
      memcpy(&BMS_rpt_status_1, CAN_RX_msg.data, sizeof BMS_rpt_status_1);
      //update_flags.flags.bms_status_1 = true;

      #ifdef DEBUG
        DebugSerial.print ("Status charger_status: ");
        DebugSerial.println (BMS_rpt_status_1.charger_status);
        DebugSerial.print ("Status load_status: ");
        DebugSerial.println (BMS_rpt_status_1.load_status);
      #endif
    }*/
    else if (CAN_RX_msg.id == BMS_ParseId (BMS_charging_id_1))
    {
      // get data from the CAN packet into the report structure
      memcpy(&BMS_rpt_charging_1, CAN_RX_msg.data, sizeof BMS_rpt_charging_1);
      update_flags.flags.bms_charging_1 = true;

      #ifdef DEBUG
        //DebugSerial.print ("Charging charge_status: ");
        //DebugSerial.println (BMS_rpt_charging_1.charge_status);
      #endif
    }
    else if (CAN_RX_msg.id == BMS_ParseId (BMS_fault_id_1))
    {
      // get data from the CAN packet into the report structure
      memcpy(&BMS_rpt_fault_1, CAN_RX_msg.data, sizeof BMS_rpt_fault_1);
      update_flags.flags.bms_fault_1 = true;

      #ifdef DEBUG
        DebugSerial.print ("Fault 1: ");
        DebugSerial.println (BMS_rpt_fault_1.byte_7);
      #endif
    }
    
    /* If I can get the CAN ID changed on the 2nd BMS .. 
    else if (CAN_RX_msg.id == BMS_ParseId (BMS_soc_id_2))
    {
      // get data from the CAN packet into the report structure
      memcpy(&BMS_rpt_soc_2, CAN_RX_msg.data, sizeof BMS_rpt_soc_2);
      update_flags.flags.bms_soc_2 = true;
    }
    else if (CAN_RX_msg.id == BMS_ParseId (BMS_temp_id_2))
    {
      // get data from the CAN packet into the report structure
      memcpy(&BMS_rpt_temp_2, CAN_RX_msg.data, sizeof BMS_rpt_temp_2);
      update_flags.flags.bms_temp_2 = true;
    }
    else if (CAN_RX_msg.id == BMS_ParseId (BMS_fault_id_2))
    {
      // get data from the CAN packet into the report structure
      memcpy(&BMS_rpt_fault_2, CAN_RX_msg.data, sizeof BMS_rpt_fault_2);
      update_flags.flags.bms_fault_2 = true;
    } 
    */
  }

  /* state machine coding principles
  - process transitions first
  - reset each update flag as it is 'used'
  - read data as needed in the state
  */

  // State dependent updates to display etc
  switch (state)
  {
    // no valid data yet
    case STATE_STARTING:

      // got valid data from BMS:
      if (update_flags.flags.bms_soc_1)
      {
        // clear the flag
        update_flags.flags.bms_soc_1 = false;

        // start the gauge & update it
        GAUGE_Init();
        GAUGE_Set (BMS_get_soc (&BMS_rpt_soc_1));

        state = STATE_IDLE;

        #ifdef DEBUG
          DebugSerial.println("To state: IDLE");
        #endif
      }

    break;

    // BMS is sending data but Votol controller is off
    case STATE_IDLE:

      // got valid data from Votol: progress to WAKEUP
      if (update_flags.flags.votol)
      {
        // clear the flag
        update_flags.flags.votol = false;

        // wait for controller to wake up
        state = STATE_WAKEUP;

        // next read
        VotolSerial.write (VOTOL_Request_Local, sizeof (VOTOL_Request_Local));

        // immediately read / clear the buffer
        // as the 'CAN' converter echos the TX packet back to RX
        VotolSerial.readBytes (VOTOL_Buffer, sizeof (VOTOL_Request_Local));

        #ifdef DEBUG
          DebugSerial.println("To state: WAKEUP");
        #endif
      }
      else if (tick_flag)
      {
        tick_flag = false;

        // Spam the VOTOL controller to stop it sending its own requests on startup (causing a delay)
        // do this on every timer tick

        #ifdef DEBUG
          DebugSerial.println ("Spamming Votol");
        #endif
        
        // TODO: Consider testing availableForWrite()? Returns available space in buffer

        // Request data from motor controller via UART
        VotolSerial.write (VOTOL_Request_Local, sizeof (VOTOL_Request_Local));

        // immediately read / clear the buffer
        // as the 'CAN' converter echos the TX packet back to RX
        // When controller first starts, it interrupts the outgoing packet, losing 1 byte
        // This causes a hang if waiting for a complete request to be read back
        // So, flush the buffer instead to max of the bytes sent
        VOTOL_flush_rx (sizeof (VOTOL_Buffer));

        #ifdef DEBUG
          //DebugSerial.println ("Readback done");
        #endif
      }

      // get data from BMS only and update the dash
      if ((update_flags.all_flags & BMS_UPDATED) == BMS_UPDATED)
      {
        update_flags.all_flags &= ~BMS_UPDATED; // reset update flags for next time

        // If charging detected whilst Votol disabled (no need to check for regen charging)rge_timeout_cnt?
        if (BMS_get_charge_status (&BMS_rpt_charging_1))
        {
          if (++charge_timeout_cnt >= T_CHARGE_TIMEOUT) // count a few times to make sure
          {
            charge_timeout_cnt = 0; // reset for next time

            state = STATE_CHARGING;

            #ifdef DEBUG
              DebugSerial.println("To state: CHARGING");
            #endif
          }
        }
        else
        {
          // not charging, reset the counter
          charge_timeout_cnt = 0;
        }

        #ifdef DEBUG
          DebugSerial.println ("Updating dash");
        #endif

        // Display 1 content according to mode
        switch (display_mode)
        {
          case MODE_AMPS:
            draw_display (display1, DISPLAY_MODE_AMPS, NULL, NULL, NULL, NULL, NULL);
            break;

          case MODE_TRIP:
            draw_display (display1, DISPLAY_MODE_TRIP, NULL, NULL, NULL, NULL, NULL);
            break;

          case MODE_STATS:
            draw_display (display1, DISPLAY_MODE_STATS, NULL, NULL, NULL, NULL, NULL);
            break;
        }

        draw_display (display2, DISPLAY_MODE_TEMPS, NULL, &BMS_rpt_soc_1, &BMS_rpt_temp_1, &BMS_rpt_fault_1, NULL);
        GAUGE_Set (BMS_get_soc (&BMS_rpt_soc_1));
      }

    break;

    // waiting for controller to wake up
    // as it tends to send junk data for a little while
    // e.g. temperature 96C
    case STATE_WAKEUP:

      if (update_flags.flags.votol)
      {
        // clear the flag
        update_flags.flags.votol = false;

        if (VOTOL_check_valid_temp (&VOTOL_Response.resp))
        {
          state = STATE_ACTIVE;

          // start read time capture
          last_read_t = millis();

          // reset the trip stats
          TRIP_STATS_reset();

          #ifdef DEBUG
            DebugSerial.println ("To state: ACTIVE");
          #endif
        }
        else
        {
          #ifdef DEBUG
            DebugSerial.println ("Temp not yet valid");
          #endif
        }
      }

      // initiate a read for next time
      if (read_flag)
      {
        // note: don't clear the read_flag here as it is used & cleared below for BMS data

        // Request data from motor controller via UART
        VotolSerial.write (VOTOL_Request_Local, sizeof (VOTOL_Request_Local));

        // immediately read / clear the buffer
        // as the 'CAN' converter echos the TX packet back to RX
        VotolSerial.readBytes (VOTOL_Buffer, sizeof (VOTOL_Request_Local));

        #ifdef DEBUG
          DebugSerial.println ("Votol request sent");
        #endif
      }

    break;

    // BMS and Votol actively responding
    case STATE_ACTIVE:

      // detect Votol timeout ane return to IDLE state
      // (e.g. controller is switched off)
      if (read_flag && !update_flags.flags.votol)
      {

        if (++votol_timeout_cnt >= T_VOTOL_TIMEOUT)
        {
          // clear the VOTOL data
          // memcpy (&VOTOL_Response.data, &VOTOL_Buffer_init, sizeof (VOTOL_Buffer_init)); // not needed with IDLE state
          votol_timeout_cnt = 0; // start counting again, when new valid data arrives

          // back to IDLE state
          state = STATE_IDLE;

          #ifdef DEBUG
            DebugSerial.println("To state: IDLE");
          #endif
        }
        else
        {
          #ifdef DEBUG
            DebugSerial.print ("Votol timeout count: ");
            DebugSerial.println (votol_timeout_cnt);
          #endif        
        }
      }

      // valid data this time, clear the timeout_count
      if (update_flags.flags.votol)
      {
        votol_timeout_cnt = 0;

        // update the stats
        read_t = millis() - last_read_t;
        last_read_t = millis();
        uint16_t rpm = VOTOL_get_rpm (&VOTOL_Response.resp);

        trip_stats.distance_mm += WHEEL_CIRC * rpm * read_t / 60000;
        trip_stats.watt_s_x100 += VOTOL_get_volts (&VOTOL_Response.resp) * VOTOL_get_amps (&VOTOL_Response.resp) * read_t / 1000;
        trip_stats.avg_speed_x10 += WHEEL_CIRC * rpm * 36 / 60000; // add current speed
        trip_stats.avg_speed_x10 >>= 1; // div by 2

        #ifdef DEBUG
            DebugSerial.printf ("Distance: %d.%d\r\n", trip_stats.distance_mm / 1000000, trip_stats.distance_mm % 10);
            DebugSerial.printf ("Watt seconds: %d\r\n", trip_stats.watt_s_x100 / 100);
            DebugSerial.printf ("Current speed x10: %d\r\n", WHEEL_CIRC * rpm * 36 / 60000);
            //DebugSerial.printf ("Avg speed: %d.%d\r\n", trip_stats.avg_speed_x10 / 10, trip_stats.avg_speed_x10 % 10);
            //DebugSerial.printf ("Trip time: %d:%02d:%02d\r\n", trip_stats.trip_time.getHours(), trip_stats.trip_time.getMinutes(), trip_stats.trip_time.getSeconds());
        #endif
      }

      // initiate a read for next time
      if (read_flag)
      {
        // note: don't clear the read_flag here as it is used & cleared below for BMS data

        // Request data from motor controller via UART
        VotolSerial.write (VOTOL_Request_Local, sizeof (VOTOL_Request_Local));

        // immediately read / clear the buffer
        // as the 'CAN' converter echos the TX packet back to RX
        VotolSerial.readBytes (VOTOL_Buffer, sizeof (VOTOL_Request_Local));

        #ifdef DEBUG
          DebugSerial.println ("Votol request sent");
        #endif
      }

      // All data updated
      // Dash needs to be updated
      // Also check for charging
      if ((update_flags.all_flags & ALL_UPDATED) == ALL_UPDATED)
      {
        update_flags.all_flags = 0; // reset update flags for next time

        // If charging detected whilst controller on but no regen (i.e must be external charger)
        if (!VOTOL_get_regen_status (&VOTOL_Response.resp) && BMS_get_charge_status (&BMS_rpt_charging_1))
        {
          if (++charge_timeout_cnt >= T_CHARGE_TIMEOUT) // count a few times to make sure
          {
            charge_timeout_cnt = 0; // reset for next time

            state = STATE_CHARGING;

            #ifdef DEBUG
              DebugSerial.println ("To state: CHARGING");
            #endif
          }
          else
          {
            #ifdef DEBUG
              DebugSerial.printf ("Charge count: %d\r\n", charge_timeout_cnt);
            #endif
          }
        }
        else
        {
          // not charging, reset the counter
          charge_timeout_cnt = 0;
        }

        #ifdef DEBUG
          DebugSerial.println ("Updating dash");
        #endif

        // Display 1 content according to mode
        switch (display_mode)
        {
          case MODE_AMPS:
            draw_display (display1, DISPLAY_MODE_AMPS, &VOTOL_Response.resp, NULL, NULL, NULL, NULL);
            break;

          case MODE_TRIP:
            draw_display (display1, DISPLAY_MODE_TRIP, NULL, NULL, NULL, NULL, &trip_stats);
            break;

          case MODE_STATS:
            draw_display (display1, DISPLAY_MODE_STATS, NULL, NULL, NULL, NULL, &trip_stats);
            break;
        }

        // display 2 content
        draw_display (display2, DISPLAY_MODE_TEMPS, &VOTOL_Response.resp, &BMS_rpt_soc_1, &BMS_rpt_temp_1, &BMS_rpt_fault_1, NULL);
        GAUGE_Set (BMS_get_soc (&BMS_rpt_soc_1));
      }

    break;

    // Charging from external charger
    case STATE_CHARGING:

      if (read_flag & !BMS_get_charge_status (&BMS_rpt_charging_1))
      {
        // Exit CHARGING state back to IDLE

        // note: no need to clear the flag as it is done below when reading the BMS
        state = STATE_IDLE;

        #ifdef DEBUG
          DebugSerial.println ("To state: IDLE");
        #endif

        // Request data from motor controller via UART (to reinitiate data reading)
        VotolSerial.write (VOTOL_Request_Local, sizeof (VOTOL_Request_Local));

        // immediately read / clear the buffer
        // as the 'CAN' converter echos the TX packet back to RX
        VotolSerial.readBytes (VOTOL_Buffer, sizeof (VOTOL_Request_Local));

        #ifdef DEBUG
          DebugSerial.println ("Votol request sent");
        #endif
      }
      else if (tick_flag)
      {
        // Disable the controller (can't drive)
        // do this every tick, to get a fast response
        tick_flag = false;

        // Send the disarm packet (park mode)
        // note, disabled this for now; risk of spurious entry to Park mode
        //VotolSerial.write (VOTOL_Request_Disarm, sizeof (VOTOL_Request_Disarm));

        // immediately read / clear the buffer
        // as the 'CAN' converter echos the TX packet back to RX
        //VotolSerial.readBytes (VOTOL_Buffer, sizeof (VOTOL_Request_Local));

        #ifdef DEBUG
          DebugSerial.println ("Votol disarm sent");
        #endif
      }

      // Update display for charging
      if (update_flags.flags.bms_soc_1)
      {
        update_flags.flags.bms_soc_1 = 0; // reset update flags for next time

        draw_display (display1, DISPLAY_MODE_CHARGE, NULL, &BMS_rpt_soc_1, &BMS_rpt_temp_1, &BMS_rpt_fault_1, NULL);
        draw_display (display2, DISPLAY_MODE_TEMPS, NULL, &BMS_rpt_soc_1, &BMS_rpt_temp_1, &BMS_rpt_fault_1, NULL);
        GAUGE_Set (BMS_get_soc (&BMS_rpt_soc_1));
      }

    break;


    // External controller connected i.e. Votol software
    case STATE_EXT_CONTROL:
      // do nothing?
      // exit state by power cycling only
    break;

  }

  // ready to read BMS data in every state
  if (read_flag)
  {
    read_flag = false; // reset

    // initiate reads in sequence

    // Request data from BMS 1 via CAN
    BMS_request_data (BMS_CAN_ID_1, BMS_DATA_ID_SOC);
    BMS_request_data (BMS_CAN_ID_1, BMS_DATA_ID_TEMP);
    BMS_request_data (BMS_CAN_ID_1, BMS_DATA_ID_FAULT);
    //BMS_request_data (BMS_CAN_ID_1, BMS_DATA_ID_STATUS); // Don't need this
    BMS_request_data (BMS_CAN_ID_1, BMS_DATA_ID_CHARGING);

    #ifdef DEBUG
      DebugSerial.println ("BMS 1 request sent");
    #endif

    /* Request data from BMS 2 via CAN (If I can get the CAN ID chnaged)
    BMS_request_data (BMS_CAN_ID_2, BMS_DATA_ID_SOC);
    BMS_request_data (BMS_CAN_ID_2, BMS_DATA_ID_TEMP);
    BMS_request_data (BMS_CAN_ID_2, BMS_DATA_ID_FAULT);

    #ifdef DEBUG
      DebugSerial.println ("BMS 2 request sent");
    #endif
    */
  }
}

// initialise the dash
void init_dash ()
{
  // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  if(!(display1.begin (SSD1306_SWITCHCAPVCC, 0, true, true) && display2.begin (SSD1306_SWITCHCAPVCC, 0, true, true)))
  {
    #ifdef DEBUG
      DebugSerial.println (F("SSD1306 allocation failed"));
    #endif
  }
  else
  {
    display_init = true;
  
    // update display to show startup mode
    draw_display (display1, DISPLAY_MODE_START, NULL, NULL, NULL, NULL, NULL);
    draw_display (display2, DISPLAY_MODE_START, NULL, NULL, NULL, NULL, NULL);
 }
}

// update the lights
void update_lights ()
{
  bool stop_state;
  bool tail_state;
  LIGHTS_Data_t LIGHTS_Data; // CAN data packet

  // read the lights data from the ports
  stop_state = digitalRead (BRAKE_IN);
  tail_state = digitalRead (LIGHTS_IN);
  blink_mode = digitalRead (TURN_L_IN) ? BLINK_MODE_LEFT : (digitalRead (TURN_R_IN) ? BLINK_MODE_RIGHT : BLINK_MODE_NONE);

  // Reset blink state & counter on change of blink mode
  if (blink_mode != last_blink_mode)
  {
    blink_state = true;
    blink_count = 0;
    last_blink_mode = blink_mode;
  }

  // update the "local" lights
  digitalWrite (TURN_L_OUT, ((blink_mode == BLINK_MODE_LEFT) && blink_state));
  digitalWrite (TURN_R_OUT, ((blink_mode == BLINK_MODE_RIGHT) && blink_state));
  digitalWrite (TURN_IND_OUT, ((blink_mode != BLINK_MODE_NONE) && blink_state));

  // update lights over CAN
  LIGHTS_Data.bits.brake = stop_state;
  LIGHTS_Data.bits.tail = tail_state;
  LIGHTS_Data.bits.turn_l = (blink_mode == BLINK_MODE_LEFT) && blink_state;  
  LIGHTS_Data.bits.turn_r = (blink_mode == BLINK_MODE_RIGHT) && blink_state;  

  #ifdef DEBUG
  //  DebugSerial.print ("Lights data: ");
  //  DebugSerial.println (LIGHTS_Data.data);
  #endif

  LIGHTS_send_data (&LIGHTS_Data);
}

// call on any change in inputs (lights, brake, turn, mode etc)
void input_ISR ()
{
    debounce_flag = true;
}

// TODO: Just set the flag here (as this is an interrupt) then process everything in main()
// TDO: Make this a tick timer; # ticks per event (read Votol, read BMS, blink, .. )
void tick ()
{
  tick_flag = true;

  // initiate a read periodically
  if (++read_cnt >= T_READ)
  {
    read_cnt = 0;
    read_flag = true;

    // update the lights every time as well
    // in particular, tail light interrupt doesn't work
    // so just update it regularly anyhow
    update_lights_flag = true;

  }

  // toggle blink state when indicators are on
  if (blink_mode != BLINK_MODE_NONE)
  {
    if (++blink_count >= T_BLINK)
    {
      blink_state = !blink_state;
      blink_count = 0;
    }
  } 
}
