
/*
   Opel MS-CAN CAN Adapter for Ignition, reverse, illumination and steering wheel buttons.
   (c) 2020 Patrick Schmidt patrick6983@gmail.com
   v1.00 - Pioneer Remote tested on SPH-DA230DAB
  
   Used HW:
   Arduino NANO
   MPC2515 8MHz CAN Board
   MCP4151
   
   Based on origin code:
   2005 VOLVO XC70 steering wheel buttons interface for Kenwood head unit
   (c) 2014 Vitaly Mayatskikh vitaly@gravicappa.info


   Steering wheel message CAN ID is 0x206 DLC:3
   
   0x206 | DLC:3 | 00 81 00
                   00 = not pressed
                   01 = pressed
                   08 = wheel

                      81 = Voice
                      82 = Phone
                      83 = Wheel and knob
                      
                      91 = Next
                      92 = Previous
                      93 = Wheel and knob (only some have it)

                         00 = hold time
                         01 = For wheel Up / for normal incremental time while holding
                         FF = for wheel down

                      
              

   Ignition and Illumination CAN ID is 0x450 DLC:4

   0x450 | DLC:4 | 46 07 06 FF
                         06 = Ignition on
                         05 = Key In
                         00 = No key
                            FF = full illumination
                            00 = no illumination

   Reverse and Speed signal CAN ID is 0x4E8 DLC:7

   0x4E8 | DLC:7 | 46 0F 00 00 00 00 00  
                                     00 = no reverse
                                     04 = reverse
                                     00000100
                                          | Reverse Bit 1 = Reverse
                               00 00    = D4 = MSB Speed, D5 = LSB Speed 
         
*/

#include <SPI.h>
#include <mcp_can.h>
#include <mcp_can_dfs.h>


#define SWM_CAN_ID 0x206
#define BCM_CAN_ID 0x450
#define REV_CAN_ID 0x4E8

#define CAN_MASK (SWM_CAN_ID | BCM_CAN_ID | REV_CAN_ID)

//#define WD_OUTPUT     8  // MAX823/MAX824 pin 4 (WDI)
#define ILUM_OUTPUT   7  // Ignition control wire (through 5->12v NPN transistor amplifier)
#define CAN_RESET     3  // MCP2515 pin 17 (-Reset)
#define IGN_OUTPUT    5  // Ignition control wire (through 5->12v NPN transistor amplifier)
#define PARK_OUTPUT   4  // Parking position
#define CAMERA_OUTPUT 6  // Reverse position (through 5->12v NPN transistor amplifier)
#define CSPINPOT      9  // CS Pin for Digipot MCP4131
#define RING_GND      A0 // GND for Ring

unsigned long lastignframe = 0;
static bool igndelay = false;

MCP_CAN CAN(10); // Set CS to pin 10

boolean can_ok = false;

// In the function for the tip commands we only change resistance for the 
// wiper where the tip is connected. The ring floats and is ignored by the 
// head unit
void MCP41xx_tip(int digiValue, int delayMs) {
      digitalWrite(CSPINPOT, LOW);
      SPI.transfer(00);
      SPI.transfer(digiValue);
      delay(delayMs);
      SPI.transfer(00);
      SPI.transfer(00);
      digitalWrite(CSPINPOT, HIGH);
}

// For ring functions the wiper for the tip is where resistance is applied
// in combination with taking the ring to ground via A0
void MCP41xx_ring(int digiValue, int delayMs) {
      digitalWrite(CSPINPOT, LOW);
      pinMode(RING_GND, OUTPUT);
      digitalWrite(RING_GND, LOW);
      SPI.transfer(00);
      SPI.transfer(digiValue);
      delay(delayMs);
      SPI.transfer(00);
      SPI.transfer(00);
      pinMode(RING_GND, INPUT);
      //digitalWrite(RING_GND, HIGH);
      digitalWrite(CSPINPOT, HIGH);
}

void can_reset()
{
  pinMode(CAN_RESET, OUTPUT);
  digitalWrite(CAN_RESET, LOW);
  delay(100);
  digitalWrite(CAN_RESET, HIGH);
  delay(100);
}

void setup_can()
{
  Serial.println("CAN Module init...");
  can_reset();

  for (int i = 0; i < 4; i++) {
    if (CAN.begin(CAN_95KBPS,MCP_8MHz) == CAN_OK) {
      can_ok = true;
      break;
    }
    Serial.print(".");
    delay(250);
  }

  if (can_ok) {
    int err = 0;
    Serial.println("... init ok!");
    Serial.println("Set MCP filter settings");
    if (CAN.init_Mask(0, 0, CAN_MASK) != MCP2515_OK) {
      Serial.println("init mask 0 failed!");
      err++;
    }
    if (CAN.init_Mask(1, 0, CAN_MASK) != MCP2515_OK) {
      Serial.println("init mask 1 failed!");
      err++;
    }
    if (CAN.init_Filt(0, 0, SWM_CAN_ID) != MCP2515_OK) {
      Serial.println("init filter 0 failed!");
      err++;
    }
    if (CAN.init_Filt(1, 0, BCM_CAN_ID) != MCP2515_OK) {
      Serial.println("init filter 1 failed!");
      err++;
    }
    if (CAN.init_Filt(2, 0, REV_CAN_ID) != MCP2515_OK) {
      Serial.println("init filter 2 failed!");
      err++;
    }
    if (err)
      can_ok = false;
  }

  if (!can_ok) {
    Serial.println("... failed!");
    delay(3000); // hw wd kills us
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println("start");
  pinMode(ILUM_OUTPUT, OUTPUT);
  digitalWrite(ILUM_OUTPUT, HIGH);
  pinMode(IGN_OUTPUT, OUTPUT);
  digitalWrite(IGN_OUTPUT, HIGH);
  pinMode(PARK_OUTPUT, INPUT); //Set parking pin to input to make ith HI-Z State
  pinMode(CAMERA_OUTPUT, OUTPUT);
  digitalWrite(CAMERA_OUTPUT, HIGH);
  pinMode(CSPINPOT, OUTPUT);
  digitalWrite(CSPINPOT, LOW);
  SPI.begin();
  SPI.transfer(0);
  SPI.transfer(0);
  digitalWrite(CSPINPOT, HIGH);
  Serial.println("Pot init ok!");
  Serial.println("Opel MS-CAN init ok!");

  setup_can();
  Serial.println("All set!");
}

void loop()
{
  if (can_ok) {
    check_canbus();

    }
}

unsigned long check_canbus()
{
  unsigned char len;
  unsigned char buf[8];
  unsigned long can_id, can_mask = 0;

  while (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&len, buf);
    can_id = CAN.getCanId();
    can_mask |= can_id;
    switch (can_id) {
      case SWM_CAN_ID:
        //next track
        if (buf[0] == 0x1 && buf[1] == 0x91 && buf[2] == 0x00) {
        Serial.println("Next Track detect!");
        MCP41xx_tip(19, 250); // 8kOhm
        }
        //previous track
        else if (buf[0] == 0x1 && buf[1] == 0x92 && buf[2] == 0x00) {
        Serial.println("Previous Track detect!");
        MCP41xx_tip(27, 250); // 11,25kOhm
        }
        //volume up
        else if (buf[0] == 0x8 && buf[1] == 0x93 && buf[2] == 0x01) {
        Serial.println("Volume UP detect!");
        MCP41xx_tip(42, 250); // 16kOhm
        }
        //volume down
        else if (buf[0] == 0x8 && buf[1] == 0x93 && buf[2] == 0xff) {
        Serial.println("Volume Down detect!");
        MCP41xx_tip(55, 250); // 24kOhm
        }
        //voice button
        else if (buf[0] == 0x1 && buf[1] == 0x81 && buf[2] == 0x00) {
        Serial.println("Voice detect!");
        MCP41xx_ring(232, 1000); // 38-88kOhm hold = Voice active
        }
        //phone button
        else if (buf[0] == 0x1 && buf[1] == 0x82 && buf[2] == 0x00) {
        Serial.println("Phone detect!");
        MCP41xx_ring(9, 250); // 3kOhm Phone answer/ long press BT Menu
        }
        else if (buf[0] == 0x1 && buf[1] == 0x82 && buf[2] > 0x00) {
        Serial.println("Phone long detect!");
        MCP41xx_ring(14, 250); // 5kOhm Phone hang up / long press reject call
        }
        break;
      case BCM_CAN_ID:
        pinMode(IGN_OUTPUT, OUTPUT);
        if ((buf[2] & 0x07) == 0x04 ||(buf[2] & 0x07) == 0x05 || (buf[2] & 0x07) == 0x06) {
        digitalWrite(IGN_OUTPUT, LOW);
        lastignframe = millis();
        igndelay = true;
        Serial.println("Ignition detect!");
        }
        else if (igndelay == true && millis() - lastignframe > 5000) {
        digitalWrite(IGN_OUTPUT, HIGH);
        igndelay = false;
        Serial.println("Ignition off after 5sec!");
        } // switching off ignition after 5 sec 
        digitalWrite(ILUM_OUTPUT, ((buf[3] > 0x00)) ? LOW : HIGH); //00 = Light off, 01-FF Light intensity
        break;
      case REV_CAN_ID:
      //  pinMode(PARK_OUTPUT, OUTPUT);
      //  digitalWrite(PARK_OUTPUT, ((buf[6] & 0x30) == 0x10) ? LOW : HIGH);
      //  Serial.println("Parking detect!");
        pinMode(CAMERA_OUTPUT, OUTPUT);
        digitalWrite(CAMERA_OUTPUT, ((buf[6] & 0x04) == 0x04) ? LOW : HIGH);
        Serial.println("Reverse detect!");
        break;
      default:
        break;
    }
  }
  return can_mask;
}
