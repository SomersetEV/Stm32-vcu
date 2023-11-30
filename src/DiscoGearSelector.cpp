#include <DiscoGearSelector.h>
#include "stm32_can.h"
#include "params.h"

uint8_t counter0 = 0x82;
uint8_t counter1 = 0x00;


void DiscoGearSelector::SetCanInterface(CanHardware* c)
{
   can = c;

   can->RegisterUserMessage(0x312);//Disco gear selector can message

}
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////Handle incomming pt can messages from the car here
////////////////////////////////////////////////////////////////////////////////////////////////////
void DiscoGearSelector::DecodeCAN(int id,  uint32_t* data)
{
    uint8_t* bytes = (uint8_t*)data;
    if (id == 0x312)
    {
        int check = bytes[2];
        switch (check)
        {
        case 0x8F:// Park
          gear = PARK;
          b0 = 0x5C;
          b1 = 0x66;
          b4 = 0xFF;
          b5 = 0x7F;
          b6 = 0x00;
          b7 = 0x80;
          Scycle = 0x82;
          Fcycle = 0x90;
          counter1 = 130;
          break;

        case 0x8E: // R
          gear = REVERSE;
          b0 = 0x7C;
          b1 = 0x24;
          b4 = 0xFE;
          b5 = 0xFF;
          b6 = 0x01;
          b7 = 0x00;
          Scycle = 0x03;
          Fcycle = 0x11;
          counter1 = 3;
          break;

        case 0x8D: // N
          gear = NEUTRAL;
          b0 = 0x7C;
          b1 = 0x25;
          b4 = 0xFD;
          b5 = 0xFF;
          b6 = 0x02;
          b7 = 0x00;
          Scycle = 0x04;
          Fcycle = 0x12;
          counter1 = 4;
          break;

          case 0x8C: // D
          gear = DRIVE;
          b0 = 0x7C;
          b1 = 0x24;
          b4 = 0xFB;
          b5 = 0xFF;
          b6 = 0x04;
          b7 = 0x00;
          Scycle = 0x06;
          Fcycle = 0x14;
          counter1 = 6;
          break;

          case 0x88: // S
          gear = DRIVE;
          b0 = 0x7C;
          b1 = 0x24;
          b4 = 0xF7;
          b5 = 0xFF;
          b6 = 0x08;
          b7 = 0x00;
          Scycle = 0x0A;
          Fcycle = 0x18;
          counter1 = 10;
           break;
      }
   }
}


void DiscoGearSelector::Task20Ms()
{
  uint8_t bytes[8];
   // Set the data in the message
    bytes[0] = b0;
    bytes[1] = b1;
    bytes[2] = counter0 - counter1;
    bytes[3] = counter0;
    counter0++;
    if (counter0 > Fcycle)
    {
      counter0 = Scycle;
    }
    bytes[4] = b4;
    bytes[5] = b5;
    bytes[6] = b6;
    bytes[7] = b7;


   can->Send(0x3F3, bytes, 8);
}

bool DiscoGearSelector::GetGear(Vehicle::gear& outGear)
{
   outGear = gear;    //send the shifter pos
   return true; //Let caller know we set a valid gear
}

