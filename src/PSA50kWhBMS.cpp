/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2024 Ben Bament, based on SIMPBMS module by Charlie Smurthwaite
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "stm32_vcu.h"
/*
 * This module receives messages from PSA50kWhBMS and updates the
 * BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the
 * received values. It also implements a timeout to indicate whether
 * the BMS is actively sending data or not. This data can be
 * used to safely stop any charging process if the BMS is not
 * working correctly.
 */

void PSA50kWhBMS::SetCanInterface(CanHardware* c)
{
   can = c;
  can->RegisterUserMessage(0x6E0, 0x700);
  //can->RegisterUserMessage(0x6E0, 0x6E7);
  can->RegisterUserMessage(0x6D1, 0x6D5);  
//can->RegisterUserMessage(0x6E2);

//can->RegisterUserMessage(0x6D4);

/*
can->RegisterUserMessage(0x6E8);
can->RegisterUserMessage(0x6E9);
can->RegisterUserMessage(0x6EB);
can->RegisterUserMessage(0x6EC);
can->RegisterUserMessage(0x6ED);
can->RegisterUserMessage(0x6EE);
can->RegisterUserMessage(0x6EF);
can->RegisterUserMessage(0x6F0);
can->RegisterUserMessage(0x6F1);
can->RegisterUserMessage(0x6F2);
can->RegisterUserMessage(0x6F3);
can->RegisterUserMessage(0x6F4);
can->RegisterUserMessage(0x6F5);
can->RegisterUserMessage(0x6F6);
can->RegisterUserMessage(0x6F7);
can->RegisterUserMessage(0x6F8);
can->RegisterUserMessage(0x6F9);
can->RegisterUserMessage(0x6FA);
can->RegisterUserMessage(0x6FB);
can->RegisterUserMessage(0x6FC);
can->RegisterUserMessage(0x6FD);
can->RegisterUserMessage(0x6FE);
can->RegisterUserMessage(0x6FF);
*/

}

bool PSA50kWhBMS::BMSDataValid() {
   // Return false if primary BMS is not sending data.
   if(timeoutCounter < 1) return false;
   return true;
}

// Return whether charging is currently permitted.
bool PSA50kWhBMS::ChargeAllowed()
{
   // Refuse to charge if the BMS is not sending data.
   if(!BMSDataValid()) return false;

   // Refuse to charge if the voltage or temperature is out of range.
   if(maxCellV > Param::GetFloat(Param::BMS_VmaxLimit)) return false;
   if(minCellV < Param::GetFloat(Param::BMS_VminLimit)) return false;
   if(maxTempC > Param::GetFloat(Param::BMS_TmaxLimit)) return false;
   if(minTempC < Param::GetFloat(Param::BMS_TminLimit)) return false;

   // Refuse to charge if the current limit is zero.
   if(chargeCurrentLimit < 0.5) return false;

   // Otherwise, charging is permitted.
   return true;
}

// Return the maximum charge current allowed by the BMS.
float PSA50kWhBMS::MaxChargeCurrent()
{
   if(!ChargeAllowed()) return 0;
   return chargeCurrentLimit / 1000.0;
}

// Process voltage and temperature message from PSA50kWhBMS.
void PSA50kWhBMS::DecodeCAN(int id, uint8_t *data)
{
   switch (id)
   {
   case 0x6E7:
    Cell1v = (data[0] << 8) | data[1];
    Cell2v = (data[2] << 8) | data[3];
    Cell3v = (data[4] << 8) | data[5];
    Cell4v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6E8:

    Cell5v = (data[0] << 8) | data[1];
    Cell6v = (data[2] << 8) | data[3];
    Cell7v = (data[4] << 8) | data[5];
    Cell8v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6E9:

    Cell9v = (data[0] << 8) | data[1];
    Cell10v = (data[2] << 8) | data[3];
    Cell11v = (data[4] << 8) | data[5];
    Cell12v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6EB:

    Cell13v = (data[0] << 8) | data[1];
    Cell14v = (data[2] << 8) | data[3];
    Cell15v = (data[4] << 8) | data[5];
    Cell16v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6EC:

    Cell17v = (data[0] << 8) | data[1];
    Cell18v = (data[2] << 8) | data[3];
    Cell19v = (data[4] << 8) | data[5];
    Cell20v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6ED:

    Cell21v = (data[0] << 8) | data[1];
    Cell22v = (data[2] << 8) | data[3];
    Cell23v = (data[4] << 8) | data[5];
    Cell24v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6EE:

    Cell25v = (data[0] << 8) | data[1];
    Cell26v = (data[2] << 8) | data[3];
    Cell27v = (data[4] << 8) | data[5];
    Cell28v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6EF:

    Cell29v = (data[0] << 8) | data[1];
    Cell30v = (data[2] << 8) | data[3];
    Cell31v = (data[4] << 8) | data[5];
    Cell32v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F0:

    Cell33v = (data[0] << 8) | data[1];
    Cell34v = (data[2] << 8) | data[3];
    Cell35v = (data[4] << 8) | data[5];
    Cell36v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F1:

    Cell37v = (data[0] << 8) | data[1];
    Cell38v = (data[2] << 8) | data[3];
    Cell39v = (data[4] << 8) | data[5];
    Cell40v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F2:

    Cell41v = (data[0] << 8) | data[1];
    Cell42v = (data[2] << 8) | data[3];
     Cell43v = (data[4] << 8) | data[5];
     Cell44v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F3:

     Cell45v = (data[0] << 8) | data[1];
     Cell46v = (data[2] << 8) | data[3];
     Cell47v = (data[4] << 8) | data[5];
     Cell48v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F4:

     Cell49v = (data[0] << 8) | data[1];
     Cell50v = (data[2] << 8) | data[3];
     Cell51v = (data[4] << 8) | data[5];
     Cell52v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F5:

     Cell53v = (data[0] << 8) | data[1];
     Cell54v = (data[2] << 8) | data[3];
     Cell55v = (data[4] << 8) | data[5];
     Cell56v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F6:

     Cell57v = (data[0] << 8) | data[1];
     Cell58v = (data[2] << 8) | data[3];
     Cell59v = (data[4] << 8) | data[5];
     Cell60v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F7:

     Cell61v = (data[0] << 8) | data[1];
     Cell62v = (data[2] << 8) | data[3];
     Cell63v = (data[4] << 8) | data[5];
     Cell64v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F8:

    Cell65v = (data[0] << 8) | data[1];
    Cell66v = (data[2] << 8) | data[3];
    Cell67v = (data[4] << 8) | data[5];
    Cell68v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6F9:

    Cell69v = (data[0] << 8) | data[1];
     Cell70v = (data[2] << 8) | data[3];
    Cell71v = (data[4] << 8) | data[5];
    Cell72v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6FA:

    Cell73v = (data[0] << 8) | data[1];
    Cell74v = (data[2] << 8) | data[3];
    Cell75v = (data[4] << 8) | data[5];
    Cell76v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6FB:

    Cell77v = (data[0] << 8) | data[1];
    Cell78v = (data[2] << 8) | data[3];
    Cell79v = (data[4] << 8) | data[5];
    Cell80v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6FC:

    Cell81v = (data[0] << 8) | data[1];
    Cell82v = (data[2] << 8) | data[3];
    Cell83v = (data[4] << 8) | data[5];
    Cell84v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6FD:

    Cell85v = (data[0] << 8) | data[1];
    Cell86v = (data[2] << 8) | data[3];
    Cell87v = (data[4] << 8) | data[5];
    Cell88v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6FE:

    Cell89v = (data[0] << 8) | data[1];
    Cell90v = (data[2] << 8) | data[3];
    Cell91v = (data[4] << 8) | data[5];
    Cell92v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6FF:

    Cell93v = (data[0] << 8) | data[1];
    Cell94v = (data[2] << 8) | data[3];
    Cell95v = (data[4] << 8) | data[5];
    Cell96v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6E1:

    Cell93v = (data[0] << 8) | data[1];
    Cell94v = (data[2] << 8) | data[3];
    Cell95v = (data[4] << 8) | data[5];
    Cell96v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6E2:

    Cell97v = (data[0] << 8) | data[1];
    Cell98v = (data[2] << 8) | data[3];
    Cell99v = (data[4] << 8) | data[5];
    Cell100v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6D3:

    Cell101v = (data[0] << 8) | data[1];
    Cell102v = (data[2] << 8) | data[3];
    Cell103v = (data[4] << 8) | data[5];
    Cell104v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

case 0x6D4:

    Cell105v = (data[0] << 8) | data[1];
    Cell106v = (data[2] << 8) | data[3];
    Cell107v = (data[4] << 8) | data[5];
    Cell108v = (data[6] << 8) | data[7];
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

break;

   case 0x6D1:
   Cell1t = data[0];
   Cell2t = data[1];
   Cell3t = data[2];
   Cell4t = data[3];
   Cell5t = data[4];
   Cell6t = data[5];
   Cell7t = data[6];
   Cell8t = data[7];
   timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

   break;

   case 0x6D2:
   Cell9t = data[0];
Cell10t = data[1];
Cell11t = data[2];
Cell12t = data[3];
Cell13t = data[4];
Cell14t = data[5];
Cell15t = data[6];
Cell16t = data[7];
   timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

   break;

   case 0x6E0:
   Cell17t = data[0];
Cell18t = data[1];
Cell19t = data[2];
Cell20t = data[3];
Cell21t = data[4];
Cell22t = data[5];

   timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

   break;

   case 0x6E3:
   Cell23t = data[0];
Cell24t = data[1];
Cell25t = data[2];
Cell26t = data[3];
Cell27t = data[4];
Cell28t = data[5];
Cell29t = data[6];
Cell30t = data[7];
   timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

   break;
   
   case 0x6E4:
   Cell31t = data[0];
Cell32t = data[1];
Cell33t = data[2];
Cell34t = data[3];
Cell35t = data[4];
Cell36t = data[5];
Cell37t = data[6];
Cell38t = data[7];
   timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

   break;

   case 0x6E5:
   Cell39t = data[0];
Cell40t = data[1];
Cell41t = data[2];
Cell42t = data[3];
Cell43t = data[4];
Cell44t = data[5];
Cell45t = data[6];
Cell46t = data[7];
   timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

   break;

   case 0x6E6:
   Cell47t = data[0];
Cell48t = data[1];
Cell49t = data[2];
Cell50t = data[3];
Cell51t = data[4];
Cell52t = data[5];
Cell53t = data[6];
Cell54t = data[7];
   timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

   break;
}
}



void PSA50kWhBMS::Task100Ms() {
   // Decrement timeout counter.
   if(timeoutCounter > 0) timeoutCounter--;
 
  //Set max charge based on voltage and temp, from charge curve
  Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());
  PSA50kWhBMS::findlowestcellvoltage();
  PSA50kWhBMS::findhighestcellvoltage();
  minCellV = lowestVoltage / 1000;
  maxCellV = highestVoltage / 1000;
  
  PSA50kWhBMS::findlowestcelltemp();
  PSA50kWhBMS::findhighestcelltemp();
  minTempC = lowestTemp - 50;
  maxTempC = highestTemp - 50;

   if(BMSDataValid()) {
      Param::SetFloat(Param::BMS_Vmin, minCellV);
      Param::SetFloat(Param::BMS_Vmax, maxCellV);
      Param::SetFloat(Param::BMS_Tmin, minTempC);
      Param::SetFloat(Param::BMS_Tmax, maxTempC);
   }
   else
   {
      Param::SetFloat(Param::BMS_Vmin, 0);
      Param::SetFloat(Param::BMS_Vmax, 0);
      Param::SetFloat(Param::BMS_Tmin, 0);
      Param::SetFloat(Param::BMS_Tmax, 0);
   }
}

void PSA50kWhBMS::findlowestcellvoltage(){
// Function to find the lowest cell voltage
   lowestVoltage = 5000; //reset value 

    // Using a macro to simplify the comparison process
    #define CHECK_VOLTAGE(cell) if ((cell < lowestVoltage) && (cell > 1000)) lowestVoltage = cell

    CHECK_VOLTAGE(Cell1v);   CHECK_VOLTAGE(Cell2v);   CHECK_VOLTAGE(Cell3v);   CHECK_VOLTAGE(Cell4v);
    CHECK_VOLTAGE(Cell5v);   CHECK_VOLTAGE(Cell6v);   CHECK_VOLTAGE(Cell7v);   CHECK_VOLTAGE(Cell8v);
    CHECK_VOLTAGE(Cell9v);   CHECK_VOLTAGE(Cell10v);  CHECK_VOLTAGE(Cell11v);  CHECK_VOLTAGE(Cell12v);
    CHECK_VOLTAGE(Cell13v);  CHECK_VOLTAGE(Cell14v);  CHECK_VOLTAGE(Cell15v);  CHECK_VOLTAGE(Cell16v);
    CHECK_VOLTAGE(Cell17v);  CHECK_VOLTAGE(Cell18v);  CHECK_VOLTAGE(Cell19v);  CHECK_VOLTAGE(Cell20v);
    CHECK_VOLTAGE(Cell21v);  CHECK_VOLTAGE(Cell22v);  CHECK_VOLTAGE(Cell23v);  CHECK_VOLTAGE(Cell24v);
    CHECK_VOLTAGE(Cell25v);  CHECK_VOLTAGE(Cell26v);  CHECK_VOLTAGE(Cell27v);  CHECK_VOLTAGE(Cell28v);
    CHECK_VOLTAGE(Cell29v);  CHECK_VOLTAGE(Cell30v);  CHECK_VOLTAGE(Cell31v);  CHECK_VOLTAGE(Cell32v);
    CHECK_VOLTAGE(Cell33v);  CHECK_VOLTAGE(Cell34v);  CHECK_VOLTAGE(Cell35v);  CHECK_VOLTAGE(Cell36v);
    CHECK_VOLTAGE(Cell37v);  CHECK_VOLTAGE(Cell38v);  CHECK_VOLTAGE(Cell39v);  CHECK_VOLTAGE(Cell40v);
    CHECK_VOLTAGE(Cell41v);  CHECK_VOLTAGE(Cell42v);  CHECK_VOLTAGE(Cell43v);  CHECK_VOLTAGE(Cell44v);
    CHECK_VOLTAGE(Cell45v);  CHECK_VOLTAGE(Cell46v);  CHECK_VOLTAGE(Cell47v);  CHECK_VOLTAGE(Cell48v);
    CHECK_VOLTAGE(Cell49v);  CHECK_VOLTAGE(Cell50v);  CHECK_VOLTAGE(Cell51v);  CHECK_VOLTAGE(Cell52v);
    CHECK_VOLTAGE(Cell53v);  CHECK_VOLTAGE(Cell54v);  CHECK_VOLTAGE(Cell55v);  CHECK_VOLTAGE(Cell56v);
    CHECK_VOLTAGE(Cell57v);  CHECK_VOLTAGE(Cell58v);  CHECK_VOLTAGE(Cell59v);  CHECK_VOLTAGE(Cell60v);
    CHECK_VOLTAGE(Cell61v);  CHECK_VOLTAGE(Cell62v);  CHECK_VOLTAGE(Cell63v);  CHECK_VOLTAGE(Cell64v);
    CHECK_VOLTAGE(Cell65v);  CHECK_VOLTAGE(Cell66v);  CHECK_VOLTAGE(Cell67v);  CHECK_VOLTAGE(Cell68v);
    CHECK_VOLTAGE(Cell69v);  CHECK_VOLTAGE(Cell70v);  CHECK_VOLTAGE(Cell71v);  CHECK_VOLTAGE(Cell72v);
    CHECK_VOLTAGE(Cell73v);  CHECK_VOLTAGE(Cell74v);  CHECK_VOLTAGE(Cell75v);  CHECK_VOLTAGE(Cell76v);
    CHECK_VOLTAGE(Cell77v);  CHECK_VOLTAGE(Cell78v);  CHECK_VOLTAGE(Cell79v);  CHECK_VOLTAGE(Cell80v);
    CHECK_VOLTAGE(Cell81v);  CHECK_VOLTAGE(Cell82v);  CHECK_VOLTAGE(Cell83v);  CHECK_VOLTAGE(Cell84v);
    CHECK_VOLTAGE(Cell85v);  CHECK_VOLTAGE(Cell86v);  CHECK_VOLTAGE(Cell87v);  CHECK_VOLTAGE(Cell88v);
    CHECK_VOLTAGE(Cell89v);  CHECK_VOLTAGE(Cell90v);  CHECK_VOLTAGE(Cell91v);  CHECK_VOLTAGE(Cell92v);
    CHECK_VOLTAGE(Cell93v);  CHECK_VOLTAGE(Cell94v);  CHECK_VOLTAGE(Cell95v);  CHECK_VOLTAGE(Cell96v);
    CHECK_VOLTAGE(Cell97v);  CHECK_VOLTAGE(Cell98v);  CHECK_VOLTAGE(Cell99v);  CHECK_VOLTAGE(Cell100v);
    CHECK_VOLTAGE(Cell101v); CHECK_VOLTAGE(Cell102v); CHECK_VOLTAGE(Cell103v); CHECK_VOLTAGE(Cell104v);
    CHECK_VOLTAGE(Cell105v); CHECK_VOLTAGE(Cell106v); CHECK_VOLTAGE(Cell107v); CHECK_VOLTAGE(Cell108v);

    #undef CHECK_VOLTAGE

    //return lowestVoltage;
}

void PSA50kWhBMS::findhighestcellvoltage(){
    // Function to find the highest cell voltage
    highestVoltage = 1000; // reset value
    // Using a macro to simplify the comparison process
    #define CHECK_VOLTAGE(cell) if ((cell > highestVoltage) && (cell > 1000)) highestVoltage = cell

    CHECK_VOLTAGE(Cell1v);   CHECK_VOLTAGE(Cell2v);   CHECK_VOLTAGE(Cell3v);   CHECK_VOLTAGE(Cell4v);
    CHECK_VOLTAGE(Cell5v);   CHECK_VOLTAGE(Cell6v);   CHECK_VOLTAGE(Cell7v);   CHECK_VOLTAGE(Cell8v);
    CHECK_VOLTAGE(Cell9v);   CHECK_VOLTAGE(Cell10v);  CHECK_VOLTAGE(Cell11v);  CHECK_VOLTAGE(Cell12v);
    CHECK_VOLTAGE(Cell13v);  CHECK_VOLTAGE(Cell14v);  CHECK_VOLTAGE(Cell15v);  CHECK_VOLTAGE(Cell16v);
    CHECK_VOLTAGE(Cell17v);  CHECK_VOLTAGE(Cell18v);  CHECK_VOLTAGE(Cell19v);  CHECK_VOLTAGE(Cell20v);
    CHECK_VOLTAGE(Cell21v);  CHECK_VOLTAGE(Cell22v);  CHECK_VOLTAGE(Cell23v);  CHECK_VOLTAGE(Cell24v);
    CHECK_VOLTAGE(Cell25v);  CHECK_VOLTAGE(Cell26v);  CHECK_VOLTAGE(Cell27v);  CHECK_VOLTAGE(Cell28v);
    CHECK_VOLTAGE(Cell29v);  CHECK_VOLTAGE(Cell30v);  CHECK_VOLTAGE(Cell31v);  CHECK_VOLTAGE(Cell32v);
    CHECK_VOLTAGE(Cell33v);  CHECK_VOLTAGE(Cell34v);  CHECK_VOLTAGE(Cell35v);  CHECK_VOLTAGE(Cell36v);
    CHECK_VOLTAGE(Cell37v);  CHECK_VOLTAGE(Cell38v);  CHECK_VOLTAGE(Cell39v);  CHECK_VOLTAGE(Cell40v);
    CHECK_VOLTAGE(Cell41v);  CHECK_VOLTAGE(Cell42v);  CHECK_VOLTAGE(Cell43v);  CHECK_VOLTAGE(Cell44v);
    CHECK_VOLTAGE(Cell45v);  CHECK_VOLTAGE(Cell46v);  CHECK_VOLTAGE(Cell47v);  CHECK_VOLTAGE(Cell48v);
    CHECK_VOLTAGE(Cell49v);  CHECK_VOLTAGE(Cell50v);  CHECK_VOLTAGE(Cell51v);  CHECK_VOLTAGE(Cell52v);
    CHECK_VOLTAGE(Cell53v);  CHECK_VOLTAGE(Cell54v);  CHECK_VOLTAGE(Cell55v);  CHECK_VOLTAGE(Cell56v);
    CHECK_VOLTAGE(Cell57v);  CHECK_VOLTAGE(Cell58v);  CHECK_VOLTAGE(Cell59v);  CHECK_VOLTAGE(Cell60v);
    CHECK_VOLTAGE(Cell61v);  CHECK_VOLTAGE(Cell62v);  CHECK_VOLTAGE(Cell63v);  CHECK_VOLTAGE(Cell64v);
    CHECK_VOLTAGE(Cell65v);  CHECK_VOLTAGE(Cell66v);  CHECK_VOLTAGE(Cell67v);  CHECK_VOLTAGE(Cell68v);
    CHECK_VOLTAGE(Cell69v);  CHECK_VOLTAGE(Cell70v);  CHECK_VOLTAGE(Cell71v);  CHECK_VOLTAGE(Cell72v);
    CHECK_VOLTAGE(Cell73v);  CHECK_VOLTAGE(Cell74v);  CHECK_VOLTAGE(Cell75v);  CHECK_VOLTAGE(Cell76v);
    CHECK_VOLTAGE(Cell77v);  CHECK_VOLTAGE(Cell78v);  CHECK_VOLTAGE(Cell79v);  CHECK_VOLTAGE(Cell80v);
    CHECK_VOLTAGE(Cell81v);  CHECK_VOLTAGE(Cell82v);  CHECK_VOLTAGE(Cell83v);  CHECK_VOLTAGE(Cell84v);
    CHECK_VOLTAGE(Cell85v);  CHECK_VOLTAGE(Cell86v);  CHECK_VOLTAGE(Cell87v);  CHECK_VOLTAGE(Cell88v);
    CHECK_VOLTAGE(Cell89v);  CHECK_VOLTAGE(Cell90v);  CHECK_VOLTAGE(Cell91v);  CHECK_VOLTAGE(Cell92v);
    CHECK_VOLTAGE(Cell93v);  CHECK_VOLTAGE(Cell94v);  CHECK_VOLTAGE(Cell95v);  CHECK_VOLTAGE(Cell96v);
    CHECK_VOLTAGE(Cell97v);  CHECK_VOLTAGE(Cell98v);  CHECK_VOLTAGE(Cell99v);  CHECK_VOLTAGE(Cell100v);
    CHECK_VOLTAGE(Cell101v); CHECK_VOLTAGE(Cell102v); CHECK_VOLTAGE(Cell103v); CHECK_VOLTAGE(Cell104v);
    CHECK_VOLTAGE(Cell105v); CHECK_VOLTAGE(Cell106v); CHECK_VOLTAGE(Cell107v); CHECK_VOLTAGE(Cell108v);

    #undef CHECK_VOLTAGE

    //return lowestVoltage;
}

void PSA50kWhBMS::findlowestcelltemp(){
// Function to find the lowest temp
   lowestTemp = 120; //reset value 

    // Using a macro to simplify the comparison process
    #define CHECK_TEMP(cell) if ((cell < lowestTemp) && (cell > 20)) lowestTemp = cell
    
    CHECK_TEMP(Cell1t);   CHECK_TEMP(Cell2t);   CHECK_TEMP(Cell3t);   CHECK_TEMP(Cell4t);
    CHECK_TEMP(Cell5t);   CHECK_TEMP(Cell6t);   CHECK_TEMP(Cell7t);   CHECK_TEMP(Cell8t);
    CHECK_TEMP(Cell9t);   CHECK_TEMP(Cell10t);  CHECK_TEMP(Cell11t);  CHECK_TEMP(Cell12t);
    CHECK_TEMP(Cell13t);  CHECK_TEMP(Cell14t);  CHECK_TEMP(Cell15t);  CHECK_TEMP(Cell16t);
    CHECK_TEMP(Cell17t);  CHECK_TEMP(Cell18t);  CHECK_TEMP(Cell19t);  CHECK_TEMP(Cell20t);
    CHECK_TEMP(Cell21t);  CHECK_TEMP(Cell22t);  CHECK_TEMP(Cell23t);  CHECK_TEMP(Cell24t);
    CHECK_TEMP(Cell25t);  CHECK_TEMP(Cell26t);  CHECK_TEMP(Cell27t);  CHECK_TEMP(Cell28t);
    CHECK_TEMP(Cell29t);  CHECK_TEMP(Cell30t);  CHECK_TEMP(Cell31t);  CHECK_TEMP(Cell32t);
    CHECK_TEMP(Cell33t);  CHECK_TEMP(Cell34t);  CHECK_TEMP(Cell35t);  CHECK_TEMP(Cell36t);
    CHECK_TEMP(Cell37t);  CHECK_TEMP(Cell38t);  CHECK_TEMP(Cell39t);  CHECK_TEMP(Cell40t);
    CHECK_TEMP(Cell41t);  CHECK_TEMP(Cell42t);  CHECK_TEMP(Cell43t);  CHECK_TEMP(Cell44t);
    CHECK_TEMP(Cell45t);  CHECK_TEMP(Cell46t);  CHECK_TEMP(Cell47t);  CHECK_TEMP(Cell48t);
    CHECK_TEMP(Cell49t);  CHECK_TEMP(Cell50t);  CHECK_TEMP(Cell51t);  CHECK_TEMP(Cell52t);
    CHECK_TEMP(Cell53t);  CHECK_TEMP(Cell54t); 


    
    #undef CHECK_TEMP

    //return lowestVoltage;
}

void PSA50kWhBMS::findhighestcelltemp(){
// Function to find the highest temp
   highestTemp = 30; //reset value 

    // Using a macro to simplify the comparison process
    #define CHECK_TEMP(cell) if ((cell > highestTemp) && (cell > 20)) highestTemp = cell
    
    CHECK_TEMP(Cell1t);   CHECK_TEMP(Cell2t);   CHECK_TEMP(Cell3t);   CHECK_TEMP(Cell4t);
    CHECK_TEMP(Cell5t);   CHECK_TEMP(Cell6t);   CHECK_TEMP(Cell7t);   CHECK_TEMP(Cell8t);
    CHECK_TEMP(Cell9t);   CHECK_TEMP(Cell10t);  CHECK_TEMP(Cell11t);  CHECK_TEMP(Cell12t);
    CHECK_TEMP(Cell13t);  CHECK_TEMP(Cell14t);  CHECK_TEMP(Cell15t);  CHECK_TEMP(Cell16t);
    CHECK_TEMP(Cell17t);  CHECK_TEMP(Cell18t);  CHECK_TEMP(Cell19t);  CHECK_TEMP(Cell20t);
    CHECK_TEMP(Cell21t);  CHECK_TEMP(Cell22t);  CHECK_TEMP(Cell23t);  CHECK_TEMP(Cell24t);
    CHECK_TEMP(Cell25t);  CHECK_TEMP(Cell26t);  CHECK_TEMP(Cell27t);  CHECK_TEMP(Cell28t);
    CHECK_TEMP(Cell29t);  CHECK_TEMP(Cell30t);  CHECK_TEMP(Cell31t);  CHECK_TEMP(Cell32t);
    CHECK_TEMP(Cell33t);  CHECK_TEMP(Cell34t);  CHECK_TEMP(Cell35t);  CHECK_TEMP(Cell36t);
    CHECK_TEMP(Cell37t);  CHECK_TEMP(Cell38t);  CHECK_TEMP(Cell39t);  CHECK_TEMP(Cell40t);
    CHECK_TEMP(Cell41t);  CHECK_TEMP(Cell42t);  CHECK_TEMP(Cell43t);  CHECK_TEMP(Cell44t);
    CHECK_TEMP(Cell45t);  CHECK_TEMP(Cell46t);  CHECK_TEMP(Cell47t);  CHECK_TEMP(Cell48t);
    CHECK_TEMP(Cell49t);  CHECK_TEMP(Cell50t);  CHECK_TEMP(Cell51t);  CHECK_TEMP(Cell52t);
    CHECK_TEMP(Cell53t);  CHECK_TEMP(Cell54t); 


    
    #undef CHECK_TEMP

    //return lowestVoltage;
}