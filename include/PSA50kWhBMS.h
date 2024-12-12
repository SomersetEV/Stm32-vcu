/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2022 Charlie Smurthwaite
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

#ifndef PSA50kWhBMS_H
#define PSA50kWhBMS_H
#include <stdint.h>
#include "bms.h"

class PSA50kWhBMS: public BMS
{
   public:
      void SetCanInterface(CanHardware* can) override;
      void DecodeCAN(int id, uint8_t * data) override;
      float MaxChargeCurrent();
      void Task100Ms();
      void findlowestcellvoltage();
      void findhighestcellvoltage();
      void findlowestcelltemp();
      void findhighestcelltemp();
   private:
      bool BMSDataValid();
      bool ChargeAllowed();
      int chargeCurrentLimit = 0;
      int timeoutCounter = 0;
      float current = 0;
      float batterycurrent;
      float batteryvoltage;
      float udcvoltage;
      float SoCValue;
      int Cell1v, Cell2v, Cell3v, Cell4v, Cell5v, Cell6v, Cell7v, Cell8v, Cell9v, Cell10v;
      int Cell11v, Cell12v, Cell13v, Cell14v, Cell15v, Cell16v, Cell17v, Cell18v, Cell19v, Cell20v;
      int Cell21v, Cell22v, Cell23v, Cell24v, Cell25v, Cell26v, Cell27v, Cell28v, Cell29v, Cell30v;
      int Cell31v, Cell32v, Cell33v, Cell34v, Cell35v, Cell36v, Cell37v, Cell38v, Cell39v, Cell40v;
      int Cell41v, Cell42v, Cell43v, Cell44v, Cell45v, Cell46v, Cell47v, Cell48v, Cell49v, Cell50v;
      int Cell51v, Cell52v, Cell53v, Cell54v, Cell55v, Cell56v, Cell57v, Cell58v, Cell59v, Cell60v;
      int Cell61v, Cell62v, Cell63v, Cell64v, Cell65v, Cell66v, Cell67v, Cell68v, Cell69v, Cell70v;
      int Cell71v, Cell72v, Cell73v, Cell74v, Cell75v, Cell76v, Cell77v, Cell78v, Cell79v, Cell80v;
      int Cell81v, Cell82v, Cell83v, Cell84v, Cell85v, Cell86v, Cell87v, Cell88v, Cell89v, Cell90v;
      int Cell91v, Cell92v, Cell93v, Cell94v, Cell95v, Cell96v, Cell97v, Cell98v, Cell99v, Cell100v;
      int Cell101v, Cell102v, Cell103v, Cell104v, Cell105v, Cell106v, Cell107v, Cell108v;

      int Cell1t, Cell2t, Cell3t, Cell4t, Cell5t, Cell6t, Cell7t, Cell8t, Cell9t, Cell10t;
      int Cell11t, Cell12t, Cell13t, Cell14t, Cell15t, Cell16t, Cell17t, Cell18t, Cell19t, Cell20t;
      int Cell21t, Cell22t, Cell23t, Cell24t, Cell25t, Cell26t, Cell27t, Cell28t, Cell29t, Cell30t;
      int Cell31t, Cell32t, Cell33t, Cell34t, Cell35t, Cell36t, Cell37t, Cell38t, Cell39t, Cell40t;
      int Cell41t, Cell42t, Cell43t, Cell44t, Cell45t, Cell46t, Cell47t, Cell48t, Cell49t, Cell50t;
      int Cell51t, Cell52t, Cell53t, Cell54t;


   
      float lowestVoltage;
      float highestVoltage;
      float lowestTemp;
      float highestTemp;
      
      float minCellV = 0;
      float maxCellV = 0;
      float minTempC = 0;
      float maxTempC = 0;
};
#endif // PSA50kWhBMS_H
