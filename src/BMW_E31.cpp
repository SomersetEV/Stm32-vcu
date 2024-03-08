/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2023 Damien Maguire
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
 #include "BMW_E31.h"
 #include "hwinit.h"
 #include <libopencm3/stm32/timer.h>
 #include <libopencm3/stm32/gpio.h>
/*
*E31 840CI Tacho:
*1000RPM = 70Hz
*2000RPM = 140Hz
*5000RPM = 345Hz
*6000RPM = 413Hz
*/


//We use this as an init function
void Bmw_E31::SetCanInterface(CanHardware* c)
{
   c = c;
   tim_setup();//Fire up timer one...
   timer_disable_counter(TIM1);//...but disable until needed
   //note we are trying to reuse the lexus gs450h oil pump pwm output here to drive the tach
   //Be aware this will prevent combo of E31 and GS450H for now ...
   timerIsRunning=false;

   can->RegisterUserMessage(0x153);//ASC message. Will confirm.
}


void Bmw_E31::SetRevCounter(int speed)
{

   uint16_t speed_input = speed;
   speed_input = MAX(750, speed_input);//
   speed_input = MIN(7500, speed_input);
   timerPeriod = 30000000 / speed_input; //TODO: find correct factor or make parameter. current gives 52Hz at 750rpm.
   timer_set_period(TIM1, timerPeriod);
   timer_set_oc_value(TIM1, TIM_OC1, timerPeriod / 2); //always stay at 50% duty cycle

}


void Bmw_E31::SetTemperatureGauge(float temp)
{
   float dc = temp * 10; //TODO find right factor for value like 0..0.5 or so
   //Would like to use digi pots here
   dc = dc;
}

void Bmw_E31::DecodeCAN(int id, uint32_t* data)
{
   uint8_t* bytes = (uint8_t*)data;//E31 CAN to be added here

     if (id == 0x153)// ASC1 contains road speed signal. Unsure if applies to E31 as yet ....
    {
      //Vehicle speed signal in Km/h
      //Calculation = ( (HEX[MSB] * 256) + HEX[LSB]) * 0.0625
      //Min: 0x160 (0 Km/h)

      float road_speed = 0.0625f * (((bytes[2] << 8) | (bytes[1])) - 0x160);

      Param::SetFloat(Param::Veh_Speed, road_speed);
    }
}

 void Bmw_E31::Task1Ms()
{


}

void Bmw_E31::Task100Ms()
{

   if(Param::GetInt(Param::T15Stat) && !timerIsRunning)
   {
   timer_enable_counter(TIM1);
   timerIsRunning=true;
   }
   else if (!Param::GetInt(Param::T15Stat) && timerIsRunning)
   {
   timer_disable_counter(TIM1);
   timerIsRunning=false;
   }

}


 bool Bmw_E31::Ready()
{
   return DigIo::t15_digi.Get();
}

bool Bmw_E31::Start()
{
   return Param::GetBool(Param::din_start);
}
