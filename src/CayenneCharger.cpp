/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2023  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
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
 *
 *Control of the Cayenne OBC Charger. By Ben Bament
 *
 */


#include <CayenneCharger.h>



bool CayenneCharger::ControlCharge(bool RunCh, bool ACReq)
{
  int chgmode = Param::GetInt(Param::interface);
  switch(chgmode)
    {
   case Unused:
   if (HVLM_Plug_Status > 1 && RunCh)
   {
       clearToStart=true;
       return true;
   }
    else
    {
        clearToStart=false;
        return false;
    }

    break;
   case i3LIM:
   if(RunCh && ACReq)//we have a startup request to AC charge from a charge interface
   {
     clearToStart=true;
     return true;
   }
   else
   {
     clearToStart=false;
     return false;
   }
    break;

    case Chademo:
   if (HVLM_Plug_Status > 1 && RunCh)
   {
       clearToStart=true;
       return true;
   }
    else
    {
        clearToStart=false;
        return false;
    }

    break;

    }

}
   void CayenneCharger::Task10Ms()
   {
   msg191();   // BMS_01   0x191
   }


   void CayenneCharger::Task100Ms()

    {
    msg3C0(); //  Klemmen_Status_01
  msg503(); // HVK_01     0x503
  msg1A1(); // BMS_02   0x1A1
  CalcValues100ms();
    }
   //messages with no CRC counter
   void CayenneCharger::msg191() // BMS_01   0x191
{
    uint8_t bytes[8];
  bytes[0] = 0x00 ;
  bytes[1] = (BMS_01[1] | vag_cnt191) ;
  bytes[2] = BMS_01[2] ;
  bytes[3] = BMS_01[3] ;
  bytes[4] = BMS_01[4] ;
  bytes[5] = BMS_01[5] ;
  bytes[6] = BMS_01[6] ;
  bytes[7] = BMS_01[7] ;
  bytes[0] = vw_crc_calc(bytes, 8, 0x191);
  can->Send(0x191, (uint32_t*)bytes, 8);
  vag_cnt191++;
  if (vag_cnt191 > 0x0f) vag_cnt191 = 0x00;
}

   void CayenneCharger::msg1A1() // BMS_02   0x1A1
{
  uint8_t bytes[8];
  bytes[0] = BMS_02[0] ;
  bytes[1] = BMS_02[1] ;
  bytes[2] = BMS_02[2] ;
  bytes[3] = BMS_02[3] ;
  bytes[4] = BMS_02[4] ;
  bytes[5] = BMS_02[5] ;
  bytes[6] = BMS_02[6] ;
  bytes[7] = BMS_02[7] ;
  can->Send(0x1A1, (uint32_t*)bytes, 8);
}

void CayenneCharger::msg64F() // BCM1_04 - 0x64F
{
  uint8_t bytes[8];
  bytes[0] = 0x00 ;
  bytes[1] = 0x00 ;
  bytes[2] = 0x00 ;
  bytes[3] = 0x00 ;
  bytes[4] = 0x00 ;
  bytes[5] = 0x00 ;
  bytes[6] = 0x00 ;
  bytes[7] = 0x00 ;
  can->Send(0x64F, (uint32_t*)bytes, 8);
}

void CayenneCharger::msg663() // NVEM_02 - 0x663
{
  uint8_t bytes[8];
  bytes[0] = 0x00 ;
  bytes[1] = 0x11 ;
  bytes[2] = 0x22 ;
  bytes[3] = 0x33 ;
  bytes[4] = 0x44 ;
  bytes[5] = 0x55 ;
  bytes[6] = 0x66 ;
  bytes[7] = 0x77 ;
  can->Send(0x663, (uint32_t*)bytes, 8);
}

   //Messages with CRC and counters
void CayenneCharger::msg503() // HVK_01     0x503
{
  uint8_t bytes[8];
  bytes[0] = 0x00;
  bytes[1] = (HVK_01[1] | vag_cnt503) ;
  bytes[2] = HVK_01[2] ;
  bytes[3] = HVK_01[3] ;
  bytes[4] = HVK_01[4] ;
  bytes[5] = HVK_01[5] ;
  bytes[6] = HVK_01[6] ;
  bytes[7] = HVK_01[7] ;
  bytes[0] = vw_crc_calc(bytes, 8, 0x503);
  can->Send(0x503, (uint32_t*)bytes, 8);
  vag_cnt503++;
  if (vag_cnt503 > 0x0f) vag_cnt503 = 0x00;
}


void CayenneCharger::msg3C0() // Klemmen_Status_01
{
  uint8_t bytes[8];
  bytes[0] = 0x00 ;
  bytes[1] = (Klemmen_Status_01[1] | vag_cnt3C0) ;
  bytes[2] = Klemmen_Status_01[2] ;
  bytes[3] = Klemmen_Status_01[3] ;
  bytes[4] = Klemmen_Status_01[4] ;
  bytes[5] = Klemmen_Status_01[5] ;
  bytes[6] = Klemmen_Status_01[6] ;
  bytes[7] = Klemmen_Status_01[7] ;
  bytes[0] = vw_crc_calc(bytes, 8, 0x3C0);
  can->Send(0x3C0, (uint32_t*)bytes, 8);
  vag_cnt3C0++;
  if (vag_cnt3C0 > 0x0f) vag_cnt3C0 = 0x00;
}
   void CayenneCharger::UnLockCP()
{

  ZV_FT_entriegeln = 1;
  ZV_entriegeln_Anf = 1;
  FCU_TK_Freigabe_Tankklappe = 1;
  ZV_verriegelt_extern_ist = 0;
}
void CayenneCharger::LockCP()
{
  ZV_FT_verriegeln = 1;
  ZV_verriegelt_extern_ist = 1;
  ZV_entriegeln_Anf = 0;
  FCU_TK_Freigabe_Tankklappe = 0;
}

void CayenneCharger::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x488);
   can->RegisterUserMessage(0x53C);
   can->RegisterUserMessage(0x564);
   can->RegisterUserMessage(0x565);
   can->RegisterUserMessage(0x67E);
   can->RegisterUserMessage(0x12DD5472);
   can->RegisterUserMessage(0x1B000044);
}


   void CayenneCharger::DecodeCAN(int id, uint32_t data[2])
{
  switch (id)
  {
    case 0x488: // HVLM_06
     CayenneCharger::handle488(data);
      break;

    case 0x53C: // HVLM_04
        CayenneCharger::handle53C(data);
      break;

    case 0X564: // LAD_01
         CayenneCharger::handle564(data);
      break;

    case 0x565: // HVLM_03
        CayenneCharger::handle565(data);
      break;

    case 0x67E: // LAD_02
       CayenneCharger::handle67E(data);
      break;

    case 0x12DD5472: // HVLM_10
        CayenneCharger::handle12DD5472(data);
      break;

    case 0x1B000044: //NMH_Ladegaeraet. Wake signal
        CayenneCharger::handle1B000044(data);
      break;
  }

}

void CayenneCharger::handle488(uint32_t data[2])

{
   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes.
      HVLM_MaxDC_ChargePower = (((bytes[2] & (0x3FU)) << 4) | ((bytes[1] >> 4) & (0x0FU))) * 250;
      HVLM_Max_DC_Voltage_DCLS = ((bytes[3] & (0xFFU)) << 2) | ((bytes[2] >> 6) & (0x03U));
      HVLM_Actual_DC_Current_DCLS = ((bytes[5] & (0x01U)) << 8) | (bytes[4] & (0xFFU));
      HVLM_Max_DC_Current_DCLS = ((bytes[6] & (0x03U)) << 7) | ((bytes[5] >> 1) & (0x7FU));
      HVLM_Min_DC_Voltage_DCLS = ((bytes[7] & (0x07U)) << 6) | ((bytes[6] >> 2) & (0x3FU));
      HVLM_Min_DC_Current_DCLS = ((bytes[7] >> 3) & (0x1FU));

}

void CayenneCharger::handle53C(uint32_t data[2])

{
   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes.
      // HVLM_ParkingHeater_Mode = ((HVLM_04[1] >> 4) & (0x07U));
      // HVLM_StationaryClimat_Timer_Stat = ((HVLM_04[1] >> 7) & (0x01U));
      // HVLM_HVEM_MaxPower = ((HVLM_04[3] & (0x01U)) << 8) | (HVLM_04[2] & (0xFFU));
      HVLM_Status_Grid = ((bytes[3] >> 1) & (0x01U));
      // HVLM_BEV_LoadingScreen = ((HVLM_04[3] >> 2) & (0x01U));
      HVLM_EnergyFlowType = ((bytes[3] >> 3) & (0x03U));
      // HVLM_VK_ParkingHeaterStatus = ((HVLM_04[3] >> 5) & (0x07U));
      // HVLM_VK_ClimateConditioningStat = (HVLM_04[4] & (0x03U));
      HVLM_OperationalMode = ((bytes[4] >> 2) & (0x03U));
      HVLM_HV_ActivationRequest = ((bytes[4] >> 4) & (0x03U));
      HVLM_ChargerErrorStatus = ((bytes[5] & (0x01U)) << 2) | ((bytes[4] >> 6) & (0x03U));
      HVLM_Park_Request = ((bytes[5] >> 1) & (0x07U));
      HVLM_Park_Request_Maintain = ((bytes[5] >> 4) & (0x03U));
      // HVLM_AWC_Mode = (HVLM_04[6] & (0x07U));
      HVLM_Plug_Status = ((bytes[6] >> 3) & (0x03U));
      HVLM_LoadRequest = ((bytes[6] >> 5) & (0x07U));
      HVLM_MaxBattChargeCurrent = (bytes[7] & (0xFFU));

}

void CayenneCharger::handle564(uint32_t data[2])

{
   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes.
      mode = ((bytes[1] >> 4) & (0x07U));
      ACvoltage = ((bytes[2] & (0xFFU)) << 1) | ((bytes[1] >> 7) & (0x01U));
      Param::SetFloat(Param::AC_Volts , ACvoltage);
      HVVoltage = (((bytes[4] & (0x03U)) << 8) | (bytes[3] & (0xFFU)));
      current = ((((bytes[5] & (0x0FU)) << 6) | ((bytes[4] >> 2) & (0x3FU))) * 0.2) - 102;
      LAD_Status_Voltage = ((bytes[5] >> 4) & (0x03U));
      temperature = bytes[6] - 40;
      Param::SetFloat(Param::ChgTemp , temperature);
      LAD_PowerLossVal = ((bytes[7] & (0xFFU))) * 20;

}

void CayenneCharger::handle565(uint32_t data[2])

{
      uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes.
      HVLM_HV_StaleTime = ((bytes[0] & (0xFFU))) * 4;
      HVLM_ChargeSystemState = (bytes[1] & (0x03U));
      // HVLM_KESSY_KeySearch = ((HVLM_03[1] >> 2) & (0x03U));
      HVLM_Status_LED = ((bytes[1] >> 4) & (0x0FU));
      MaxACAmps = ((bytes[3] & (0x7FU))) / 2; // Max amps from EVSE
      HVLM_LG_ChargerTargetMode = ((bytes[3] >> 7) & (0x01U));
      HVLM_TankCapReleaseRequest = (bytes[4] & (0x03U));
      HVLM_RequestConnectorLock = ((bytes[4] >> 2) & (0x03U));
      HVLM_Start_VoltageMeasure_DCLS = ((bytes[4] >> 4) & (0x03U));
      // PnC_Trigger_OBC_cGW = ((HVLM_03[5] & (0x03U)) << 2) | ((HVLM_03[4] >> 6) & (0x03U));
      // HVLM_ReleaseAirConditioning = ((HVLM_03[5] >> 2) & (0x03U));
      HVLM_ChargeReadyStatus = ((bytes[6] >> 1) & (0x07U));
      // HVLM_IsolationRequest = ((HVLM_03[6] >> 5) & (0x01U));
      HVLM_Output_Voltage_HV = ((bytes[7] & (0xFFU)) << 2) | ((bytes[6] >> 6) & (0x03U));
}

void CayenneCharger::handle67E(uint32_t data[2])

{
      uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes.
    LAD_Reduction_ChargerTemp = ((bytes[1] >> 4) & (0x01U));
      LAD_Reduction_Current = ((bytes[1] >> 5) & (0x01U));
      LAD_Reduction_SocketTemp = ((bytes[1] >> 6) & (0x01U));
      LAD_MaxChargerPower_HV = (((bytes[3] & (0x01U)) << 8) | (bytes[2] & (0xFFU))) * 100;
      PPLim = (bytes[4] & (0x07U));
      LAD_ControlPilotStatus = ((bytes[4] >> 3) & (0x01U));
      LAD_LockFeedback = ((bytes[4] >> 4) & (0x01U));
      LAD_ChargerCoolingDemand = ((bytes[4] >> 6) & (0x03U));
      // LAD_MaxLadLeistung_HV_Offset = ((LAD_02[7] >> 1) & (0x03U));
      LAD_ChargerWarning = ((bytes[7] >> 6) & (0x01U));
      LAD_ChargerFault = ((bytes[7] >> 7) & (0x01U));
}

void CayenneCharger::handle12DD5472(uint32_t data[2])

{
      uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes.
    HVLM_RtmWarnLadeverbindung = ((bytes[4] & (0x03U)) << 1) | ((bytes[3] >> 7) & (0x01U));
      HVLM_RtmWarnLadesystem = ((bytes[4] >> 2) & (0x07U));
      HVLM_RtmWarnLadestatus = ((bytes[4] >> 5) & (0x07U));
      HVLM_RtmWarnLadeKommunikation = (bytes[5] & (0x07U));
}

void CayenneCharger::handle1B000044(uint32_t data[2])

{
      uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes.
    carwakeup = bytes[7];
}



   void CayenneCharger::CalcValues100ms() // Run to calculate values every 100 ms
{
  // Runtime Values:
  BMS_Batt_Curr = (current + 2047);

 // BMS_SOC_HiRes = (battery_status.SOCx10) * 2;
 // BMS_SOC_Kaltstart = (battery_status.SOCx10) * 2;

  //BMS Limits Discharge:
  BMS_MaxDischarge_Curr = 1500;
 // BMS_Min_Batt_Volt = (battery_status.BMSMinVolt);
 // BMS_Min_Batt_Volt_Discharge = (battery_status.BMSMinVolt);
  //BMS Limits Charge:
  if(clearToStart)
  {
      if(actVolts<Param::GetInt(Param::Voltspnt)) BMS_MaxCharge_Curr++;
      if(actVolts>=Param::GetInt(Param::Voltspnt)) BMS_MaxCharge_Curr--;
      if(BMS_MaxCharge_Curr>=GetInt(Param::BMS_ChargeLim)) BMS_MaxCharge_Curr = GetInt(Param::BMS_ChargeLim);//clamp to max of BMS charge limit
  }

  else
  {
   BMS_MaxCharge_Curr = 0;
  }

  HVEM_SollStrom_HV = 50;
  BMS_MaxCharge_Curr_Offset = 0;
  BMS_Batt_Max_Volt = GetInt(Param::Voltspnt);  //(HVDCSetpnt);
  BMS_Min_Batt_Volt_Charge = GetInt(Param::Voltspnt) - 200;
 // BMS_OpenCircuit_Volts = (BMSBattCellSumx10) / 10;


  //BMS_Status_ServiceDisconnect = (battery_status.HVIL_Open);

//  BMS_Faultstatus = (battery_status.BMS_Status);
 // BMS_Batt_Ah = (battery_status.BMSCellAhx10) / 2;
  //BMS_Target_SOC_HiRes = (battery_status.SOC_Targetx10) * 2;

  //BMS_Batt_Temp = ((battery_status.BMS_Battery_Tempx10) + 400) / 5;
  //BMS_CurrBatt_Temp = ((battery_status.BMS_Battery_Tempx10) + 400) / 5;
  //BMS_CoolantTemp_Act = ((battery_status.BMS_Coolant_Tempx10) + 400) / 5;
  //BMS_Batt_Energy = (battery_status.CapkWhx10) * 2;
  BMS_Battdiag = 0;

//  BMS_Max_Wh = (battery_status.CapkWhx10 * 2);
  BMS_BattEnergy_Wh_HiRes = 0;
  BMS_MaxBattEnergy_Wh_HiRes = 0;
  //BMS_SOC = (battery_status.SOCx10) / 5;
  BMS_ResidualEnergy_Wh = 0;

  //BMS_SOC_ChargeLim = (battery_status.SOC_Targetx10) / 10;
  BMS_EnergyCount = 0;
  //BMS_EnergyReq_Full = ((battery_status.SOC_Targetx10 - battery_status.SOCx10) * battery_status.CapkWhx10) / 2500;
  BMS_ChargePowerMax = 625;
  BMS_ChargeEnergyCount = 0;

 // BMS_BattCell_Temp_Max = ((battery_status.BMS_Cell_H_Tempx10) + 400) / 5;
//  BMS_BattCell_Temp_Min = ((battery_status.BMS_Cell_L_Tempx10) + 400) / 5;
 // BMS_BattCell_MV_Max = (battery_status.BMS_Cell_H_mV) - 1000;
 // BMS_BattCell_MV_Min = (battery_status.BMS_Cell_L_mV) - 1000;

  HVEM_Nachladen_Anf = false; // Request for HV charging with plugged in connector and deactivated charging request
  //HVEM_SollStrom_HV =;  // Target current charging on the HV side
  //HVEM_MaxSpannung_HV =; // Maximum charging voltage to the charger or DC charging station

  if (HVLM_Park_Request = 1)
  {
    if (HMS_Systemstatus = 2) HMS_Systemstatus = 3 ;
    else HMS_Systemstatus = 2 ; //0 "No_function_active" 1 "Hold_active" 2 "Parking_requested" 3 "Parking_active" 4 "Keep parking_active" 5 "Start_active" 6 "Release_request_active" 7 "Release_request_by_driver" 8 "Slipping_detected" 9 "Hold_standby_active" 10 "Start_standby_active" 14 "Init" 15 "Error " ;
    if (HMS_Systemstatus < 1)
    {
      HMS_aktives_System = 6; //0 "No_System__Init_Error" 1 "Driver request_active" 2 "HMS_internal_active" 3 "ACC_active" 4 "Autohold_active" 5 "HHC_active" 6 "HVLM_active" 7 "Getriebe_aktiv" 8 "EBKV_aktiv" 9 "ParkAssist_aktiv" 10 "ARA_aktiv" 12 "Autonomous_Hold_aktiv" 13 "STA_aktiv " 14 "Motor_aktiv" 15 "EA_aktiv" 16 "VLK_aktiv" ;
    }
    else
    {
      HMS_aktives_System = 0;
    }
    //HMS_Fehlerstatus = false; //0 "No error" 1 "Stopping_not_possible" 2 "Special operating mode_active" 3 "System restriction" 4 "System fault" ;
  }

  if ( HVLM_HV_ActivationRequest = 1)
  {
    HV_Bordnetz_aktiv = true; // Indicates an active high-voltage vehicle electrical system: 0 = Not Active,  1 = Active
    HVK_BMS_Sollmodus = 4;
    BMS_IstModus = 4; // 0=Standby, 1=HV Active (Driving) 2=Balancing 4=AC charge, 6=DC charge, 7=init
    BMS_HV_Status = 2; // HV System Voltage Detected  // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    HVK_MO_EmSollzustand = 50;
    BMS_Charger_Active = 1;
    BMS_Batt_Volt = 400 * 4;
    BMS_Batt_Volt_HVterm = 400 * 2;
    if (HVVoltage > 250)
    {
      BMS_Batt_Volt = (HVVoltage) * 4;
      BMS_Batt_Volt_HVterm = (HVVoltage) * 2;
    }
  }

  if ( HVLM_HV_ActivationRequest = 0)
  {
    HV_Bordnetz_aktiv = false; // Indicates an active high-voltage vehicle electrical system: 0 = Not Active,  1 = Active
    HVK_BMS_Sollmodus = 0;
    BMS_IstModus = 0; // 0=Standby, 1=HV Active (Driving) 2=Balancing 4=AC charge, 6=DC charge, 7=init
    BMS_HV_Status = 1; // HV No Voltage // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    HVK_MO_EmSollzustand = 50;
    BMS_Charger_Active = 0;
    BMS_Batt_Volt = (HVVoltage) * 4;
    BMS_Batt_Volt_HVterm = (HVVoltage) * 2;
  }
  if (BMS_HV_Status = 1)
  {
    HVK_DCDC_Sollmodus = 2; // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    EM1_Status_Spgfreiheit = 2; // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    HVK_Gesamtst_Spgfreiheit = 2; // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
  }
  if (BMS_HV_Status = 0)
  {
    HVK_DCDC_Sollmodus = 1; // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    EM1_Status_Spgfreiheit = 1; // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
    HVK_Gesamtst_Spgfreiheit = 1; // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage
  }

  switch (chargeractive)
  {
    case 0:     // Charger Standby
      HVK_HVLM_Sollmodus = false; // Requested target mode of the charging manager: 0=Not Enabled, 1=Enabled
      break;

    case 1:     // HV Active - Charger Active
      //HVEM_Nachladen_Anf = true; // Request for HV charging with plugged in connector and deactivated charging request
      HVK_HVLM_Sollmodus = true; // Requested target mode of the charging manager: 0=Not Enabled, 1=Enabled

      break;
  }

  //  BMS_01
  BMS_01[0] = 0x00;
  BMS_01[1] = (0x00 & (0x0FU)) | ((BMS_Batt_Curr & (0x0FU)) << 4);
  BMS_01[2] = ((BMS_Batt_Curr >> 4) & (0xFFU));
  BMS_01[3] = (BMS_Batt_Volt & (0xFFU));
  BMS_01[4] = ((BMS_Batt_Volt >> 8) & (0x0FU)) | ((BMS_Batt_Volt_HVterm & (0x0FU)) << 4);
  BMS_01[5] = ((BMS_Batt_Volt_HVterm >> 4) & (0x7FU)) | ((BMS_SOC_HiRes & (0x01U)) << 7);
  BMS_01[6] = ((BMS_SOC_HiRes >> 1) & (0xFFU));
  BMS_01[7] = ((BMS_SOC_HiRes >> 9) & (0x03U)) | ((0x00 & (0x01U)) << 2) | ((0x00 & (0x0FU)) << 4);

  //  BMS_02
  BMS_02[0] = 0x00;
  BMS_02[1] = (BMS_MaxCharge_Curr_Offset & (0x0FU)) | ((BMS_MaxDischarge_Curr & (0x0FU)) << 4);
  BMS_02[2] = ((BMS_MaxDischarge_Curr >> 4) & (0x7FU)) | ((BMS_MaxCharge_Curr & (0x01U)) << 7);
  BMS_02[3] = ((BMS_MaxCharge_Curr >> 1) & (0xFFU));
  BMS_02[4] = ((BMS_MaxCharge_Curr >> 9) & (0x03U)) | ((BMS_Min_Batt_Volt & (0x3FU)) << 2);
  BMS_02[5] = ((BMS_Min_Batt_Volt >> 6) & (0x0FU)) | ((BMS_Min_Batt_Volt_Discharge & (0x0FU)) << 4);
  BMS_02[6] = ((BMS_Min_Batt_Volt_Discharge >> 4) & (0x3FU)) | ((BMS_Min_Batt_Volt_Charge & (0x03U)) << 6);
  BMS_02[7] = ((BMS_Min_Batt_Volt_Charge >> 2) & (0xFFU));

  //  BMS_03
  BMS_03[0] = (BMS_OpenCircuit_Volts & (0xFFU));
  BMS_03[1] = ((BMS_OpenCircuit_Volts >> 8) & (0x03U)) | ((BMS_Batt_Max_Volt & (0x0FU)) << 4);
  BMS_03[2] = ((BMS_Batt_Max_Volt >> 4) & (0x3FU)) | ((BMS_MaxDischarge_Curr & (0x03U)) << 6);
  BMS_03[3] = ((BMS_MaxDischarge_Curr >> 2) & (0xFFU));
  BMS_03[4] = ((BMS_MaxDischarge_Curr >> 10) & (0x01U)) | ((BMS_MaxCharge_Curr & (0x7FU)) << 1);
  BMS_03[5] = ((BMS_MaxCharge_Curr >> 7) & (0x0FU)) | ((BMS_Min_Batt_Volt_Discharge & (0x0FU)) << 4);
  BMS_03[6] = ((BMS_Min_Batt_Volt_Discharge >> 4) & (0x3FU)) | ((BMS_Min_Batt_Volt_Charge & (0x03U)) << 6);
  BMS_03[7] = ((BMS_Min_Batt_Volt_Charge >> 2) & (0xFFU));

  //  BMS_04
  // BMS_IstModus = Target mode 0=Standby, 1=HV Active (Driving) 2=Balancing 4=AC charge, 6=DC charge, 7=init
  BMS_04[0] = 0x00;
  BMS_04[1] = (0x00 & (0x0FU)) | ((BMS_Status_ServiceDisconnect & (0x01U)) << 5) | ((BMS_HV_Status & (0x03U)) << 6);
  BMS_04[2] = 0x00 | ((BMS_IstModus & (0x07U)) << 1) | ((BMS_Faultstatus & (0x07U)) << 4) | ((BMS_Batt_Ah & (0x01U)) << 7);
  BMS_04[3] = ((BMS_Batt_Ah >> 1) & (0xFFU));
  BMS_04[4] = ((BMS_Batt_Ah >> 9) & (0x03U));
  BMS_04[6] = ((BMS_Target_SOC_HiRes & (0x07U)) << 5);
  BMS_04[7] = ((BMS_Target_SOC_HiRes >> 3) & (0xFFU));


  //  BMS_07

  //  BMS_Gesamtst_Spgfreiheit = Volt Free Status
  //  BMS_RIso_Ext = Isolation Resistance

  BMS_07[0] = 0x00;
  BMS_07[1] = (0x00 & (0x0FU)) | ((BMS_Batt_Energy & (0x0FU)) << 4);
  BMS_07[2] = ((BMS_Batt_Energy >> 4) & (0x7FU)) | ((BMS_Charger_Active & (0x01U)) << 7); //BMS_07[2] = ((BMS_Batt_Energy >> 4) & (0x7FU)) | ((BMS_Charger_Active & (0x01U)) << 7);
  BMS_07[3] = (BMS_Battdiag & (0x07U)) | ((BMS_Freig_max_Perf & (0x03U)) << 3) | ((BMS_Balancing_Active & (0x03U)) << 6);
  BMS_07[4] = (BMS_Max_Wh & (0xFFU));
  BMS_07[5] = ((BMS_Max_Wh >> 8) & (0x07U)) | ((0x0 & (0x01U)) << 3) | ((0x00 & (0x03U)) << 4) | ((0x00 & (0x03U)) << 6); //BMS_07[5] = ((BMS_Max_Wh >> 8) & (0x07U)) | ((0x0 & (0x01U)) << 3) | ((BMS_Gesamtst_Spgfreiheit & (0x03U)) << 4) | ((BMS_RIso_Ext & (0x03U)) << 6);
  BMS_07[6] = ((BMS_RIso_Ext >> 2) & (0xFFU));
  //BMS_07[7] = ((BMS_RIso_Ext >> 10) & (0x03U)) | ((BMS_Batt_Warn & (0x03U)) << 2) | ((BMS_Coolant_Leak & (0x03U)) << 4);
  BMS_07[7] = ((BMS_RIso_Ext >> 10) & (0x03U)) | ((0x00 & (0x03U)) << 2) | ((0x00 & (0x03U)) << 4);
  //  BMS_09
  BMS_09[2] = ((BMS_HV_Auszeit_Status & (0x03U)) << 5) | ((BMS_HV_Auszeit & (0x01U)) << 7);
  BMS_09[3] = ((BMS_HV_Auszeit >> 1) & (0xFFU));
  BMS_09[4] = (BMS_Kapazitaet & (0xFFU));
  BMS_09[5] = ((BMS_Kapazitaet >> 8) & (0x07U)) | ((BMS_SOC_Kaltstart & (0x1FU)) << 3);
  BMS_09[6] = ((BMS_SOC_Kaltstart >> 5) & (0x3FU)) | ((BMS_max_Grenz_SOC & (0x03U)) << 6);
  BMS_09[7] = ((BMS_max_Grenz_SOC >> 2) & (0x07U)) | ((BMS_min_Grenz_SOC & (0x1FU)) << 3);

  //  BMS_10
  BMS_10[0] = (BMS_BattEnergy_Wh_HiRes & (0xFFU));
  BMS_10[1] = ((BMS_BattEnergy_Wh_HiRes >> 8) & (0x7FU)) | ((BMS_MaxBattEnergy_Wh_HiRes & (0x01U)) << 7);
  BMS_10[2] = ((BMS_MaxBattEnergy_Wh_HiRes >> 1) & (0xFFU));
  BMS_10[3] = ((BMS_MaxBattEnergy_Wh_HiRes >> 9) & (0x3FU)) | ((BMS_SOC & (0x03U)) << 6);
  BMS_10[4] = ((BMS_SOC >> 2) & (0x3FU)) | ((BMS_ResidualEnergy_Wh & (0x03U)) << 6);
  BMS_10[5] = ((BMS_ResidualEnergy_Wh >> 2) & (0xFFU));
  BMS_10[6] = ((BMS_ResidualEnergy_Wh >> 10) & (0x03U)) | ((0x64 & (0x3FU)) << 2);
  BMS_10[7] = ((0x64 >> 6) & (0x01U)) | ((0x64 & (0x7FU)) << 1);


  //  BMS_11

  BMS_11[0] = 0x00;
  BMS_11[1] = 0x00;
  BMS_11[2] = ((0x02 & (0x0FU)) << 1) | ((0x01 & (0x07U)) << 5);
  BMS_11[3] = (BMS_BattCell_Temp_Max & (0xFFU));
  BMS_11[4] = (BMS_BattCell_Temp_Min & (0xFFU));
  BMS_11[5] = (BMS_BattCell_MV_Max & (0xFFU));
  BMS_11[6] = ((BMS_BattCell_MV_Max >> 8) & (0x0FU)) | ((BMS_BattCell_MV_Min & (0x0FU)) << 4);
  BMS_11[7] = ((BMS_BattCell_MV_Min >> 4) & (0xFFU));

  //  BMS_27

  // BMS_EnergyCount = Progressive Energy counter from 0-13?
  // BMS_ChargeEnergyCount = Same as above for charge
  BMS_27[0] = 0x00;
  BMS_27[1] = 0x00;
  BMS_27[2] = 0x00;
  BMS_27[3] = ((BMS_SOC_ChargeLim & (0x3FU)) << 2);
  BMS_27[4] = ((BMS_SOC_ChargeLim >> 6) & (0x01U)) | ((BMS_EnergyCount & (0x0FU)) << 1) | ((BMS_EnergyReq_Full & (0x07U)) << 5);
  BMS_27[5] = ((BMS_EnergyReq_Full >> 3) & (0xFFU));
  BMS_27[6] = (BMS_ChargePowerMax & (0xFFU));
  BMS_27[7] = ((BMS_ChargePowerMax >> 8) & (0x0FU)) | ((BMS_ChargeEnergyCount & (0x0FU)) << 4);

  //  BMS_DC_01

  //  BMS_Status_DCLS = Status of the voltage monitoring at the DC charging interface | 0=inactive, 1= i.O, 2= n.i.O, 3= Active
  //  BMS_DCLS_Spannung = DC voltage of the charging station. Measurement between the DC HV lines.
  //  BMS_DCLS_MaxLadeStrom = maximum permissible DC charging current

  // BMS_DC_01[0] = (BMS_DC_01_CRC & (0xFFU));
  // BMS_DC_01[1] = (BMS_DC_01_BZ & (0x0FU)) | ((BMS_Status_DCLS & (0x03U)) << 4) | ((BMS_DCLS_Spannung & (0x03U)) << 6);
  // BMS_DC_01[2] = ((BMS_DCLS_Spannung >> 2) & (0xFFU));
  // BMS_DC_01[3] = (BMS_DCLS_MaxLadeStrom & (0xFFU));
  // BMS_DC_01[4] = ((BMS_DCLS_MaxLadeStrom >> 8) & (0x01U));
  // BMS_DC_01[5] = 0x00;
  // BMS_DC_01[6] = 0x00;
  // BMS_DC_01[7] = 0x00;

  //  DCDC_01 - For DC/DC 12V Converter
  //  Intend to mirror HV voltage from main bus (unless found elsewhere)
  //  Charger only seems interested in the 12V output Current & Voltage from module?
  // DCDC_01[0] = 0x00;
  // DCDC_01[1] = (0x00 & (0x0FU)) | ((DC_IstSpannung_HV & (0x0FU)) << 4);
  // DCDC_01[2] = ((DC_IstSpannung_HV >> 4) & (0xFFU));
  // DCDC_01[3] = (DC_IstStrom_HV_02 & (0xFFU));
  // DCDC_01[4] = ((DC_IstStrom_HV_02 >> 8) & (0x03U)) | ((DC_IstStrom_NV & (0x3FU)) << 2);
  // DCDC_01[5] = ((DC_IstStrom_NV >> 6) & (0x0FU));
  // DCDC_01[7] = (DC_IstSpannung_NV & (0xFFU));

  //  DCDC_03 - For DC/DC 12V Converter
  //  Charger only seems interested in the DC_IstModus_02 - Status signal, Set to 0 for standby, or 3 for raise
  // DCDC_03[0] = (DCDC_03_CRC & (0xFFU));
  // DCDC_03[1] = (DCDC_03_BZ & (0x0FU));
  //DCDC_03[2] = (DC_Fehlerstatus & (0x07U)) | ((DC_Peakstrom_verfuegbar & (0x01U)) << 3) | ((DC_Abregelung_Temperatur & (0x01U)) << 4) | ((DC_IstModus_02 & (0x07U)) << 5);
  DCDC_03[2] = (0x00 & (0x07U)) | ((0x00 & (0x01U)) << 3) | ((0x00 & (0x01U)) << 4) | ((DC_IstModus_02 & (0x07U)) << 5);
  // DCDC_03[3] = ((DC_HV_EKK_IstModus & (0x07U)) << 4);
  // DCDC_03[5] = ((DC_Status_Spgfreiheit_HV & (0x03U)) << 6);
  // DCDC_03[6] = (DC_IstSpannung_EKK_HV & (0xFFU));
  // DCDC_03[7] = (DC_Temperatur & (0xFFU));

  //  Dimmung_01
  //  Charger conserned about the 58x signals - Likely just for the Charge port light output, Setting to 100% static for now
  // Dimmung_01[0] = (DI_KL_58xd & (0xFFU));
  // Dimmung_01[1] = (DI_KL_58xs & (0x7FU)) | ((DI_Display_Nachtdesign & (0x01U)) << 7);
  // Dimmung_01[2] = (DI_KL_58xt & (0x7FU));
  // Dimmung_01[3] = (DI_Fotosensor & (0xFFU));
  // Dimmung_01[4] = ((DI_Fotosensor >> 8) & (0xFFU));
  // Dimmung_01[5] = (BCM1_Stellgroesse_Kl_58s & (0x7FU));
  // Dimmung_01[6] = 0x00;
  // Dimmung_01[7] = 0x00;

  //  HVEM_05
  //  HVEM_Nachladen_Anf - Request for HV charging with plugged in connector and deactivated charging request
  //  HVEM_SollStrom_HV - Target current charging on the HV side
  //  HVEM_MaxSpannung_HV - Maximum charging voltage to the charger or DC charging station
  //  HVEM_Abschaltstatus - Climate Reduction - Set to 0 for 100% (No Reduction)
  HVEM_05[1] = 0x00;
  HVEM_05[2] = 0x00;
  HVEM_05[3] = 0x00;
  HVEM_05[4] = (HVEM_Nachladen_Anf & (0x01U)) | ((HVEM_SollStrom_HV & (0x7FU)) << 1);
  HVEM_05[5] = ((HVEM_SollStrom_HV >> 7) & (0x0FU)) | ((HVEM_MaxSpannung_HV & (0x0FU)) << 4);
  HVEM_05[6] = ((HVEM_MaxSpannung_HV >> 4) & (0x3FU)) | ((0x00 & (0x03U)) << 6);
  HVEM_05[7] = 0x00;

  // Authentic_Time_01 & NavData_02
  Authentic_Time_01[4] = (UnixTime & (0xFFU));
  Authentic_Time_01[5] = ((UnixTime >> 8) & (0xFFU));
  Authentic_Time_01[6] = ((UnixTime >> 16) & (0xFFU));
  Authentic_Time_01[7] = ((UnixTime >> 24) & (0xFFU));

  ESP_15[4] = (0x00 & (0x01U)) | ((0x00 & (0x07U)) << 1) | ((HMS_Systemstatus & (0x0FU)) << 4);
  ESP_15[5] = (0x00 & (0x07U)) | ((HMS_aktives_System & (0x1FU)) << 3);
  ESP_15[6] = (0x00 & (0x01U)) | ((0x00 & (0x01U)) << 1) | ((HMS_Fehlerstatus & (0x07U)) << 2) | ((0x00 & (0x01U)) << 5) | ((0x00 & (0x03U)) << 6);

  //Klemmen_Status_01[1] = (0x00& (0x0FU)) | ((RSt_Fahrerhinweise & (0x0FU)) << 4);
  //Klemmen_Status_01[2] = (ZAS_Kl_S & (0x01U)) | ((ZAS_Kl_15 & (0x01U)) << 1) | ((ZAS_Kl_X & (0x01U)) << 2) | ((ZAS_Kl_50_Startanforderung & (0x01U)) << 3) | ((BCM_Remotestart_Betrieb & (0x01U)) << 4) | ((ZAS_Kl_Infotainment & (0x01U)) << 5) | ((BCM_Remotestart_KL15_Anf & (0x01U)) << 6) | ((BCM_Remotestart_MO_Start & (0x01U)) << 7);
  Klemmen_Status_01[2] = (ZAS_Kl_S & (0x01U)) | ((ZAS_Kl_15 & (0x01U)) << 1) | ((ZAS_Kl_X & (0x01U)) << 2) | ((ZAS_Kl_50_Startanforderung & (0x01U)) << 3) | ((0x00 & (0x01U)) << 4) | ((0x00 & (0x01U)) << 5) | ((0x00 & (0x01U)) << 6) | ((0x00 & (0x01U)) << 7);
  //Klemmen_Status_01[3] = (KST_Warn_P1_ZST_def & (0x01U)) | ((KST_Warn_P2_ZST_def & (0x01U)) << 1) | ((KST_Fahrerhinweis_1 & (0x01U)) << 2) | ((KST_Fahrerhinweis_2 & (0x01U)) << 3) | ((BCM_Ausparken_Betrieb & (0x01U)) << 4) | ((KST_Fahrerhinweis_4 & (0x01U)) << 5) | ((KST_Fahrerhinweis_5 & (0x01U)) << 6) | ((KST_Fahrerhinweis_6 & (0x01U)) << 7);


  HVK_01[1] = (0x00 & (0x0FU)) | ((0x00 & (0x01U)) << 4) | ((0x00 & (0x03U)) << 5);
  HVK_01[2] = (HVK_MO_EmSollzustand & (0xFFU));
  //HVK_01[3] = (HVK_BMS_Sollmodus & (0x07U)) | ((HVK_DCDC_Sollmodus & (0x07U)) << 3) | ((HVK_EKK_Sollmodus & (0x03U)) << 6);
  HVK_01[3] = (HVK_BMS_Sollmodus & (0x07U)) | ((HVK_DCDC_Sollmodus & (0x07U)) << 3) | ((0x00 & (0x03U)) << 6);
  //HVK_01[4] = ((HVK_EKK_Sollmodus >> 2) & (0x01U)) | ((HVK_HVPTC_Sollmodus & (0x07U)) << 1) | ((HVK_HVLM_Sollmodus & (0x07U)) << 4) | ((HVK_HV_Netz_Warnungen & (0x01U)) << 7);
  HVK_01[4] = ((0x00 >> 2) & (0x01U)) | ((0x00 & (0x07U)) << 1) | ((HVK_HVLM_Sollmodus & (0x07U)) << 4) | ((0x00 & (0x01U)) << 7);
  //HVK_01[5] = ((HVK_HV_Netz_Warnungen >> 1) & (0x01U)) | ((HV_Bordnetz_aktiv & (0x01U)) << 1) | ((HV_Bordnetz_Fehler & (0x01U)) << 2) | ((HVK_Gesamtst_Spgfreiheit & (0x03U)) << 3) | ((HVK_AktiveEntladung_Anf & (0x01U)) << 5);
  HVK_01[5] = ((0x00 >> 1) & (0x01U)) | ((HV_Bordnetz_aktiv & (0x01U)) << 1) | ((0x00 & (0x01U)) << 2) | ((HVK_Gesamtst_Spgfreiheit & (0x03U)) << 3) | ((0x00 & (0x01U)) << 5);
  //HVK_01[6] = ((HVK_Iso_Messung_Start & (0x07U)) << 2);
  //HVK_01[7] = ((HVK_DCDC_EKK_Sollmodus & (0x03U)) << 6);

  // ZV_01[1] = (ZV_01_BZ & (0x0FU)) | ((ZV_FT_verriegeln & (0x01U)) << 4) | ((ZV_FT_entriegeln & (0x01U)) << 5) | ((ZV_BT_verriegeln & (0x01U)) << 6) | ((ZV_BT_entriegeln & (0x01U)) << 7);
  ZV_01[1] = (0x00 & (0x0FU)) | ((ZV_FT_verriegeln & (0x01U)) << 4) | ((ZV_FT_entriegeln & (0x01U)) << 5) | ((ZV_BT_verriegeln & (0x01U)) << 6) | ((ZV_BT_entriegeln & (0x01U)) << 7);
  // ZV_01[2] = (ZV_HFS_verriegeln & (0x01U)) | ((ZV_HFS_entriegeln & (0x01U)) << 1) | ((ZV_HBFS_verriegeln & (0x01U)) << 2) | ((ZV_HBFS_entriegeln & (0x01U)) << 3) | ((ZV_zentral_safen & (0x01U)) << 4) | ((ZV_zentral_entsafen & (0x01U)) << 5) | ((ZV_Spg_Anklappen & (0x01U)) << 6) | ((ZV_Softtouch_betaetigt & (0x01U)) << 7);
  // ZV_01[3] = (ZV_LED_Steuerung & (0x01U)) | ((ZV_LED_Uebernahme & (0x01U)) << 1) | ((ZV_auf_FT & (0x01U)) << 2) | ((ZV_zu_FT & (0x01U)) << 3) | ((ZV_auf_BT & (0x01U)) << 4) | ((ZV_zu_BT & (0x01U)) << 5) | ((ZV_auf_Kessy & (0x01U)) << 6) | ((ZV_zu_Kessy & (0x01U)) << 7);
  // ZV_01[4] = (ZV_auf_Funk & (0x01U)) | ((ZV_zu_Funk & (0x01U)) << 1) | ((VIP_Sensor_betaetigt & (0x01U)) << 2) | ((VIP_Freigabe & (0x01U)) << 3) | ((ZV_zu_Zeitl_Nachverr & (0x01U)) << 4) | ((ZV_HSK_entriegeln & (0x01U)) << 5) | ((ZV_HSK_verriegeln & (0x01U)) << 6) | ((ZV_Verdeck_zu & (0x01U)) << 7);
  // ZV_01[5] = (ZV_Verdeck_auf & (0x01U)) | ((FH_FT_hoch & (0x01U)) << 1) | ((FH_FT_tief & (0x01U)) << 2) | ((FH_BT_hoch & (0x01U)) << 3) | ((FH_BT_tief & (0x01U)) << 4) | ((FH_HFS_hoch & (0x01U)) << 5) | ((FH_HFS_tief & (0x01U)) << 6) | ((FH_HBFS_hoch & (0x01U)) << 7);
  // ZV_01[6] = (FH_HBFS_tief & (0x01U)) | ((BCM_Spg_Synchron & (0x01U)) << 1) | ((BCM_BF_Spg_Absenkung & (0x01U)) << 2) | ((ZV_Signatur & (0x1FU)) << 3);
  // ZV_01[7] = ((ZV_Signatur >> 5) & (0x3FU)) | ((ZV_entriegeln_Anf & (0x01U)) << 6) | ((ZV_auto_Ansteuerung & (0x01U)) << 7);
  ZV_01[7] = ((0x00 >> 5) & (0x3FU)) | ((ZV_entriegeln_Anf & (0x01U)) << 6) | ((0x00 & (0x01U)) << 7);

  // ZV_02[1] = ((BCM_FH_Freigabe & (0x01U)) << 4) | ((BCM_Komfortfkt_Freigabe & (0x01U)) << 5) | ((BCM_HSK_Freigabe & (0x01U)) << 6) | ((BCM_Verdeck_Freigabe & (0x01U)) << 7);
  // ZV_02[2] = (ZV_verriegelt_intern_ist & (0x01U)) | ((ZV_verriegelt_extern_ist & (0x01U)) << 1) | ((ZV_verriegelt_intern_soll & (0x01U)) << 2) | ((ZV_verriegelt_extern_soll & (0x01U)) << 3) | ((ZV_gesafet_extern_ist & (0x01U)) << 4) | ((ZV_gesafet_extern_soll & (0x01U)) << 5) | ((ZV_Einzeltuerentriegelung & (0x01U)) << 6) | ((ZV_Heckeinzelentriegelung & (0x01U)) << 7);
  ZV_02[2] = (ZV_verriegelt_intern_ist & (0x01U)) | ((ZV_verriegelt_extern_ist & (0x01U)) << 1) | ((ZV_verriegelt_intern_soll & (0x01U)) << 2) | ((ZV_verriegelt_extern_soll & (0x01U)) << 3) | ((0x00 & (0x01U)) << 4) | ((0x00 & (0x01U)) << 5) | ((0x00 & (0x01U)) << 6) | ((0x00 & (0x01U)) << 7);
  // ZV_02[3] = (ZV_FT_offen & (0x01U)) | ((ZV_BT_offen & (0x01U)) << 1) | ((ZV_HFS_offen & (0x01U)) << 2) | ((ZV_HBFS_offen & (0x01U)) << 3) | ((ZV_HD_offen & (0x01U)) << 4) | ((ZV_HS_offen & (0x01U)) << 5) | ((IRUE_aktiv & (0x01U)) << 6) | ((DWA_aktiv & (0x01U)) << 7);
  // ZV_02[4] = (HD_Hauptraste & (0x01U)) | ((HD_Vorraste & (0x01U)) << 1) | ((FFB_CarFinder & (0x01U)) << 6) | ((FFB_Komfortoeffnen & (0x01U)) << 7);
  // ZV_02[5] = (FFB_Komfortschliessen & (0x01U)) | ((ZV_Schluessel_Zugang & (0x0FU)) << 2) | ((ZV_SafeFunktion_aktiv & (0x01U)) << 6) | ((FBS_Warn_Schluessel_Batt & (0x01U)) << 7);
  // ZV_02[6] = (ZV_Oeffnungsmodus & (0x03U)) | ((HFS_verriegelt & (0x01U)) << 2) | ((HFS_gesafet & (0x01U)) << 3) | ((HBFS_verriegelt & (0x01U)) << 4) | ((HBFS_gesafet & (0x01U)) << 5) | ((ZV_ist_Zustand_verfuegbar & (0x01U)) << 6) | ((IRUE_Taster_Fkts_LED & (0x01U)) << 7);
  // ZV_02[7] = (ZV_Tankklappe_offen & (0x01U)) | ((ZV_Rollo_auf & (0x01U)) << 1) | ((ZV_Rollo_zu & (0x01U)) << 2) | ((ZV_SAD_auf & (0x01U)) << 3) | ((ZV_SAD_zu & (0x01U)) << 4) | ((BCM_Tankklappensteller_Fehler & (0x01U)) << 5) | ((ZV_verriegelt_soll & (0x03U)) << 6);

  // FCU_02[1] = ((FCU_Warn_H2_Konzentrat_Motorraum & (0x07U)) << 4) | ((FCU_nicht_verfuegbar & (0x01U)) << 7);
  // FCU_02[2] = (FCU_Startstopp_Anforderung & (0x0FU)) | ((FCU_HV_Anf & (0x03U)) << 4) | ((FCU_Klima_Eingr & (0x03U)) << 6);
  // FCU_02[3] = (FCU_IstModus & (0x1FU)) | ((FCU_Status_Spgfreiheit & (0x03U)) << 5) | ((FCU_Fehler_Leistungsreduzierung & (0x01U)) << 7);
  // FCU_02[4] = (FCU_Fehler_Abschaltanforderung & (0x01U)) | ((FCU_Fehler_Notabschaltung & (0x01U)) << 1) | ((FCU_Fehler_Entladung_defekt & (0x03U)) << 2) | ((FCU_Fehler_Isofehler_I & (0x03U)) << 4) | ((FCU_Fehler_Isofehler_II & (0x03U)) << 6);
  // FCU_02[5] = (FCU_OBD_Lampe_Anf & (0x01U)) | ((FCU_Absperrventil_schliessen & (0x03U)) << 1) | ((FCU_TK_Betankung_Anforderung & (0x01U)) << 3) | ((FCU_MinAbs_Leistung_ro & (0x0FU)) << 4);
  FCU_02[5] = (0x00 & (0x01U)) | ((0x00 & (0x03U)) << 1) | ((FCU_TK_Betankung_Anforderung & (0x01U)) << 3) | ((0x00 & (0x0FU)) << 4);
  // FCU_02[6] = ((FCU_MinAbs_Leistung_ro >> 4) & (0x3FU)) | ((FCU_Ionenspuelung_aktiv & (0x03U)) << 6);
  FCU_02[7] = (FCU_TK_Freigabe_Tankklappe & (0x03U));

  EM_HYB_11[1] = (0x00 & (0x0FU)) | ((EM1_Istmodus2 & (0x0FU)) << 4);
  EM_HYB_11[2] = (0x00 & (0x07U)) | ((EM1_Status_Spgfreiheit & (0x03U)) << 3) | ((0x00 & (0x01U)) << 5);

}



