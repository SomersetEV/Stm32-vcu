#ifndef CayenneCharger_H
#define CayenneCharger_H

#include <stdint.h>
#include "my_fp.h"
#include "my_math.h"
#include "hwinit.h"
#include "params.h"
#include "chargerhw.h"
#include <libopencm3/stm32/timer.h>

class CayenneCharger: public Chargerhw
{

public:
bool ControlCharge(bool RunCh, bool ACReq);
bool parked = false;
bool locked = false;
void DecodeCAN(int id, uint32_t data[2]);
void SetCanInterface(CanHardware* c);

private:
int opmode;
void Task10Ms();
void Task100Ms();
void Task200Ms();
void CalcValues100ms();
int stopcharge = 0;
static void handle377(uint32_t data[2]);
static void handle389(uint32_t data[2]);
static void handle38A(uint32_t data[2]);
void handle488(uint32_t data[2]); // 1
void handle53C(uint32_t data[2]); // 1
void handle564(uint32_t data[2]); //1
void handle565(uint32_t data[2]); //1
void handle67E(uint32_t data[2]); //1
void handle415(uint32_t data[2]); //1
void handle1B000044(uint32_t data[2]); //1
void handle12DD5472(uint32_t data[2]); //1
void msg3C0(); // 1
void msg1A1();      // BMS_02     0x1A1  1
void msg64F();      // BCM1_04    0x64F  1
void msg663();      // NVEM_02    0x663  11
void msg191();      // BMS_01     0x191  1
void msg503();      // HVK_01     0x503  1
void msg39D();      // BMS_03     0x39D
void msg415();      // stop charge message
void msg184();      // 
void msg17B();      // 
void msg583(); // ZV_02
void msg552(); //  HVEM_05
void UnLockCP(); // 1
void LockCP(); // 1
static void canRX_488(); // HVLM_06
static void canRX_53C(); // HVLM_04
static void canRX_564(); // LAD_01
static void canRX_565(); // HVLM_03
static void canRX_67E(); // LAD_02
static void canRX_12DD5472(); // HVLM_10
static void canRX_12DD5491(); // HVLM_11
static void canRX_1A55549D(); // HVLM_08
static void canRX_1A55554D(); // HVLM_15
static void canRX_1A55554F(); // LAD_06

uint8_t vag_cnt3C0 = 0x00;
uint8_t vag_cnt040 = 0x00;
uint8_t vag_cnt184 = 0x00;
uint8_t vag_cnt191 = 0x00;
uint8_t vag_cnt1A2 = 0x00;
uint8_t vag_cnt37C = 0x00;
uint8_t vag_cnt2AE = 0x00;
uint8_t vag_cnt503 = 0x00;
uint8_t vag_cnt578 = 0x00;
uint8_t vag_cnt5A2 = 0x00;
uint8_t vag_cnt5CA = 0x00;
uint8_t vag_cnt5CD = 0x00;

uint16_t HVDCSetpnt;
  uint16_t IDCSetpnt;
  uint8_t modeSet;
  bool active;

uint16_t setVolts , actVolts , termAmps;
int16_t actAmps;
uint8_t Airbag_01[8]      = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t Authentic_Time_01[8]  = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BCM1_04[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_01[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_02[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_03[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_04[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_06[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_07[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_09[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_10[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_11[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_16[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_27[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t BMS_DC_01[8]      = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t EM_HYB_11[8]      = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t ESP_15[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t Dimmung_01[8]     = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t DCDC_01[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t DCDC_02[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t DCDC_03[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t FCU_02[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVK_01[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVEM_02[8]        = {0xFF, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVEM_05[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t Klemmen_Status_01[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t MSG_TME_02[8]     = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t NVEM_02[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t ORU_01[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t ZV_02[8]          = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVLM_06[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVLM_04[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t LAD_01[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVLM_03[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t LAD_02[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVLM_10[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVLM_11[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVLM_08[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t HVLM_15[8]        = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t LAD_06[8]         = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t ZV_01[8]          = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
uint16_t ACvoltage ;
   uint32_t HVLM_MaxDC_ChargePower; // maximum DC charging power
  uint16_t HVLM_Max_DC_Voltage_DCLS; // maximum DC charging voltage
  uint16_t HVLM_Actual_DC_Current_DCLS; // actual DC charging current
  uint16_t HVLM_Max_DC_Current_DCLS; // maximum DC charging current
  uint16_t HVLM_Min_DC_Voltage_DCLS; // minimum DC charging voltage
  uint16_t HVLM_Min_DC_Current_DCLS; // minimum DC charging current
  uint8_t HVLM_Status_Grid; // Information as to whether the vehicle is connected to a power grid.
  uint8_t HVLM_EnergyFlowType; // Display of whether electricity flows into the vehicle and what it is used for.
  uint8_t HVLM_OperationalMode; // Current Mode: 0=Inactive, 1=Active, 2=Init, 3=Error
  uint8_t HVLM_HV_ActivationRequest; // HV activation request and reason for the request: 0=No Request, 1=Charging, 2=Battery Balancing, 3=AC/Climate
  uint8_t HVLM_ChargerErrorStatus; // Current error status of the charger: 0=No Error, 1=DC-NotOK, 2=AC-NotOK, 3=Interlock, 4=Reserved, 5=Reserved, 6=No Component Function, 7=Init
  uint8_t HVLM_Park_Request; // Request to lock the drive train
  uint8_t HVLM_Park_Request_Maintain; // Request to keep the drive train locked
  uint8_t HVLM_Plug_Status; // Plug detection status independent of charging mode (AC or DC): 0=Init, 1=No Plug Inserted, 2=Connector Inserted, Not Locked, 3=Connector Inserted & Locked
  uint8_t HVLM_LoadRequest; // Charger Status: 0=No Request, 1=AC Charge, 2=DC Charge, 3=Recharge 12V, 4=AC AWC Charge, 5=Reserved, 6=Init, 7=Error
  uint8_t HVLM_MaxBattChargeCurrent; // Recommended HV battery charging current for a planned charge
  uint8_t LAD_Mode; // Operating mode of the charger
  uint16_t LAD_AC_Volt_RMS; // Actual value AC grid voltage (RMS)
  uint16_t LAD_VoltageOut_HV; // Output voltage of the charger
  uint16_t LAD_CurrentOut_HV; // Output current charger
  uint8_t LAD_Status_Voltage; //
  uint16_t LAD_Temperature; // Instantaneous value: Charger temperature
  uint16_t LAD_PowerLossVal; // Instantaneous value: power loss charger
  uint16_t HVLM_HV_StaleTime; // Period between HV deactivated and HV activated
  uint8_t HVLM_ChargeSystemState; // Displaying status about the charging system. 0=No issue, 1=System Defective, 2=System Incompatable 3=DC Charge not possible
  uint8_t HVLM_Status_LED; // Status of the charging LED: 0=Colour1-off, 1=Colour2-White, 2=Colour3-Yellow, 3=Colour4-Green, 4=Colour5-Red, 5=Yellow Pulsing, 6=Green Pulsing, 7=Red Pulsing, 8=Green/Red Pulsing, 9=Green Flashing, 14=Init, 15=Error
  uint8_t HVLM_MaxCurrent_AC; // Maximum permissible current on the primary side (AC)
  bool HVLM_LG_ChargerTargetMode; // AC charger target mode: 0=Standby, 1=Mains Charging
  uint8_t HVLM_TankCapReleaseRequest; // Fuel Cap Release: 0=No Release, 1=Release, 2=Init, 3=Error
  uint8_t HVLM_RequestConnectorLock; // Request connector lock: 0=Unlock, 1=lock, 2=Init, 3=No Request
  uint8_t HVLM_Start_VoltageMeasure_DCLS; //Message to the BMS that a measurement voltage is being applied to the DC charging station: 0=Inactive, 1=DCLS With Diode, 2=DCLS without Diode, 3=Reserve
  uint8_t HVLM_ChargeReadyStatus; // Display whether AC or DC charging is not possible: 0=No Error, 1=AC Charge Not Possible, 2=DC Charge Not Possible, 3=AC & DC Charge Not Possible
  uint16_t HVLM_Output_Voltage_HV; // Output voltage of the charger and DC voltage of the charging station. Measurement between the DC HV lines.
  bool LAD_Reduction_ChargerTemp; // Reduction due to internal overtemperature in the charger
  bool LAD_Reduction_Current; // Regulation due to current or voltage at the input or output
  bool LAD_Reduction_SocketTemp; // Reduction due to excessive charging socket temperature
  uint16_t LAD_MaxChargerPower_HV; // Maximum power charger in relation to the maximum power infrastructure (cable, charging station) and taking into account the charger efficiency
  uint8_t LAD_PRX_CableCurrentLimit; // AC current limit due to PRX cable coding
  bool LAD_ControlPilotStatus; // Status control pilot monitoring (detection of the control pilot duty cycle)
  bool LAD_LockFeedback; // Status of connector lock (feedback contact of the locking actuator)
  uint8_t LAD_ChargerCoolingDemand; // Cooling demand of the charger
  bool LAD_ChargerWarning; // Collective warning charger: 0=Normal, 1=Warning
  bool LAD_ChargerFault; // Collective error charger: 0=Normal, 1=No Charging Possible
  uint8_t HVLM_RtmWarnLadeverbindung; // RTM Charging connection fault
  uint8_t HVLM_RtmWarnLadesystem; // RTM Electrical machine CAN communication fault
  uint8_t HVLM_RtmWarnLadestatus; // RTM Warning of charging status fault
  uint8_t HVLM_RtmWarnLadeKommunikation; // RTM Warning of charging communction fault
bool UnLock;
uint32_t UnixTime;
uint16_t BMS_Batt_Curr;
uint16_t BMS_Batt_Volt;
uint16_t BMS_Batt_Volt_HVterm;
uint16_t BMS_SOC_HiRes;
uint16_t BMS_MaxDischarge_Curr;
uint16_t BMS_Min_Batt_Volt;
uint16_t BMS_Min_Batt_Volt_Discharge;
uint16_t BMS_MaxCharge_Curr;
uint16_t BMS_MaxCharge_Curr_Offset;
uint16_t BMS_Batt_Max_Volt;
uint16_t BMS_Min_Batt_Volt_Charge;
uint16_t BMS_OpenCircuit_Volts;
bool BMS_Status_ServiceDisconnect;
uint8_t BMS_HV_Status;
bool BMS_Faultstatus;
int BMS_IstModus;
int carwakeup;
uint16_t BMS_Batt_Ah;
uint16_t BMS_Target_SOC_HiRes;

uint16_t BMS_Batt_Temp;
uint16_t BMS_CurrBatt_Temp;
uint16_t BMS_CoolantTemp_Act;
uint16_t BMS_Batt_Energy;
uint16_t BMS_Max_Wh;
uint16_t BMS_BattEnergy_Wh_HiRes;
uint16_t BMS_MaxBattEnergy_Wh_HiRes;
uint16_t BMS_SOC;
uint16_t SOCx10 = 351;
uint16_t BMS_ResidualEnergy_Wh;

uint16_t BMS_SOC_ChargeLim;
uint16_t BMS_EnergyCount;
uint16_t BMS_EnergyReq_Full;
uint16_t BMS_ChargePowerMax;
uint16_t BMS_ChargeEnergyCount;

uint16_t BMS_IsoTest;
uint16_t BMS_BattCell_Temp_Max;
uint16_t BMS_BattCell_Temp_Min;
uint16_t BMS_BattCell_MV_Max;
uint16_t BMS_BattCell_MV_Min;
bool HVEM_Nachladen_Anf;
uint16_t HVEM_SollStrom_HV;
uint16_t HVEM_MaxSpannung_HV;
uint8_t HMS_Systemstatus;
uint8_t HMS_aktives_System;
bool HMS_Fehlerstatus;
uint8_t HVK_HVLM_Sollmodus; // Requested target mode of the charging manager: 0=Not Enabled, 1=Enabled
bool HV_Bordnetz_aktiv; // Indicates an active high-voltage vehicle electrical system: 0 = Not Active,  1 = Active
uint8_t HVK_MO_EmSollzustand; // 0 "HvOff" 1 "HvStbyReq" 2 "HvStbyWait" 3 "HvBattOnReq" 4 "HvBattOnWait" 10 "HvOnIdle" 20 "HvOnDrvRdy" 46 "HvAcChPreReq" 47 "HvAcChPreWait" 48 "HvAcChReq" 49 "HvAcChWait" 50 "HvAcCh" 56 "HvDcChPreReq" 57 "HvDcChPreWait" 58 "HvDcChReq" 59 "HvDcChWait" 60 "HvDcCh" 67 "HvChOffReq" 68 "HvChOffWait" 69 "HvOnIdleReq" 70 "HvOnIdleWait" 96 "HvCpntOffReq" 97 "HvCpntOffWait" 98 "HvBattOffReq" 99 "HvBattOffWait" 119 "HvElmOffReq" 120 "HvElmOff"
uint8_t HVK_BMS_Sollmodus; // 0 "HV_Off" 1 "HV_On" 3 "AC_Charging_ext" 4 "AC_Charging" 6 "DC_Charging" 7 "Init" ;
uint8_t HVK_DCDC_Sollmodus; //0 "Standby" 1 "HV_On_Precharging" 2 "Step down" 3 "Step up" 4 "Test pulse_12V" 7 "Initialization" ;
bool ZV_FT_verriegeln;
bool ZV_FT_entriegeln;
bool ZV_BT_verriegeln;
bool ZV_BT_entriegeln;
bool ZV_entriegeln_Anf;
bool ZV_verriegelt_intern_ist;
bool ZV_verriegelt_extern_ist;
bool ZV_verriegelt_intern_soll;
bool ZV_verriegelt_extern_soll;
bool FCU_TK_Betankung_Anforderung;
uint8_t FCU_TK_Freigabe_Tankklappe;
bool BMS_Charger_Active;
uint16_t BMS_RIso_Ext = 4090;
uint8_t HVK_Gesamtst_Spgfreiheit;
uint8_t BMS_Balancing_Active = 2;
uint8_t BMS_Freig_max_Perf = 1;
uint8_t BMS_Battdiag = 1; // Battery Display Diagnostics: 1 = Display Battery, 4 = Display Battery OK, 5 = Charging, 6 = Check Battery
uint8_t DC_IstModus_02 = 2;

uint8_t BMS_HV_Auszeit_Status = 1; // Status HV timeout.
uint16_t BMS_HV_Auszeit = 25; // Time since last HV Activity
uint16_t BMS_Kapazitaet = 1000; //  Total Energy Capacity (aged)
uint16_t BMS_SOC_Kaltstart = 0; // SOC Cold
uint8_t BMS_max_Grenz_SOC = 30; // Upper limit of SOC operating strategy (70 offset, so 30 = 100)
uint8_t BMS_min_Grenz_SOC = 15; // Lower limit of SOC Operating strategy

uint8_t EM1_Istmodus2; // EM1 Status, 0=standby
uint8_t EM1_Status_Spgfreiheit; // Voltage Status: 0=Init, 1=NoVoltage, 2=Voltage, 3=Fault & Voltage

bool ZAS_Kl_S; // KeySwitch Inserted
bool ZAS_Kl_15; // Acc position
bool ZAS_Kl_X; // Run position
bool ZAS_Kl_50_Startanforderung; // Start

uint8_t chargeractive;
uint8_t Counter_Statemachine;
uint8_t ChargeActiveDelayCnt;


  uint16_t HVVoltage ;
  uint16_t MaxHV ;
  int8_t temperature ;
  uint8_t mode;
  uint16_t current;
  uint16_t targetcurrent;
  uint8_t MaxACAmps;
  uint8_t ISetPnt;
  uint8_t PPLim;
uint8_t currentRamp;
bool clearToStart=false , shutDownReq=false, pwmON=false;
static uint8_t chgStatus , evseDuty;
static float dcBusV , temp_1 , temp_2 , ACVolts , DCAmps , ACAmps;
static float LV_Volts , LV_Amps;



uint8_t vw_crc_calc(uint8_t* inputBytes, uint8_t length, uint16_t address)
{
  const uint8_t poly = 0x2F;
  const uint8_t xor_output = 0xFF;
  // VAG Magic Bytes
  const uint8_t MB0040[16] = { 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40, 0x40 };
  const uint8_t MB0073[16] = { 0xdb, 0x71, 0x31, 0xf9, 0x26, 0x43, 0x6d, 0x22, 0x52, 0xe4, 0xe8, 0xa9, 0x73, 0x1c, 0xc5, 0x67 };
  const uint8_t MB0074[16] = { 0xb7, 0x6e, 0x93, 0xf5, 0xdc, 0x38, 0xe2, 0x40, 0x06, 0xfb, 0x80, 0x1c, 0x0e, 0xe3, 0xd5, 0xe5 };
  const uint8_t MB0075[16] = { 0x63, 0x04, 0xb4, 0x32, 0x56, 0xc1, 0xfa, 0x97, 0x1f, 0xa6, 0xae, 0xd7, 0x64, 0xc3, 0xbe, 0x42 };
  const uint8_t MB0080[16] = { 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80 };
  const uint8_t MB0081[16] = { 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81 };
  const uint8_t MB0082[16] = { 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82, 0x82 };
  const uint8_t MB0083[16] = { 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83, 0x83 };
  const uint8_t MB0087[16] = { 0x14, 0x40, 0x8b, 0x7a, 0x78, 0x23, 0xc6, 0x13, 0x1c, 0x03, 0xed, 0xf4, 0xcb, 0xf8, 0xf0, 0xe5 };
  const uint8_t MB0097[16] = { 0x3C, 0x54, 0xCF, 0xA3, 0x81, 0x93, 0x0B, 0xC7, 0x3E, 0xDF, 0x1C, 0xB0, 0xA7, 0x25, 0xD3, 0xD8 };
  const uint8_t MB009D[16] = { 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D, 0x9D };
  const uint8_t MB00A0[16] = { 0x4a, 0xb4, 0xe8, 0x24, 0x99, 0x5a, 0x59, 0x8b, 0x1e, 0xaa, 0x94, 0xb6, 0x09, 0xd5, 0xaf, 0x97 };
  const uint8_t MB00A8[16] = { 0x52, 0x8c, 0x50, 0xee, 0x4f, 0xa6, 0xcc, 0xcf, 0x7d, 0x2f, 0x98, 0x6b, 0x27, 0x41, 0x9f, 0x93 };
  const uint8_t MB00AA[16] = { 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA };
  const uint8_t MB00AB[16] = { 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB, 0xAB };
  const uint8_t MB00B1[16] = { 0xe1, 0x9e, 0xc2, 0xe9, 0x48, 0x98, 0xf5, 0xb6, 0x47, 0x83, 0x26, 0x45, 0x9a, 0x10, 0x7b, 0x1c };
  const uint8_t MB00B3[16] = { 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41, 0x41 };
  const uint8_t MB0102[16] = { 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03, 0x03 };
  const uint8_t MB0105[16] = { 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 };
  const uint8_t MB0108[16] = { 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09 };
  const uint8_t MB010E[16] = { 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F };
  const uint8_t MB0111[16] = { 0xb3, 0xeb, 0xe1, 0xf7, 0xba, 0x9e, 0x5f, 0xd9, 0xc2, 0xec, 0xf6, 0xe9, 0x24, 0x18, 0x48, 0xed };
  const uint8_t MB0114[16] = { 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15, 0x15 };
  const uint8_t MB0116[16] = { 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac, 0xac };
  const uint8_t MB0121[16] = { 0xe9, 0x65, 0xae, 0x6b, 0x7b, 0x35, 0xe5, 0x5f, 0x4e, 0xc7, 0x86, 0xa2, 0xbb, 0xdd, 0xeb, 0xb4 };
  const uint8_t MB0124[16] = { 0x12, 0x7E, 0x34, 0x16, 0x25, 0x8F, 0x8E, 0x35, 0xBA, 0x7F, 0xEA, 0x59, 0x4C, 0xF0, 0x88, 0x15 };
  const uint8_t MB0136[16] = { 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37 };
  const uint8_t MB0153[16] = { 0x03, 0x13, 0x23, 0x7a, 0x40, 0x51, 0x68, 0xba, 0xa8, 0xbe, 0x55, 0x02, 0x11, 0x31, 0x76, 0xec };
  const uint8_t MB0154[16] = { 0x21, 0x93, 0x4a, 0x04, 0x5c, 0x18, 0xe0, 0x69, 0x14, 0xce, 0x26, 0x8f, 0x5e, 0xb1, 0xdf, 0x53 };
  const uint8_t MB0184[16] = { 0xe3, 0xe8, 0xc4, 0x7f, 0xf1, 0x95, 0xd8, 0xb0, 0xc7, 0xa3, 0xf4, 0x6c, 0xe7, 0xc3, 0x5a, 0xf7 };
  const uint8_t MB0187[16] = { 0x7F, 0xED, 0x17, 0xC2, 0x7C, 0xEB, 0x44, 0x21, 0x01, 0xFA, 0xDB, 0x15, 0x4A, 0x6B, 0x23, 0x05 };
  const uint8_t MB018D[16] = { 0x9c, 0xcc, 0xe1, 0x4c, 0xc9, 0x39, 0xe3, 0x53, 0x78, 0x33, 0x15, 0x49, 0x55, 0x0a, 0x62, 0x83 };
  const uint8_t MB0191[16] = { 0x7d, 0x0e, 0xaf, 0x0b, 0x24, 0x58, 0x88, 0x9e, 0x89, 0x3a, 0x81, 0x61, 0x4e, 0xe3, 0x04, 0x1f };
  const uint8_t MB01A2[16] = { 0xcb, 0x47, 0xd7, 0xd2, 0x56, 0x3e, 0x20, 0xea, 0x6e, 0x6d, 0x8c, 0x1b, 0xdf, 0x49, 0xaa, 0xab };
  const uint8_t MB01A3[16] = { 0x4a, 0x06, 0x0a, 0x36, 0xf8, 0xd3, 0x60, 0x79, 0xaa, 0xd2, 0x2e, 0x89, 0xa3, 0x76, 0x5f, 0x45 };
  const uint8_t MB01F3[16] = { 0xc4, 0x1e, 0x9c, 0xae, 0x59, 0xcc, 0x66, 0x99, 0xe1, 0x87, 0xe8, 0x4c, 0x65, 0x4a, 0xc9, 0xde };
  const uint8_t MB01F9[16] = { 0x14, 0xf8, 0xf6, 0x94, 0x2a, 0x12, 0x39, 0x3d, 0x98, 0x3f, 0xfb, 0x9d, 0x36, 0xca, 0x1d, 0xe0 };
  const uint8_t MB020F[16] = { 0xd4, 0x22, 0xad, 0x3f, 0x25, 0xaa, 0x62, 0x5b, 0xdc, 0x73, 0xed, 0xc3, 0x9a, 0x14, 0x2f, 0x3e };
  const uint8_t MB02AD[16] = { 0x7b, 0xed, 0xa6, 0xd1, 0x57, 0x4b, 0x69, 0x9f, 0x13, 0x82, 0x9e, 0x19, 0x3e, 0x08, 0x5e, 0x78 };
  const uint8_t MB02AE[16] = { 0x83, 0x50, 0xbf, 0x5d, 0x67, 0x68, 0x2f, 0x4b, 0x32, 0x4e, 0x1a, 0xba, 0xc4, 0x39, 0xf5, 0xdb };
  const uint8_t MB030E[16] = { 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D, 0x0D };
  const uint8_t MB0312[16] = { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x11 };
  const uint8_t MB031B[16] = { 0x67, 0x8a, 0xae, 0x22, 0x4d, 0xd0, 0x51, 0x80, 0x5c, 0xb9, 0xce, 0x1e, 0xdf, 0x02, 0x2d, 0xd4 };
  const uint8_t MB0365[16] = { 0xe3, 0x0a, 0x29, 0xd3, 0x96, 0xaa, 0xeb, 0x89, 0xe6, 0x5f, 0x14, 0xc4, 0xe9, 0x27, 0x0b, 0x2d };
  const uint8_t MB0367[16] = { 0xa1, 0xb0, 0xdb, 0x69, 0x9c, 0xef, 0xbd, 0x08, 0xdd, 0xcc, 0x1b, 0x05, 0xce, 0x20, 0xe1, 0x48 };
  const uint8_t MB037C[16] = { 0xed, 0x03, 0x1c, 0x13, 0xc6, 0x23, 0x78, 0x7a, 0x8b, 0x40, 0x14, 0x51, 0xbf, 0x68, 0x32, 0xba };
  const uint8_t MB0393[16] = { 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90, 0x90 };
  const uint8_t MB0394[16] = { 0x47, 0x94, 0x92, 0x6a, 0x67, 0xb5, 0x0d, 0x38, 0xe3, 0x8a, 0x5d, 0xb4, 0x54, 0xab, 0xae, 0x27 };
  const uint8_t MB03A6[16] = { 0xB6, 0x1C, 0xC1, 0x23, 0x6D, 0x8B, 0x0C, 0x51, 0x38, 0x32, 0x24, 0xA8, 0x3F, 0x3A, 0xA4, 0x02 };
  const uint8_t MB03AF[16] = { 0x94, 0x6A, 0xB5, 0x38, 0x8A, 0xB4, 0xAB, 0x27, 0xCB, 0x22, 0x88, 0xEF, 0xA3, 0xE1, 0xD0, 0xBB };
  const uint8_t MB03BE[16] = { 0x1f, 0x28, 0xc6, 0x85, 0xe6, 0xf8, 0xb0, 0x19, 0x5b, 0x64, 0x35, 0x21, 0xe4, 0xf7, 0x9c, 0x24 };
  const uint8_t MB03C0[16] = { 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3, 0xc3 };
  const uint8_t MB03F4[16] = { 0x26, 0xe4, 0xc5, 0x16, 0xcf, 0x20, 0x65, 0xad, 0xed, 0x6a, 0xa3, 0x49, 0x02, 0x6b, 0xb2, 0xbd };
  const uint8_t MB03FA[16] = { 0x26, 0xe3, 0x43, 0x0a, 0x6d, 0x29, 0x22, 0xd3, 0x52, 0x96, 0xe4, 0xaa, 0xe8, 0xeb, 0xa9, 0x89 };
  const uint8_t MB0450[16] = { 0x30, 0x25, 0xc2, 0x1f, 0x02, 0xb1, 0x9e, 0xe0, 0x73, 0x51, 0xe1, 0xa5, 0xab, 0x0d, 0xf4, 0x2f };
  const uint8_t MB0451[16] = { 0x19, 0xc1, 0x1a, 0xf6, 0x7d, 0x28, 0x7b, 0x12, 0xcb, 0x29, 0x59, 0x98, 0xd2, 0xbf, 0x53, 0x9d };
  const uint8_t MB0452[16] = { 0x74, 0x4a, 0x84, 0x18, 0xb9, 0x14, 0x9e, 0x8f, 0x0c, 0xdf, 0x29, 0x40, 0x63, 0x1f, 0xd4, 0xca };
  const uint8_t MB0457[16] = { 0x82, 0x79, 0xee, 0xb9, 0xf9, 0x67, 0x97, 0xad, 0xb1, 0xd5, 0xcb, 0xf7, 0xb6, 0x70, 0x13, 0xaa };
  const uint8_t MB0489[16] = { 0x7A, 0x13, 0xF4, 0xE5, 0xC9, 0x07, 0x21, 0xDD, 0x6F, 0x94, 0x63, 0x9B, 0xD2, 0x93, 0x42, 0x33 };
  const uint8_t MB0503[16] = { 0xed, 0xd6, 0x96, 0x63, 0xa5, 0x12, 0xd5, 0x9a, 0x1e, 0x0d, 0x24, 0xcd, 0x8c, 0xa6, 0x2f, 0x41 };
  const uint8_t MB0504[16] = { 0x03, 0x63, 0x3e, 0x04, 0xf9, 0xb4, 0x4f, 0x32, 0x25, 0x56, 0x1e, 0xc1, 0x83, 0xfa, 0x31, 0x97 };
  const uint8_t MB053C[16] = { 0x8e, 0xf9, 0xe1, 0x2e, 0x83, 0x34, 0xd0, 0xf1, 0xaa, 0x2c, 0xbb, 0x81, 0x17, 0x60, 0x11, 0x37 };
  const uint8_t MB0560[16] = { 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65, 0x65 };
  const uint8_t MB0564[16] = { 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24, 0x24 };
  const uint8_t MB0578[16] = { 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48, 0x48 };
  const uint8_t MB05A2[16] = { 0xeb, 0x4c, 0x44, 0xaf, 0x21, 0x8d, 0x01, 0x58, 0xfa, 0x93, 0xdb, 0x89, 0x15, 0x10, 0x4a, 0x61 };
  const uint8_t MB05A3[16] = { 0x0a, 0xd3, 0xaa, 0x89, 0x5f, 0xc4, 0x27, 0x2d, 0xfb, 0xa0, 0x47, 0x1e, 0x28, 0xa8, 0x44, 0x5c };
  const uint8_t MB05CA[16] = { 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43, 0x43 };
  const uint8_t MB05CC[16] = { 0x92, 0x9d, 0xb8, 0x71, 0xc4, 0x40, 0x3d, 0x49, 0x5b, 0x4d, 0xfa, 0x94, 0x6d, 0x72, 0xb3, 0x46 };
  const uint8_t MB05CD[16] = { 0x93, 0x04, 0x18, 0x69, 0xce, 0x8f, 0xb1, 0x53, 0x40, 0x2d, 0xe4, 0xca, 0x90, 0x25, 0x79, 0x8c };
  const uint8_t MB0641[16] = { 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47, 0x47 };
  const uint8_t MB065D[16] = { 0xac, 0xb3, 0xab, 0xeb, 0x7a, 0xe1, 0x3b, 0xf7, 0x73, 0xba, 0x7c, 0x9e, 0x06, 0x5f, 0x02, 0xd9 };
  const uint8_t MB067E[16] = { 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25, 0x25 };
  const uint8_t MB06A3[16] = { 0xC1, 0x8B, 0x38, 0xA8, 0xA4, 0x27, 0xEB, 0xC8, 0xEF, 0x05, 0x9A, 0xBB, 0x39, 0xF7, 0x80, 0xA7 };
  const uint8_t MB06A4[16] = { 0xC7, 0xD8, 0xF1, 0xC4, 0xE3, 0x5E, 0x9A, 0xE2, 0xA1, 0xCB, 0x02, 0x4F, 0x57, 0x4E, 0x8E, 0xE4 };
  const uint8_t MB092DD5477[16] = { 0x41, 0x5e, 0x7f, 0xc7, 0xc3, 0x04, 0x0a, 0x11, 0x34, 0x4d, 0x9b, 0x27, 0xaf, 0x0d, 0x0f, 0x31 };
  const uint8_t MB092DD5478[16] = { 0xe0, 0x2f, 0x8d, 0x3c, 0x70, 0x22, 0x03, 0x60, 0x0e, 0x18, 0xd9, 0xeb, 0x95, 0x63, 0xcb, 0x76 };
  const uint8_t MB092DD5490[16] = { 0x48, 0x52, 0xa7, 0xe6, 0x83, 0x2b, 0x8c, 0xfb, 0x7b, 0x64, 0x58, 0x50, 0xbc, 0xb3, 0x0f, 0x9c };
  const uint8_t MB092DD5491[16] = { 0xb1, 0x44, 0x37, 0xca, 0xb5, 0xd6, 0x0e, 0xa3, 0x3b, 0x16, 0xa6, 0x1a, 0x23, 0x1d, 0x91, 0x9c };
  const uint8_t MB092DD5493[16] = { 0x74, 0x7e, 0x45, 0xe8, 0x3c, 0xd0, 0x13, 0x1f, 0x4e, 0x3a, 0x88, 0x0b, 0x7d, 0xd2, 0x6d, 0xab };
  const uint8_t MB092DD54AB[16] = { 0xab, 0xe1, 0x73, 0x9e, 0x02, 0xc2, 0x30, 0xe9, 0x75, 0x48, 0x2e, 0x98, 0x7f, 0xf5, 0xa7, 0xb6 };
  const uint8_t MB092DD54AD[16] = { 0x02, 0xf4, 0x6f, 0x42, 0x4a, 0x62, 0xcd, 0x1b, 0xb4, 0xa0, 0x16, 0x14, 0xe8, 0x0a, 0x71, 0xef };
  const uint8_t MB092DD54E0[16] = { 0xd0, 0x4d, 0x22, 0xae, 0x8a, 0x67, 0xf4, 0x35, 0x15, 0x2a, 0xb8, 0x4e, 0xcc, 0x07, 0xad, 0xa2 };
  const uint8_t MB092DD550B[16] = { 0x67, 0xf7, 0x44, 0xdc, 0xd2, 0xca, 0xe7, 0x2b, 0xd6, 0x5c, 0xc6, 0xa3, 0xbe, 0x0d, 0x16, 0x58 };
  const uint8_t MB096A95414[16] = { 0xa3, 0xed, 0x65, 0xcf, 0xc5, 0x26, 0x54, 0x1a, 0x99, 0x3c, 0x01, 0x84, 0xf4, 0x46, 0x9c, 0x28 };
  const uint8_t MB096A95415[16] = { 0x01, 0x40, 0xf0, 0x21, 0x22, 0x92, 0x44, 0x1a, 0x85, 0xeb, 0x7b, 0xcd, 0x7c, 0x59, 0xc5, 0xc2 };
  const uint8_t MB096A954A6[16] = { 0x79, 0xb9, 0x67, 0xad, 0xd5, 0xf7, 0x70, 0xaa, 0x44, 0x61, 0x5a, 0xdc, 0x26, 0xb4, 0xd2, 0xc3 };
  const uint8_t MB09A555517[16] = { 0xfb, 0xee, 0x5d, 0x2d, 0xfa, 0x85, 0x27, 0xd4, 0xc9, 0xc4, 0xa5, 0x45, 0x5f, 0x76, 0xa3, 0x89 };
  const uint8_t MB09A555545[16] = { 0xc2, 0xe1, 0xd3, 0x20, 0x42, 0xce, 0x68, 0x05, 0xa9, 0x1b, 0xa3, 0xcc, 0xe2, 0xdd, 0x14, 0x08 };

  uint8_t crc = 0xFF;
  uint8_t magicByte = 0x00;
  uint8_t counter = inputBytes[1] & 0x0F; // only the low byte of the couner is relevant

switch (address)
  {
    case 0x03C0: // ??
      magicByte = MB03C0[counter];
      break;
    case 0x0040: // ??
      magicByte = MB0040[counter];
      break;
    case 0x0184: // ??
      magicByte = MB0184[counter];
      break;
    case 0x0191: // ??
      magicByte = MB0191[counter];
      break;
    case 0x01A2: // ??
      magicByte = MB01A2[counter];
      break;
    case 0x02AE: // ??
      magicByte = MB02AE[counter];
      break;
    case 0x037C: // ??
      magicByte = MB037C[counter];
      break;
    case 0x0503: // ??
      magicByte = MB0503[counter];
      break;
    case 0x0578: // ??
      magicByte = MB0578[counter];
      break;
    case 0x05A2: // ??
      magicByte = MB05A2[counter];
      break;
    case 0x05CA: // ??
      magicByte = MB05CA[counter];
      break;
    case 0x05CD: // ??
      magicByte = MB05CD[counter];
      break;
    default: // this won't lead to correct CRC checksums
      magicByte = 0x00;
      break;
  }

  for (uint8_t i = 1; i < length + 1; i++)
  {
    // We skip the empty CRC position and start at the timer
    // The last element is the VAG magic byte for address 0x187 depending on the counter value.
    if (i < length)
      crc ^= inputBytes[i];
    else
      crc ^= magicByte;

    for (uint8_t j = 0; j < 8; j++)
    {
      if (crc & 0x80)
        crc = (crc << 1) ^ poly;
      else
        crc = (crc << 1);
    }
  }

  crc ^= xor_output;

  inputBytes[0] = crc; // set the CRC checksum directly in the output bytes
  return crc;
}
};

#endif // CAYENNECHARGER_H