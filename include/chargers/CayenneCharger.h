#ifndef CayenneCharger_H
#define CayenneCharger_H

#include "chargers/chargerhw.h"
#include "hwinit.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include <libopencm3/stm32/timer.h>
#include <stdint.h>

// Supports the 7.2kW 2-phase Cayenne PHEV charger (5QE 915 681 BQ) on the
// hybrid CAN bus. The 11kW/22kW MLBevo chargers (e-tron, e-tron GT, Taycan) use
// a different message set and are not supported by this module.
class CayenneCharger : public Chargerhw {

public:
  bool ControlCharge(bool RunCh, bool ACReq);
  void DecodeCAN(int id, uint32_t data[2]);
  void SetCanInterface(CanHardware *c);
  void Task10Ms();
  void Task100Ms();
  void Task200Ms();
  void Off();
  void DeInit();

private:
  void CalcValues100ms();
  int stopcharge = 0;
  void handle488(uint32_t data[2]);      // HVLM_06
  void handle53C(uint32_t data[2]);      // HVLM_04
  void handle564(uint32_t data[2]);      // LAD_01
  void handle565(uint32_t data[2]);      // HVLM_03
  void handle67E(uint32_t data[2]);      // LAD_02
  void handle415(uint32_t data[2]);      // stop charge message
  void handle1B000044(uint32_t data[2]); // NMH_Ladegeraet, wake
  void handle12DD5472(uint32_t data[2]); // HVLM_10
  void msg1A1();                         // BMS_02     0x1A1
  void msg39D();                         // BMS_03     0x39D
  void msg184();                         // ZV_01      0x184
  void msg17B();                         // FCU_02     0x17B
  void msg583();                         // ZV_02      0x583
  void msg552();                         // HVEM_05    0x552
  void UnLockCP();

  uint8_t vag_cnt184 = 0x00;

  uint8_t BMS_02[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t BMS_03[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t FCU_02[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t HVEM_05[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t ZV_02[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint8_t ZV_01[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  uint16_t ACvoltage;
  uint32_t HVLM_MaxDC_ChargePower;      // maximum DC charging power
  uint16_t HVLM_Max_DC_Voltage_DCLS;    // maximum DC charging voltage
  uint16_t HVLM_Actual_DC_Current_DCLS; // actual DC charging current
  uint16_t HVLM_Max_DC_Current_DCLS;    // maximum DC charging current
  uint16_t HVLM_Min_DC_Voltage_DCLS;    // minimum DC charging voltage
  uint16_t HVLM_Min_DC_Current_DCLS;    // minimum DC charging current
  uint8_t HVLM_Status_Grid;     // Information as to whether the vehicle is
                                // connected to a power grid.
  uint8_t HVLM_EnergyFlowType;  // Display of whether electricity flows into the
                                // vehicle and what it is used for.
  uint8_t HVLM_OperationalMode; // Current Mode: 0=Inactive, 1=Active, 2=Init,
                                // 3=Error
  uint8_t HVLM_HV_ActivationRequest; // HV activation request and reason for the
                                     // request: 0=No Request, 1=Charging,
                                     // 2=Battery Balancing, 3=AC/Climate
  uint8_t HVLM_ChargerErrorStatus; // Current error status of the charger: 0=No
                                   // Error, 1=DC-NotOK, 2=AC-NotOK,
                                   // 3=Interlock, 4=Reserved, 5=Reserved, 6=No
                                   // Component Function, 7=Init
  uint8_t HVLM_Park_Request;       // Request to lock the drive train
  uint8_t HVLM_Park_Request_Maintain; // Request to keep the drive train locked
  uint8_t
      HVLM_Plug_Status; // Plug detection status independent of charging mode
                        // (AC or DC): 0=Init, 1=No Plug Inserted, 2=Connector
                        // Inserted, Not Locked, 3=Connector Inserted & Locked
  uint8_t HVLM_LoadRequest; // Charger Status: 0=No Request, 1=AC Charge, 2=DC
                            // Charge, 3=Recharge 12V, 4=AC AWC Charge,
                            // 5=Reserved, 6=Init, 7=Error
  uint8_t HVLM_MaxBattChargeCurrent; // Recommended HV battery charging current
                                     // for a planned charge
  uint8_t LAD_Status_Voltage;        //
  uint16_t LAD_PowerLossVal;         // Instantaneous value: power loss charger
  uint16_t HVLM_HV_StaleTime; // Period between HV deactivated and HV activated
  uint8_t
      HVLM_ChargeSystemState; // Displaying status about the charging system.
                              // 0=No issue, 1=System Defective, 2=System
                              // Incompatable 3=DC Charge not possible
  uint8_t HVLM_Status_LED;    // Status of the charging LED: 0=Colour1-off,
                              // 1=Colour2-White, 2=Colour3-Yellow,
  // 3=Colour4-Green, 4=Colour5-Red, 5=Yellow Pulsing,
  // 6=Green Pulsing, 7=Red Pulsing, 8=Green/Red
  // Pulsing, 9=Green Flashing, 14=Init, 15=Error
  bool HVLM_LG_ChargerTargetMode; // AC charger target mode: 0=Standby, 1=Mains
                                  // Charging
  uint8_t HVLM_TankCapReleaseRequest;     // Fuel Cap Release: 0=No Release,
                                          // 1=Release, 2=Init, 3=Error
  uint8_t HVLM_RequestConnectorLock;      // Request connector lock: 0=Unlock,
                                          // 1=lock, 2=Init, 3=No Request
  uint8_t HVLM_Start_VoltageMeasure_DCLS; // Message to the BMS that a
                                          // measurement voltage is being
                                          // applied to the DC charging station:
                                          // 0=Inactive, 1=DCLS With Diode,
                                          // 2=DCLS without Diode, 3=Reserve
  uint8_t HVLM_ChargeReadyStatus;  // Display whether AC or DC charging is not
                                   // possible: 0=No Error, 1=AC Charge Not
                                   // Possible, 2=DC Charge Not Possible, 3=AC &
                                   // DC Charge Not Possible
  uint16_t HVLM_Output_Voltage_HV; // Output voltage of the charger and DC
                                   // voltage of the charging station.
                                   // Measurement between the DC HV lines.
  bool LAD_Reduction_ChargerTemp;  // Reduction due to internal overtemperature
                                   // in the charger
  bool LAD_Reduction_Current;    // Regulation due to current or voltage at the
                                 // input or output
  bool LAD_Reduction_SocketTemp; // Reduction due to excessive charging socket
                                 // temperature
  uint16_t LAD_MaxChargerPower_HV; // Maximum power charger in relation to the
                                   // maximum power infrastructure (cable,
                                   // charging station) and taking into account
                                   // the charger efficiency
  bool LAD_ControlPilotStatus; // Status control pilot monitoring (detection of
                               // the control pilot duty cycle)
  bool LAD_LockFeedback; // Status of connector lock (feedback contact of the
                         // locking actuator)
  uint8_t LAD_ChargerCoolingDemand; // Cooling demand of the charger
  bool LAD_ChargerWarning; // Collective warning charger: 0=Normal, 1=Warning
  bool LAD_ChargerFault;   // Collective error charger: 0=Normal, 1=No Charging
                           // Possible
  uint8_t HVLM_RtmWarnLadeverbindung; // RTM Charging connection fault
  uint8_t
      HVLM_RtmWarnLadesystem; // RTM Electrical machine CAN communication fault
  uint8_t HVLM_RtmWarnLadestatus;        // RTM Warning of charging status fault
  uint8_t HVLM_RtmWarnLadeKommunikation; // RTM Warning of charging communction
                                         // fault
  uint16_t BMS_MaxDischarge_Curr;
  uint16_t BMS_Min_Batt_Volt;
  uint16_t BMS_Min_Batt_Volt_Discharge;
  uint16_t BMS_MaxCharge_Curr;
  uint16_t BMS_MaxCharge_Curr_Offset;
  uint16_t BMS_Batt_Max_Volt;
  uint16_t BMS_Min_Batt_Volt_Charge;
  uint16_t BMS_OpenCircuit_Volts;
  int carwakeup;

  bool HVEM_Nachladen_Anf;
  uint16_t HVEM_SollStrom_HV;
  uint16_t HVEM_MaxSpannung_HV;
  bool ZV_FT_verriegeln;
  bool ZV_FT_entriegeln;
  bool ZV_BT_verriegeln;
  bool ZV_BT_entriegeln;
  bool ZV_entriegeln_Anf;
  bool ZV_verriegelt_intern_ist;
  bool ZV_verriegelt_extern_ist;
  bool ZV_verriegelt_intern_soll;
  bool ZV_verriegelt_extern_soll;
  bool ZV_verriegelt_soll;
  bool ZV_Tankklappe_offen;
  bool ZV_Rollo_auf;
  bool ZV_Rollo_zu;
  bool ZV_SAD_auf;
  bool ZV_SAD_zu;
  bool BCM_Tankklappensteller_Fehler;
  bool FCU_TK_Betankung_Anforderung;
  uint8_t FCU_TK_Freigabe_Tankklappe;

  uint16_t HVVoltage;
  int8_t temperature;
  uint8_t mode;
  uint16_t current;
  float hvCurrent; // A, charger HV output current from LAD_01
  uint8_t MaxACAmps;
  uint8_t PPLim;
  bool clearToStart = false;
};

#endif // CAYENNECHARGER_H