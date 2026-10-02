/*
 * This file is part of the ZombieVerter project.
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

#include "vag_hvk.h"
#include "params.h"
#include "vag_crc.h"

#define MAX_CAN 2
#define MIN_VALID_PACK 50 // V, below this the pack side reading is not trusted

namespace VagHvk {
Hvk01 hvk01;
Klemmen klemmen;
Bms01 bms01;
} // namespace VagHvk

namespace {
struct Requests {
  CanHardware *can[MAX_CAN];
  int count;
};

Requests requests10Ms;
Requests requests100Ms;
uint8_t vag_cnt191 = 0x00;
uint8_t vag_cnt3C0 = 0x00;
uint8_t vag_cnt503 = 0x00;

void Add(Requests &requests, CanHardware *can) {
  for (int i = 0; i < requests.count; i++)
    if (requests.can[i] == can)
      return;
  if (requests.count < MAX_CAN)
    requests.can[requests.count++] = can;
}

void SendToAll(Requests &requests, uint32_t id, uint8_t *bytes) {
  for (int i = 0; i < requests.count; i++)
    requests.can[i]->Send(id, bytes, 8);
}

void NextCounter(uint8_t &counter) {
  counter++;
  if (counter > 0x0f)
    counter = 0x00;
}

void Msg191() // BMS_01
{
  using VagHvk::bms01;
  uint16_t BMS_Batt_Volt = VagHvk::PackVoltage() * 4;
  uint16_t BMS_Batt_Volt_HVterm = bms01.linkVoltage * 2;

  // Use the VCU SoC, falling back to a fixed value when no SoC source is
  // configured
  float soc = Param::GetFloat(Param::SOC);
  if (soc <= 0)
    soc = 35.1f;
  uint16_t BMS_SOC_HiRes = soc * 20; // 0.05% per bit

  uint8_t bytes[8];
  bytes[0] = 0x00;
  bytes[1] = vag_cnt191 | ((bms01.BMS_Batt_Curr & (0x0FU)) << 4);
  bytes[2] = ((bms01.BMS_Batt_Curr >> 4) & (0xFFU));
  bytes[3] = (BMS_Batt_Volt & (0xFFU));
  bytes[4] = ((BMS_Batt_Volt >> 8) & (0x0FU)) |
             ((BMS_Batt_Volt_HVterm & (0x0FU)) << 4);
  bytes[5] = ((BMS_Batt_Volt_HVterm >> 4) & (0x7FU)) |
             ((BMS_SOC_HiRes & (0x01U)) << 7);
  bytes[6] = ((BMS_SOC_HiRes >> 1) & (0xFFU));
  bytes[7] = ((BMS_SOC_HiRes >> 9) & (0x03U));
  VagCrc::Calc(bytes, 8, 0x191);
  SendToAll(requests10Ms, 0x191, bytes);
  NextCounter(vag_cnt191);
}

void Msg3C0() // Klemmen_Status_01
{
  using VagHvk::klemmen;
  uint8_t bytes[8] = {0};
  bytes[1] = vag_cnt3C0;
  bytes[2] = (klemmen.ZAS_Kl_S & (0x01U)) |
             ((klemmen.ZAS_Kl_15 & (0x01U)) << 1) |
             ((klemmen.ZAS_Kl_X & (0x01U)) << 2) |
             ((klemmen.ZAS_Kl_50_Startanforderung & (0x01U)) << 3);
  VagCrc::Calc(bytes, 8, 0x3C0);
  SendToAll(requests100Ms, 0x3C0, bytes);
  NextCounter(vag_cnt3C0);
}

void Msg503() // HVK_01
{
  using VagHvk::hvk01;
  // While a DC-DC precharges the link the BMS stays off, as in the vehicle
  uint8_t BMS_Sollmodus = hvk01.Vorladung_aktiv ? 0 : hvk01.BMS_Sollmodus;
  bool HV_Bordnetz_aktiv = hvk01.HV_Bordnetz_aktiv && !hvk01.Vorladung_aktiv;

  uint8_t bytes[8] = {0};
  bytes[1] = vag_cnt503 | ((hvk01.Vorladung_aktiv & (0x01U)) << 7);
  bytes[2] = (hvk01.MO_EmSollzustand & (0xFFU));
  bytes[3] =
      (BMS_Sollmodus & (0x07U)) | ((hvk01.DCDC_Sollmodus & (0x07U)) << 3);
  bytes[4] = ((hvk01.HVLM_Sollmodus & (0x07U)) << 4);
  bytes[5] = ((HV_Bordnetz_aktiv & (0x01U)) << 1) |
             ((hvk01.Gesamtst_Spgfreiheit & (0x03U)) << 3);
  VagCrc::Calc(bytes, 8, 0x503);
  SendToAll(requests100Ms, 0x503, bytes);
  NextCounter(vag_cnt503);
}
} // namespace

float VagHvk::PackVoltage() {
  int shuntType = Param::GetInt(Param::ShuntType);
  float udc2 = Param::GetFloat(Param::udc2);

  if (hvk01.Vorladung_aktiv)
    return Param::GetFloat(Param::udcsw);
  if (shuntType >= 1 && shuntType <= 4 && udc2 > MIN_VALID_PACK)
    return udc2;
  return bms01.packVoltage;
}

void VagHvk::Request10Ms(CanHardware *can) { Add(requests10Ms, can); }

void VagHvk::Request100Ms(CanHardware *can) { Add(requests100Ms, can); }

void VagHvk::Task10Ms() {
  if (requests10Ms.count == 0)
    return;
  Msg191();
  requests10Ms.count = 0;
}

void VagHvk::Task100Ms() {
  if (requests100Ms.count == 0)
    return;
  Msg3C0();
  Msg503();
  requests100Ms.count = 0;
}
