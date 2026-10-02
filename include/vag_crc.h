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
 *
 * VAG E2E CRC8 (AUTOSAR CRC8H2F with a per message, per counter magic byte),
 * shared by the VW/Audi/Porsche modules on the hybrid CAN bus.
 */

#ifndef VAG_CRC_H
#define VAG_CRC_H

#include <stdint.h>

namespace VagCrc {
// Calculates the CRC over bytes[1..length-1] plus the magic byte for the
// counter in the low nibble of bytes[1], writes it to bytes[0] and returns it.
uint8_t Calc(uint8_t *bytes, uint8_t length, uint32_t id);
} // namespace VagCrc

#endif // VAG_CRC_H
