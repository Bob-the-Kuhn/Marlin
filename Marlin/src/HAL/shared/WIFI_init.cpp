/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 */


#include "../../inc/MarlinConfig.h"
#include "Delay.h"

void WIFI_init(void) {
  #if defined(ESP_WIFI_MODULE_RESET_PIN) && (ESP_WIFI_MODULE_RESET_PIN != -1)
    OUT_WRITE(ESP_WIFI_MODULE_RESET_PIN,0);
    delay(1);
    OUT_WRITE(ESP_WIFI_MODULE_RESET_PIN,1);
  #endif
  #if defined(ESP_WIFI_MODULE_ENABLE_PIN) && (ESP_WIFI_MODULE_ENABLE_PIN != -1)
    OUT_WRITE(ESP_WIFI_MODULE_ENABLE_PIN,1);
  #endif
}