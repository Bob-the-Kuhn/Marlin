/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
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

/**
 * R25 = 100 kOhm,Epcos B57540G0104G000,
 * 4.7 kOhm pull-up,4.7KOhm series
 * Used in Printrboard G2
 */


const short temptable_80[][2] PROGMEM = {
{ OV(  517), 300},
{ OV(  518), 290},
{ OV(  519), 280},
{ OV(  520), 270},
{ OV(  521), 265},
{ OV(  522), 255},
{ OV(  523), 250},
{ OV(  524), 245},
{ OV(  525), 240},
{ OV(  526), 235},
{ OV(  528), 230},
{ OV(  529), 225},
{ OV(  531), 220},
{ OV(  533), 215},
{ OV(  535), 210},
{ OV(  537), 205},
{ OV(  540), 200},
{ OV(  542), 195},
{ OV(  546), 190},
{ OV(  549), 185},
{ OV(  553), 180},
{ OV(  557), 175},
{ OV(  562), 170},
{ OV(  567), 165},
{ OV(  573), 160},
{ OV(  580), 155},
{ OV(  588), 150},
{ OV(  596), 145},
{ OV(  605), 140},
{ OV(  615), 135},
{ OV(  626), 130},
{ OV(  639), 125},
{ OV(  652), 120},
{ OV(  667), 115},
{ OV(  683), 110},
{ OV(  700), 105},
{ OV(  718), 100},
{ OV(  737),  95},
{ OV(  756),  90},
{ OV(  777),  85},
{ OV(  798),  80},
{ OV(  819),  75},
{ OV(  840),  70},
{ OV(  860),  65},
{ OV(  879),  60},
{ OV(  898),  55},
{ OV(  915),  50},
{ OV(  931),  45},
{ OV(  945),  40},
{ OV(  958),  35},
{ OV(  969),  30},
{ OV(  979),  25},
{ OV(  987),  20},
{ OV(  994),  15},
{ OV( 1000),  10},
{ OV( 1005),   5},
{ OV( 1008),   0},
{ OV( 1012),  -5},
{ OV( 1014), -10},
{ OV( 1016), -15},
{ OV( 1018), -20},
{ OV( 1019), -25},
{ OV( 1020), -30},
{ OV( 1021), -35},
{ OV( 1021), -40},
{ OV( 1022), -45},
}