/************************************************************************************//**
* \file         Demo/ARMCM4_STM32F4_Nucleo_F429ZI_GCC/Boot/flash_layout.c
* \brief        Custom flash layout table source file.
* \ingroup      Boot_ARMCM4_STM32F4_Nucleo_F429ZI_GCC
* \internal
*----------------------------------------------------------------------------------------
*                          C O P Y R I G H T
*----------------------------------------------------------------------------------------
*   Copyright (c) 2026  by Feaser    http://www.feaser.com    All rights reserved
*
*----------------------------------------------------------------------------------------
*                            L I C E N S E
*----------------------------------------------------------------------------------------
* This file is part of OpenBLT. OpenBLT is free software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published by the Free
* Software Foundation, either version 3 of the License, or (at your option) any later
* version.
*
* OpenBLT is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
* without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. See the GNU General Public License for more details.
*
* You have received a copy of the GNU General Public License along with OpenBLT. It
* should be located in ".\Doc\license.html". If not, contact Feaser to obtain a copy.
*
* \endinternal
****************************************************************************************/

/** \brief   Array wit the layout of the flash memory.
 *  \details Also controls what part of the flash memory is reserved for the bootloader.
 *           If the bootloader size changes, the reserved sectors for the bootloader
 *           might need adjustment to make sure the bootloader doesn't get overwritten.
 */
static const tFlashSector flashLayout[] =
#ifdef F413
{
  /* space is reserved for a bootloader configuration with all supported communication
   * interfaces enabled. when for example only UART is needed, then the space required
   * for the bootloader can be made a lot smaller here.
   */
  /* { 0x08000000, 0x04000,  0},           flash sector  0 - reserved for bootloader   */
  /* { 0x08004000, 0x04000,  1},           flash sector  1 - reserved for bootloader   */
  /* { 0x08008000, 0x04000,  2},           flash sector  2 - reserved for bootloader   */
  { 0x0800c000, 0x04000,  3},           /* flash sector  3 -  16kb                     */
  { 0x08010000, 0x10000,  4},           /* flash sector  4 -  64kb                     */
  { 0x08020000, 0x20000,  5},           /* flash sector  5 - 128kb                     */
  { 0x08040000, 0x20000,  6},           /* flash sector  6 - 128kb                     */
  { 0x08060000, 0x20000,  7},           /* flash sector  7 - 128kb                     */
  { 0x08080000, 0x20000,  8},           /* flash sector  8 - 128kb                     */
  { 0x080A0000, 0x20000,  9},           /* flash sector  9 - 128kb                     */
  { 0x080C0000, 0x20000, 10},           /* flash sector 10 - 128kb                     */
  { 0x080E0000, 0x20000, 11},           /* flash sector 11 - 128kb                     */
  { 0x08100000, 0x04000, 12},           /* flash sector 12 -  16kb                     */
  { 0x08104000, 0x04000, 13},           /* flash sector 13 -  16kb                     */
  { 0x08108000, 0x04000, 14},           /* flash sector 14 -  16kb                     */
  { 0x0810c000, 0x04000, 15},           /* flash sector 15 -  16kb                     */
  { 0x08110000, 0x10000, 16},           /* flash sector 16 -  64kb                     */
  { 0x08120000, 0x20000, 17},           /* flash sector 17 - 128kb                     */
  { 0x08140000, 0x20000, 18},           /* flash sector 18 - 128kb                     */
  { 0x08160000, 0x20000, 19},           /* flash sector 19 - 128kb                     */
  { 0x08180000, 0x20000, 20},           /* flash sector 20 - 128kb                     */
  { 0x081A0000, 0x20000, 21},           /* flash sector 21 - 128kb                     */
  { 0x081C0000, 0x20000, 22},           /* flash sector 22 - 128kb                     */
  { 0x081E0000, 0x20000, 23},           /* flash sector 23 - 128kb                     */
};
#elif H725
{
/* { 0x08000000, 0x20000,  0},           flash sector  0 - reserved for bootloader   */
  { 0x08020000, 0x20000,  1},           /* flash sector  1 - 128kb                     */
  { 0x08040000, 0x20000,  2},           /* flash sector  2 - 128kb                     */
  { 0x08060000, 0x20000,  3},           /* flash sector  3 - 128kb                     */
  { 0x08080000, 0x20000,  4},           /* flash sector  4 - 128kb                     */
  { 0x080A0000, 0x20000,  5},           /* flash sector  5 - 128kb                     */
  { 0x080C0000, 0x20000,  6},           /* flash sector  6 - 128kb                     */
  { 0x080E0000, 0x20000,  7},           /* flash sector  7 - 128kb                     */
};
#endif


/*********************************** end of flash_layout.c *****************************/
