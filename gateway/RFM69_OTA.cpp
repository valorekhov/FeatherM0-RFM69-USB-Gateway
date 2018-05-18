// **********************************************************************************
// Library for OTA wireless programming of Moteinos using an RFM69 transceiver
// **********************************************************************************
// Hardware requirements:
//   - DualOptiboot bootloader - ships with all Moteinos
//   - SPI "Flash MEM" chip on Moteino (optional)
// Library requirements:
//   - RFM69      - get library at: https://github.com/LowPowerLab/RFM69
//   - SPIFLash.h - get it here: http://github.com/LowPowerLab/SPIFlash
// **********************************************************************************
// Copyright LowPowerLab LLC 2018, https://www.LowPowerLab.com/contact
// **********************************************************************************
// License
// **********************************************************************************
// This program is free software; you can redistribute it 
// and/or modify it under the terms of the GNU General    
// Public License as published by the Free Software       
// Foundation; either version 3 of the License, or        
// (at your option) any later version.                    
//                                                        
// This program is distributed in the hope that it will   
// be useful, but WITHOUT ANY WARRANTY; without even the  
// implied warranty of MERCHANTABILITY or FITNESS FOR A   
// PARTICULAR PURPOSE. See the GNU General Public        
// License for more details.                              
//                                                        
// Licence can be viewed at                               
// http://www.gnu.org/licenses/gpl-3.0.txt
//
// Please maintain this license information along with authorship
// and copyright notices in any redistribution of this code
// **********************************************************************************
#include "RFM69_OTA.h"
#include "RFM69registers.h"

#ifdef __AVR__
  #include <avr/wdt.h>
#endif

#ifdef MOTEINO_ZERO
  #if defined(SERIAL_PORT_USBVIRTUAL)
    #define Serial SERIAL_PORT_USBVIRTUAL // output on SerialUSB instead of Serial
  #endif
#endif

//===================================================================================================================
// validateHEXData() - returns length of HEX data bytes if everything is valid
//returns 0 if any validation failed
//===================================================================================================================
uint8_t validateHEXData(void* data, uint8_t length)
{
  //assuming 1 byte record length, 2 bytes address, 1 byte record type, N data bytes, 1 CRC byte
  char* input = (char*)data;
  if (length <12 || length%2!=0) return 0; //shortest possible intel data HEX record is 12 bytes
  //Serial.print(F("VAL > ")); Serial.println((char*)input);

  uint8_t checksum=0;
  //check valid HEX data and CRC
  for (uint8_t i=0; i<length;i++)
  {
    if (!((input[i] >=48 && input[i]<=57) || (input[i] >=65 && input[i]<=70))) //0-9,A-F
      return 255;
    if (i%2 && i<length-2) checksum+=BYTEfromHEX(input[i-1], input[i]);
  }
  checksum=(checksum^0xFF)+1;
  
  //TODO : CHECK for address continuity (intel HEX addresses are big endian)

  //Serial.print(F("final CRC:"));Serial.println((uint8_t)checksum, HEX);
  //Serial.print(F("CRC byte:"));Serial.println(BYTEfromHEX(input[length-2], input[length-1]), HEX);

  //check CHECKSUM byte
  if (((uint8_t)checksum) != BYTEfromHEX(input[length-2], input[length-1]))
    return 254;

  uint8_t dataLength = BYTEfromHEX(input[0], input[1]); //length of actual HEX flash data (usually 16bytes)
  //calculate record length
  if (length != dataLength*2 + 10) //add headers and checksum bytes (a total of 10 combined)
    return 253;

  return dataLength; //all validation OK!
}


//===================================================================================================================
// prepareSendBuffer() - returns the final size of the buf
//===================================================================================================================
uint8_t prepareSendBuffer(char* hexdata, uint8_t*buf, uint8_t length, uint16_t seq)
{
  uint8_t seqLen = sprintf(((char*)buf), "FLX:%u:", seq);
  for (uint8_t i=0; i<length;i++)
    buf[seqLen+i] = BYTEfromHEX(hexdata[i*2], hexdata[i*2+1]);
  return seqLen+length;
}


//===================================================================================================================
// BYTEfromHEX() - converts from ASCII HEX to byte, assume A and B are valid HEX chars [0-9A-F]
//===================================================================================================================
uint8_t BYTEfromHEX(char MSB, char LSB)
{
  return (MSB>=65?MSB-55:MSB-48)*16 + (LSB>=65?LSB-55:LSB-48);
}


//===================================================================================================================
// sendHEXPacket() - return the SEQ of the ACK received, or -1 if invalid
//===================================================================================================================
uint8_t sendHEXPacket(RFM69& radio, uint8_t targetID, uint8_t* sendBuf, uint8_t hexDataLen, uint16_t seq, uint16_t TIMEOUT, uint16_t ACKTIMEOUT, uint8_t DEBUG)
{
  long now = millis();
  
  while(1) {
    if (DEBUG) { Serial.print(F("RFTX > ")); PrintHex83(sendBuf, hexDataLen); }
    if (radio.sendWithRetry(targetID, sendBuf, hexDataLen, 2, ACKTIMEOUT))
    {
      uint8_t ackLen = radio.DATALEN;
      
      if (DEBUG) { Serial.print(F("RFACK > ")); Serial.print(ackLen); Serial.print(F(" > ")); PrintHex83((uint8_t*)radio.DATA, ackLen); }
      
      if (ackLen >= 8 && radio.DATA[0]=='F' && radio.DATA[1]=='L' && radio.DATA[2]=='X' && 
          radio.DATA[3]==':' && radio.DATA[ackLen-3]==':' &&
          radio.DATA[ackLen-2]=='O' && radio.DATA[ackLen-1]=='K')
      {
        uint16_t tmp=0;
        sscanf((const char*)radio.DATA, "FLX:%u:OK", &tmp);
        return tmp == seq;
      }
    }

    if (millis()-now > TIMEOUT)
    {
      Serial.println(F("Timeout waiting for packet ACK, aborting FLASH operation ..."));
      break; //abort FLASH sequence if no valid ACK was received for a long time
    }
  }
  return false;
}


//===================================================================================================================
// PrintHex83() - prints 8-bit data in HEX format
//===================================================================================================================
void PrintHex83(uint8_t* data, uint8_t length) 
{
  uint8_t tmp;
  for (uint8_t i=0; i<length; i++) 
  {
    tmp = (data[i] >> 4) | 48;
    Serial.print((char)((tmp > 57) ? tmp+7 : tmp));
    tmp = (data[i] & 0x0F) | 48;
    Serial.print((char)((tmp > 57) ? tmp+7 : tmp));
  }
}

//===================================================================================================================
// resetUsingWatchdog() - Use watchdog to reset the MCU
//===================================================================================================================
void resetUsingWatchdog(uint8_t DEBUG)
{
#ifdef __AVR__
  //wdt_disable();
  if (DEBUG) Serial.print(F("REBOOTING"));
  wdt_enable(WDTO_15MS);
  while(1) if (DEBUG) Serial.print(F("."));
#elif defined(MOTEINO_ZERO)
  WDT->CTRL.reg = 0; // disable watchdog
  while (WDT->STATUS.bit.SYNCBUSY == 1); // sync is required
  WDT->CONFIG.reg = 0; // see Table 18.8.2 Timeout Period (valid values 0-11)
  WDT->CTRL.reg = WDT_CTRL_ENABLE; //enable WDT
  while (WDT->STATUS.bit.SYNCBUSY == 1);
  WDT->CLEAR.reg= 0x00; // system reset via WDT
  while (WDT->STATUS.bit.SYNCBUSY == 1);
#endif
}
