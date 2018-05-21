// Adaptation of LowPowerLab gateway.ino sketch to Feather M0 RFM69 module

// **********************************************************************************
#include "RFM69.h"         //get it here: https://www.github.com/lowpowerlab/rfm69
#include "RFM69_ATC.h"     //get it here: https://www.github.com/lowpowerlab/rfm69
#include <SPI.h>           //included with Arduino IDE install (www.arduino.cc)

//*********************************************************************************************
//************ IMPORTANT SETTINGS - YOU MUST CHANGE/CONFIGURE TO FIT YOUR HARDWARE *************
//*********************************************************************************************
//#define NODEID        1    // Moved to Network_Config.h
//#define NETWORKID     100  // Moved to Network_Config.h
//Match frequency to the hardware version of the radio on your Moteino (uncomment one):
#define FREQUENCY     RF69_433MHZ // Moved to Network_Config.h
//#define FREQUENCY     RF69_868MHZ
//#define FREQUENCY     RF69_915MHZ
//#define ENCRYPTKEY    "sampleEncryptKey" // Moved to Network_Config.h
#include "Network_Config.h"

#define IS_RFM69HW_HCW  1 //uncomment only for RFM69HW/HCW! Leave out if you have RFM69W/CW!
//*********************************************************************************************
//Auto Transmission Control - dials down transmit power to save battery
//Usually you do not need to always transmit at max output power
//By reducing TX power even a little you save a significant amount of battery power
//This setting enables this gateway to work with remote nodes that have ATC enabled to
//dial their power down to only the required level
#define ENABLE_ATC    //comment out this line to disable AUTO TRANSMISSION CONTROL
//*********************************************************************************************
#define SERIAL_BAUD   115200

// for Feather M0 Radio
#define RFM69_CS      8
#define RFM69_IRQ     3
#define RFM69_IRQN    3  // Pin 3 is IRQ 3!
#define RFM69_RST     4


#define LED LED_BUILTIN

#ifdef ENABLE_ATC
  RFM69_ATC radio = RFM69_ATC(RFM69_CS, RFM69_IRQ, IS_RFM69HW_HCW, RFM69_IRQN);
#else
  RFM69 radio = RFM69(RFM69_CS, RFM69_IRQ, IS_RFM69HW_HCW, RFM69_IRQN);
#endif

bool promiscuousMode = false; //set to 'true' to sniff all packets on the same network

void setup() {
  while (!Serial) {blink(1000);blink(1500);blink(2000);}
  Serial.begin(SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
  blink(3000);  

  pinMode(RFM69_RST, OUTPUT);
  digitalWrite(RFM69_RST, HIGH);
  delay(100);
  digitalWrite(RFM69_RST, LOW);
  delay(100);

  radio.initialize(FREQUENCY,NODEID,NETWORKID);
#ifdef IS_RFM69HW_HCW
  radio.setHighPower(); //must include this only for RFM69HW/HCW!
#endif
  radio.encrypt(ENCRYPTKEY);
  radio.promiscuous(promiscuousMode);
  //radio.setFrequency(919000000); //set frequency to some custom frequency
  char buff[50];
  sprintf(buff, "\nListening at %d Mhz...", FREQUENCY==RF69_433MHZ ? 433 : FREQUENCY==RF69_868MHZ ? 868 : 915);
  Serial.println(buff);
  /*if (flash.initialize())
  {
    Serial.print("SPI Flash Init OK. Unique MAC = [");
    flash.readUniqueId();
    for (byte i=0;i<8;i++)
    {
      Serial.print(flash.UNIQUEID[i], HEX);
      if (i!=8) Serial.print(':');
    }
    Serial.println(']');
    
    //alternative way to read it:
    //byte* MAC = flash.readUniqueId();
    //for (byte i=0;i<8;i++)
    //{
    //  Serial.print(MAC[i], HEX);
    //  Serial.print(' ');
    //}
  }
  else*/
    Serial.println("SPI Flash MEM not found (is chip soldered?)...");
    
#ifdef ENABLE_ATC
  Serial.println("RFM69_ATC Enabled (Auto Transmission Control)");
#endif
}

void loop() {
  //process any serial input
  if (Serial.available() > 0)
  {
    char input = Serial.read();
    if (input == 'r') //d=dump all register values
      radio.readAllRegs();
    if (input == 'E') //E=enable encryption
      radio.encrypt(ENCRYPTKEY);
    if (input == 'e') //e=disable encryption
      radio.encrypt(null);
    if (input == 'p')
    {
      promiscuousMode = !promiscuousMode;
      radio.promiscuous(promiscuousMode);
      Serial.print("Promiscuous mode ");Serial.println(promiscuousMode ? "on" : "off");
    }
    
    /*if (input == 'd') //d=dump flash area
    {
      Serial.println("Flash content:");
      int counter = 0;

      while(counter<=256){
        Serial.print(flash.readByte(counter++), HEX);
        Serial.print('.');
      }
      while(flash.busy());
      Serial.println();
    }
    if (input == 'D')
    {
      Serial.print("Deleting Flash chip ... ");
      flash.chipErase();
      while(flash.busy());
      Serial.println("DONE");
    }
    if (input == 'i')
    {
      Serial.print("DeviceID: ");
      word jedecid = flash.readDeviceId();
      Serial.println(jedecid, HEX);
    }*/
    if (input == 't')
    {
      byte temperature =  radio.readTemperature(-1); // -1 = user cal factor, adjust for correct ambient
      byte fTemp = 1.8 * temperature + 32; // 9/5=1.8
      Serial.print( "Radio Temp is ");
      Serial.print(temperature);
      Serial.print("C, ");
    }
    if (input == '\xFC'){
      input = Serial.read();
      if (input == '\x00'){ // NOP
        Serial.print("\xFC\x01");
        Serial.write((byte)0);
        Serial.flush();
        Serial.write(0x0A);
      }
      if (input == '\x10'){ // SEND
        byte state = Serial.read();
        byte dstNodeId = Serial.read();
        byte retries = Serial.read();
        byte len = Serial.read();
        char* txBuf = new char[len];
        for(byte i = 0; i < len; i++){
          txBuf[i] = Serial.read();
        }
        
        Serial.print("\xFC\x01\x10");
        Serial.write(state);
        if (radio.sendWithRetry(dstNodeId, txBuf, len, retries)){  // 0 = only 1 attempt, no retries
          Serial.write((byte)0);
        } else {
          Serial.write((byte)1);
        }
        Serial.write(0x0A);
        Serial.flush();
        delete [] txBuf;
      }
    }
  }

  if (radio.receiveDone())
  {
    char buf[3];
    Serial.print("\xFC\x11");
    encodeByte(NETWORKID);
    encodeByte(radio.SENDERID);
    encodeByte(radio.TARGETID);
    encodeByte(radio.RSSI);
    encodeByte(radio.DATALEN);
    for (byte i = 0; i < radio.DATALEN; i++){
      encodeByte((uint8_t)radio.DATA[i]);
    }
    
    if (radio.ACKRequested())
    {
      byte theNodeID = radio.SENDERID;
      radio.sendACK();
      Serial.write((byte)1);
    } else {
      Serial.write((byte)0);
    }
    Serial.write(0x0A);
    Serial.flush();
    blink(100);blink(100);blink(100);
  }
}

void encodeByte(uint8_t value){
  if (value == 0xFC){
    Serial.print("\xFC\xFC");
  } else if (value == 0x0A){
    Serial.print("\xFC\xFF");
  } else {
    Serial.write(value);
  }
}

void blink(unsigned int delayMs)
{
  pinMode(LED, OUTPUT);
  digitalWrite(LED,HIGH);
  delay(delayMs);
  digitalWrite(LED,LOW);
}
