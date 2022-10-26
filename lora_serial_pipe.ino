// Feather9x_TX
// -*- mode: C++ -*-
// Prototype sketch for a tty-USB-lora pipe
// for now handles either RX or TX depending on a define

#include <SPI.h>
#include <RH_RF95.h>



/* for feather32u4 */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7


#define TX 1
#define RX 0
#define MODE TX


#if defined(ESP8266)
  /* for ESP w/featherwing */ 
  #define RFM95_CS  2    // "E"
  #define RFM95_RST 16   // "D"
  #define RFM95_INT 15   // "B"

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)
  // Feather M0 w/Radio
  #define RFM95_CS      8
  #define RFM95_INT     3
  #define RFM95_RST     4

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  #define RFM95_INT     9  // "A"
  #define RFM95_CS      10  // "B"
  #define RFM95_RST     11  // "C"
  
#elif defined(ESP32)  
  /* ESP32 feather w/wing */
  #define RFM95_RST     27   // "A"
  #define RFM95_CS      33   // "B"
  #define RFM95_INT     12   //  next to A

#elif defined(ARDUINO_NRF52832_FEATHER)
  /* nRF52832 feather w/wing */
  #define RFM95_RST     7   // "A"
  #define RFM95_CS      11   // "B"
  #define RFM95_INT     31   // "C"
  
#elif defined(TEENSYDUINO)
  /* Teensy 3.x w/wing */
  #define RFM95_RST     9   // "A"
  #define RFM95_CS      10   // "B"
  #define RFM95_INT     4    // "C"
#endif

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

//makes buffered line reading better
//SerialLineReader reader(Serial);

void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.begin(115200);
  while (!Serial) {
    delay(1);
  }

  delay(100);

  Serial.println("Feather LoRa TX Serial Pipe!");

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  if(MODE==TX){
    Serial.println("Entering TX Mode");
  }else{
    Serial.println("Entering RX Mode");
    Serial.print("max message length is"); Serial.println(RH_RF95_MAX_MESSAGE_LEN);
  }
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(23, false);

  //Send an online message
  char hellomsg[9] = "Tx Ready";
  rf95.send((uint8_t *)hellomsg,9);

}

void loop() 
{
  //Wait for characters on either
  // when received, print them on the USB terminal
  //

    
  if (MODE==TX)
  {// wait for while lines from serial. Send as a packet
    
    // reader.poll(); //watch for characters
	  // if(reader.available())  //if a /n comes, read out the string
    // {
		//   char text[reader.len()];
	  // 	reader.read(text);
		//   //Serial.println(text);
	  // }
    
    // read entire line from serial
    // from https://arduinogetstarted.com/reference/serial-readstringuntil
    if (Serial.available() > 0) 
    {
    // read the incoming string:
    String incomingString = Serial.readStringUntil('\n');

    int msglen = incomingString.length();//
    msglen++;//add +1 to account for \n?
    // prints the received data
    digitalWrite(13, HIGH);   // turn the LED on 
    Serial.print("I received: ");
    Serial.println(incomingString);
    Serial.print("Length:"); Serial.println(msglen);
    Serial.print("Sending "); Serial.println(incomingString);
    //radiopacket[19] = 0; //in the lora example the last byte is set to 0.  Why?
    
    Serial.println("Sending...");
    delay(10);
    //rf95.send((uint8_t *)incomingString, 20);
    uint8_t buf[msglen];
    incomingString.toCharArray(buf,msglen);
    rf95.send(buf,msglen);
    delay(10);
    rf95.waitPacketSent();
    Serial.println("Done ok.");
    digitalWrite(13, LOW);   // turn the LED on 
    }

  }
  else //RX
  {//wait for packets, print as lines
    if (rf95.waitAvailableTimeout(1000))
    {
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      // Should be a reply message for us now   
      if (rf95.recv(buf, &len))
      {
        Serial.print("RX: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);    
      }
      else
      {
        Serial.println("No Downlink");
      }
    }
  }
 
  

}