#include <SPI.h>
#include <RH_RF95.h>

// Todo: implement counter for radio check
#define WAITFORREPLY false // if false system will send a single radio packet and continue init.
#define WAITTIME 10 // override radio init after 10 tries and continue

/* for feather m0  */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

#define LED 13 //blinky on receipt
// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);


bool radio_init() {
  pinMode(RFM95_RST, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setTxPower(23, false);
  // init complete, test send packet


  bool reply = WAITFORREPLY;
  uint8_t wait_time = WAITTIME;
  if (reply) {
    while (wait_time > 0) {
      char radiopacket[12] = "Test packet";
      rf95.send((uint8_t *)radiopacket, 12);
      rf95.waitPacketSent();
      uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
      uint8_t len = sizeof(buf);
      if (rf95.waitAvailableTimeout(1000))
      {
        // Should be a reply message for us now
        if (rf95.recv(buf, &len))
        {
          digitalWrite(LED, HIGH);
          Serial.print("Got reply: ");
          Serial.println((char*)buf);
          Serial.print("RSSI: ");
          Serial.println(rf95.lastRssi(), DEC);
          digitalWrite(LED, LOW);
          return true;
        }
        else
        {
          Serial.println("Receive failed");
        }
      }
      else
      {
        Serial.println("No reply, is there a listener around?");
        wait_time = wait_time + 1;
      }
    }
    return false;
  }


  // Okay if we are not waiting on a reply, skip an infinite loop
  // and go straight to a single test
  else
  {
    char radiopacket[12] = "Test packet";
    rf95.send((uint8_t *)radiopacket, 12);
    rf95.waitPacketSent();
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    if (rf95.waitAvailableTimeout(1000))
      // Should be a reply message for us now
      if (rf95.recv(buf, &len))
      {
        digitalWrite(LED, HIGH);
        Serial.print("Got reply: ");
        Serial.println((char*)buf);
        Serial.print("RSSI: ");
        Serial.println(rf95.lastRssi(), DEC);
        digitalWrite(LED, LOW);
      }
      else
      {
        Serial.println("Receive failed");
      }
  
  else
  {
    Serial.println("No reply, is there a listener around?");
  }
  return true; //send true regardless if we don't care about establishing a comms channel.
}
}

