#include <M5Stack.h>				//  https://github.com/m5stack/M5Stack
#include <M5_UI.h>					//  https://github.com/dsiberia9s/M5_UI
#include "BLoRaWan.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"


//Task
TaskHandle_t TaskGPS;
TaskHandle_t TaskPixel;

//Image
extern const unsigned char gImage_logoM5[];

char * appKey = "AABBCCDDAABBCCDDAABBCCDDAABBCCDD";

float latitude, longitude, hdop, alt, hdop2;
int sats;

//LoRa
int isf = 0;
int oldisf = 0;
char *dr[6] = {"DR5", "DR4", "DR3", "DR2", "DR1", "DR0"};
char *sf[6] = {"SF7", "SF8", "SF9", "SF10", "SF11", "SF12"};
RTC_DATA_ATTR int iwm = 0;
char *workmode[7] = {"NACK", "ACK", "MAN", "LCM", "SSV", "OTAA", "SET"};
char buffer[256];
short length;
short rssi;
float snr;
char charsnr[5];
short gwcnt;
byte coords[9];
byte ncoords[1];
long sentMillis = 0;
long currentMillis = 0;
RTC_DATA_ATTR int iiv = 0;
long interval[5] = {15000, 30000, 45000, 60000, 120000};
char *ttext[5] = {"15s", "30s", "45s", "60s", "120s"};
RTC_DATA_ATTR int cnt = -1;
String txcnt;
int otaa = 0;
int otaaack = 0;

//Battery
int8_t BattLevel = 0;
#define FULL       (   3)

//SDCard
char filename[] = "/";
bool cardin = false;
bool sdwrite = false;
File dataFile;

//GPX
int year;
byte month, day, hour, minute, second;
char filename1[20];
char date1[22];
char filepath[20];

//SSV
char filename2[20];
char date2[22];
char filepath2[20];
bool firstssv = false;
bool lastssv = false;
String ssvresult = "DR ";

//M5Stack
bool dim = false;
RTC_DATA_ATTR bool powersave = false;

/* RootVar's for UI elements (note: not edit manually) */
String UIInputbox_6nssds = "";        //No GWs for LCR
String UITextbox_vimqus = "SF7";      //SpreadingFactor (B2)
String UITextbox_eq79hh46 = "NACK";   //Workmode  (B1)
String UITextbox_67ofwdh = "Dim";     //Dimming (B3)
String UIProgressbar_eymzer = "70";   //Progressbar RSSI
String UITextbox_859t1hi = "-130";    //RSSI
String UIInputbox_awnh87 = "inactive";//Status
String UITextbox_4t0l0bn = "0";		  //Stattelites
String UITextbox_q7sl3uo = "0";		  //HDOP
String UITextbox_403ohip = "0";		  //Battery Level
String UITextbox_olwwlae = "-20.00";  //SNR
String UITextbox_7mnuudb = "SNR";     //SNR

/* Function for layer default: */
void LayerFunction_default(String* rootVar) {
  /* UI Elements */
  UIInputbox(160, 58, 150, "default", "No of GWs", 0, &UIInputbox_6nssds);
  UITextbox(144, 214, 50, 20, 0x0000, "default", &UITextbox_vimqus);
  UITextbox(44, 215, 50, 20, 0x0000, "default", &UITextbox_eq79hh46);
  UITextbox(227, 215, 50, 20, 0x0000, "default", &UITextbox_67ofwdh);
  UIProgressbar(10, 144, 300, "default", "RSSI, dB", &UIProgressbar_eymzer);
  UITextbox(124, 142, 50, 20, 0x0000, "default", &UITextbox_859t1hi);
  UIInputbox(5, 58, 150, "default", "Status", 0, &UIInputbox_awnh87);
  UITextbox(40, 11, 25, 20, 0x0000, "default", &UITextbox_4t0l0bn);
  UITextbox(100, 11, 60, 20, 0x0000, "default", &UITextbox_q7sl3uo);
  UITextbox(270, 11, 50, 20, 0x0000, "default", &UITextbox_403ohip);
  UITextbox(249, 142, 70, 20, 0x0000, "default", &UITextbox_olwwlae);
  UITextbox(200, 142, 40, 20, 0x0000, "default", &UITextbox_7mnuudb);

  /* To open this layer use: */
  UILayer("default");
}

//Delay without delay
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {} while (millis() - start < ms);
}

//Settings for LoRaWAN
void initlora() {

  Serial.println("initlora");
  blora.init();

  if (powersave == false) {
    Serial.println("initlora no powersave. delay 1000");
    delay(1000);

    memset(buffer, 0, 256);
    blora.getVersion(buffer, 256, 1);
    Serial.println("lora modem version:");
    Serial.print(buffer);

    memset(buffer, 0, 256);
    blora.getId(buffer, 256, 1);
    Serial.println("lora id:");
    Serial.println(buffer);

    // void setId(char *DevAddr, char *DevEUI, char *AppEUI);
    //blora.setId("yourdeviceaddress", NULL, NULL);            //for ABP
    //blora.setId("ABP-yourdeviceaddress", "OTAA-yourdeviceEUI", "OTAA-yourAppEUI"); //for OTAA

    // setKey(char *NwkSKey, char *AppSKey, char *AppKey);
    blora.setKey(NULL, NULL, appKey);          //for ABP
    //blora.setKey("ABP-yourNetworkSKey", "ABP-yourappSKey", "OTAAyourAppKey);   //for OTAA

    blora.setDeviceMode(LWOTAA);
    blora.setDataRate(DR5, EU868);

    blora.setChannel(0, 868.1);
    blora.setChannel(1, 868.3);
    blora.setChannel(2, 868.5);
    blora.setChannel(3, 867.1);
    blora.setChannel(4, 867.3);
    blora.setChannel(5, 867.5);
    blora.setChannel(6, 867.7);
    blora.setChannel(7, 867.9);

    blora.setReceiveWindowFirst(0, 868.1);
    blora.setReceiveWindowSecond(869.525, DR3);

    blora.setPower(14);
    blora.setPort(1);
    blora.setAdaptiveDataRate(false);
    blora.setDutyCycle(true);
    blora.setDeviceLowPower();
  }
  Serial.println("initlora done");
}

//Settings for LoRaWAN ABP
void initloraabp() {
  Serial.println("initloraabp");
  blora.sendDevicePing();
  blora.setDeviceMode(LWABP);
  blora.setAdaptiveDataRate(false);
  blora.setDutyCycle(true);
  otaa = 0;
  cnt = -1;
  blora.setDeviceLowPower();
}

//Settings for LoRaWAN OTAA
void initloraotaa() {
  Serial.println("initloraotaa");
  blora.sendDevicePing();
  blora.setDeviceMode(LWOTAA);
  blora.setAdaptiveDataRate(true);
  blora.setDutyCycle(false);
  UISet(&UIInputbox_awnh87, "Joining");
  while (!blora.setOTAAJoin(JOIN, 10));
  UISet(&UIInputbox_awnh87, "Joined");
  blora.setDutyCycle(true);
  otaa = 1;
  cnt = -1;
  blora.setDeviceLowPower();
}

//Send data using LoRaWAN
void sendobject() {
  bool result = false;

  int32_t lat = latitude * 10000;
  int32_t lon = longitude * 10000;
  int16_t altitude = alt * 100;
  int8_t hdopGPS = hdop / 10;

  coords[0] = lat;
  coords[1] = lat >> 8;
  coords[2] = lat >> 16;

  coords[3] = lon;
  coords[4] = lon >> 8;
  coords[5] = lon >> 16;

  coords[6] = altitude;
  coords[7] = altitude >> 8;

  coords[8] = hdopGPS;

  sentMillis = millis();

  if (iwm == 0) {
    UISet(&UIInputbox_awnh87, "Sending");
    blora.sendDevicePing();

    if (oldisf != isf) {
      if (isf == 0) {
        blora.setDataRate(DR5, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 1) {
        blora.setDataRate(DR4, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 2) {
        blora.setDataRate(DR3, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 3) {
        blora.setDataRate(DR2, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 4) {
        blora.setDataRate(DR1, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 5) {
        blora.setDataRate(DR0, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      }
    }
    result = blora.transferPacket(ncoords, sizeof(ncoords), 5);
    if (result == true) {
      cnt++;
      txcnt = String("Sent " + String(cnt));
      UISet(&UIInputbox_awnh87, txcnt);
    } 
    else if (blora.dutycycle == true) {
      UISet(&UIInputbox_awnh87, "DutyCycle");
    } 
    else {
      UISet(&UIInputbox_awnh87, "Error");
    }
  } 
  else if ((iwm == 1) || (iwm == 2)) {
    UISet(&UIInputbox_awnh87, "ACK");
    blora.sendDevicePing();

    if (isf == 0) {
      blora.setDataRate(DR5, EU868);
      oldisf = 6;
      cnt = -1;
    } else if (isf == 1) {
      blora.setDataRate(DR4, EU868);
      oldisf = 6;
      cnt = -1;
    } else if (isf == 2) {
      blora.setDataRate(DR3, EU868);
      oldisf = 6;
      cnt = -1;
    } else if (isf == 3) {
      blora.setDataRate(DR2, EU868);
      oldisf = 6;
      cnt = -1;
    } else if (isf == 4) {
      blora.setDataRate(DR1, EU868);
      oldisf = 6;
      cnt = -1;
    } else if (isf == 5) {
      blora.setDataRate(DR0, EU868);
      oldisf = 6;
      cnt = -1;
    }


    result = blora.transferPacketWithConfirmed(ncoords, sizeof(ncoords), 5);

    if (result == true) {
      cnt++;

      UISet(&UIInputbox_awnh87, "ACK OK");

      short length;
      short rssi;
      float snr;
      char charsnr[5];
      short gwcnt;

      memset(buffer, 0, 256);
      length = blora.receivePacket(buffer, 256, &rssi, &snr, &gwcnt);

      dtostrf(snr, 5, 1, charsnr);

      UISet(&UIProgressbar_eymzer, rssi + 130);
      UISet(&UITextbox_859t1hi, rssi);
      UISet(&UITextbox_olwwlae, charsnr);
    } else {
      UISet(&UIInputbox_awnh87, "Error");
    }
  } else if (iwm == 3) {

    UISet(&UIInputbox_awnh87, "LCR");
    blora.sendDevicePing();

    if (oldisf != isf) {
      if (isf == 0) {
        blora.setDataRate(DR5, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 1) {
        blora.setDataRate(DR4, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 2) {
        blora.setDataRate(DR3, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 3) {
        blora.setDataRate(DR2, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 4) {
        blora.setDataRate(DR1, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      } else if (isf == 5) {
        blora.setDataRate(DR0, EU868);
        blora.setAdaptiveDataRate(false);
        blora.setDutyCycle(true);
        oldisf = isf;
        cnt = -1;
      }
    }

    result = blora.transferPacketLinkCheckReq(5);

    if (result == true) {
      cnt++;

      UISet(&UIInputbox_awnh87, "LCR OK");

      short length;
      short rssi;
      float snr;
      char charsnr[5];
      short gwcnt;

      memset(buffer, 0, 256);
      length = blora.receivePacket(buffer, 256, &rssi, &snr, &gwcnt);

      dtostrf(snr, 5, 1, charsnr);

      UISet(&UIProgressbar_eymzer, rssi + 130);
      UISet(&UITextbox_859t1hi, rssi);
      UISet(&UITextbox_olwwlae, charsnr);
      UISet(&UIInputbox_6nssds, gwcnt);

    } else if (blora.dutycycle == true) {
      UISet(&UIInputbox_awnh87, "DutyCycle");
    } else {
      UISet(&UIInputbox_awnh87, "Error");
    }
  }
  blora.setDeviceLowPower();
}

void sendobjectotaa() {

  bool result = false;

  int32_t lat = latitude * 10000;
  int32_t lon = longitude * 10000;
  int16_t altitude = alt * 100;
  int8_t hdopGPS = hdop / 10;

  coords[0] = lat;
  coords[1] = lat >> 8;
  coords[2] = lat >> 16;

  coords[3] = lon;
  coords[4] = lon >> 8;
  coords[5] = lon >> 16;

  coords[6] = altitude;
  coords[7] = altitude >> 8;

  coords[8] = hdopGPS;

  sentMillis = millis();

  blora.sendDevicePing();

  UISet(&UIInputbox_awnh87, "Sending");

  if (otaaack == 0) {
    result = blora.transferPacket(ncoords, sizeof(ncoords), 5);
  } else if (otaaack == 1) {
    result = blora.transferPacketWithConfirmed(ncoords, sizeof(ncoords), 5);
  }

  if (result == true) {
    cnt++;
    txcnt = String("Sent " + String(cnt));
    UISet(&UIInputbox_awnh87, txcnt);

    short length;
    short rssi;
    float snr;
    char charsnr[5];
    short gwcnt;

    memset(buffer, 0, 256);
    length = blora.receivePacket(buffer, 256, &rssi, &snr, &gwcnt);

    dtostrf(snr, 5, 1, charsnr);
    if (rssi >= -200) {
      UISet(&UIProgressbar_eymzer, rssi + 130);
      UISet(&UITextbox_859t1hi, rssi);
      UISet(&UITextbox_olwwlae, charsnr);

    }
  } else if (blora.dutycycle == true) {
    UISet(&UIInputbox_awnh87, "DutyCycle");
  } else {
    UISet(&UIInputbox_awnh87, "Error");
  }
  blora.setDeviceLowPower();
}

//initial setup
void setup() {
  /* Prepare M5STACK */
  M5.begin();
  M5.Power.begin();
  M5.Lcd.setBrightness(50);
  //M5.Lcd.drawBitmap(0, 0, 320, 240, (uint16_t *)imgName);
  M5.Lcd.drawBitmap(0, 0, 320, 240, (uint16_t *)gImage_logoM5);
  initlora();

  /* Prepare UI */
  UIBegin();
  LayerFunction_default(0);

  if (SD.exists(filename)) {
    M5.Lcd.drawBitmap(200, 5, 24, 24, (uint16_t *)ICON_22_24);
    cardin = true;
  }

  //Prepare UI for iwm = 0
  UISet(&UITextbox_vimqus, sf[isf]);
  UIDisable(true, &UIProgressbar_eymzer);
  UIDisable(true, &UITextbox_859t1hi);
  UIDisable(true, &UITextbox_olwwlae);
  UIDisable(true, &UIInputbox_6nssds);
  UIDisable(true, &UITextbox_7mnuudb);
  UIDisable(false, &UIInputbox_awnh87);

  Serial.println("Started");

  if (powersave == true) {
    smartDelay(1000);
    sendobject();
    esp_sleep_enable_timer_wakeup(interval[iiv] * 1000);
    esp_deep_sleep_start();
  }

}

void loop() {

  //update button status
  if (M5.BtnA.wasPressed()) {
    if (iwm == 6) {
      iwm = 0;
      UISet(&UITextbox_eq79hh46, workmode[iwm]);
      if (otaa == 1) {
        initloraabp();
      }
    } else if (iwm == 3) {
      iwm++;
      UISet(&UITextbox_eq79hh46, workmode[iwm]);
    } else {
      iwm++; iwm++; // Weirdness after removal of GPS code.
      UISet(&UITextbox_eq79hh46, workmode[iwm]);
    }

    if (iwm == 0) {
      UISet(&UITextbox_vimqus, sf[isf]);
      UIDisable(false, &UIInputbox_awnh87);
      UISet(&UITextbox_67ofwdh, "Dim");
    } 
    else if (iwm == 1) {
      UIDisable(false, &UIProgressbar_eymzer);
      UIDisable(false, &UITextbox_859t1hi);
      UIDisable(false, &UITextbox_olwwlae);
      UIDisable(false, &UITextbox_7mnuudb);
    } 
    else if (iwm == 2) {
      UISet(&UITextbox_67ofwdh, "Send");
    } 
    else if (iwm == 3) {
      UIDisable(false, &UIInputbox_6nssds);
    } 
    else if (iwm == 4) {
      UIDisable(true, &UIProgressbar_eymzer);
      UIDisable(true, &UITextbox_859t1hi);
      UIDisable(true, &UITextbox_olwwlae);
      UIDisable(true, &UIInputbox_6nssds);
      UIDisable(true, &UITextbox_7mnuudb);
    } 
    else if (iwm == 5) {
      UIDisable(true, &UIInputbox_6nssds);
      UIDisable(true, &UITextbox_7mnuudb);
      UISet(&UITextbox_vimqus, "Join");
      UISet(&UITextbox_67ofwdh, "NACK");
      UIDisable(false, &UIProgressbar_eymzer);
      UIDisable(false, &UITextbox_859t1hi);
      UIDisable(false, &UITextbox_olwwlae);
      //strip.SetPixelColor(7, off);
    } 
    else if (iwm == 6) {
      UISet(&UITextbox_vimqus, ttext[iiv]);
      UIDisable(true, &UIInputbox_awnh87);
      UIDisable(true, &UIProgressbar_eymzer);
      UIDisable(true, &UITextbox_859t1hi);
      UIDisable(true, &UITextbox_olwwlae);
      UISet(&UITextbox_67ofwdh, "PS");
    }
  }

  if (M5.BtnB.wasPressed()) {
    if (isf == 5) {
      isf = 0;
      UISet(&UITextbox_vimqus, sf[isf]);
    } 
    else if (iwm == 5 && otaa == 0) {
      initloraotaa(); //OTAA Join
      UISet(&UITextbox_vimqus, "Send");
    } 
    else if (iwm == 5 && otaa == 1) {
      sendobjectotaa(); //Manual send
    } 
    else if (iwm == 6) {
      if (iiv == 4) {
        iiv = 0;
        UISet(&UITextbox_vimqus, ttext[iiv]);
      } else {
        iiv++;
        UISet(&UITextbox_vimqus, ttext[iiv]);
      }
    }
    else {
      isf++;
      UISet(&UITextbox_vimqus, sf[isf]);
    }
  }

  if (M5.BtnC.wasPressed()) {
    if (iwm < 2 && dim == false) {
      dim = true;
      M5.Lcd.setBrightness(0);
    } 
    else if (iwm < 2 && dim == true) {
      dim = false;
      M5.Lcd.setBrightness(70);
    } 
    else if (iwm == 4) {
    }
    else if (iwm == 5 && otaaack == 0) {
      otaaack = 1;
      UISet(&UITextbox_67ofwdh, "ACK");
    } 
    else if (iwm == 5 && otaaack == 1) {
      otaaack = 0;
      UISet(&UITextbox_67ofwdh, "NACK");
    } 
    else if (iwm == 6 && powersave == false) {
      powersave = true;
      iwm = 0;
    } 
    else if (iwm == 6 && powersave == true) {
      powersave = false;
    } 
    else if (iwm > 1) {
      sendobject();
    }
  }



  //Battery Status
  if (M5.Power.isCharging() == true) {
    M5.Lcd.drawBitmap(240, 5, 24, 24, (uint16_t *)ICON_40_24);
  }

  if (M5.Power.isChargeFull() == true) {
    UISet(&UITextbox_403ohip, "Full");
  }
  else {
    BattLevel = M5.Power.getBatteryLevel();
    String strbattlevel = String(BattLevel);
    strbattlevel = String(strbattlevel + "%");
    UISet(&UITextbox_403ohip, strbattlevel);
  }

  //Sending intervall
  currentMillis = millis();
  if ((currentMillis - sentMillis > interval[iiv]) && iwm < 2) {
    sendobject();
  }

  if ((currentMillis - sentMillis > interval[iiv]) && iwm == 5 && otaa == 1) {
    sendobjectotaa();
  }

  //light sleep timer
  if (powersave == true) {
    esp_sleep_enable_timer_wakeup(interval[iiv] * 1000);
    esp_deep_sleep_start();
  }

  //used to deflicker the display, more possible, but with less reactive buttons
  smartDelay(200);

  M5.update();
}
