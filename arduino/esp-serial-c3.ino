// $GNRMC Time, date, position, course and speed data.
// $--RMC,hhmmss.sss,<Valid - A , Warning -V>,Latitude,N/S indicator ,Longitude ,E/W Indicator ,SOG,COG,UTC Date ,Mode indicator <N -invalid, A -Autonomous mode,D -Differential mode, E  Estimated (dead reckoning) mode>,,v*hh<CR><LF>

// $GNGGA Time, position, and fix related data of the receiver.
// $--GGA,hhmmss.ss,llll.lll,a,yyyyy.yyy,a,x,uu,v.v,w.w,M,x.x,M,,zzzz*hh<CR><LF>
//  x GPS quality indicator need to be > 0

// https://www.qso.com.ar/datasheets/Receptores%20GNSS-GPS/NMEA_Format_v0.1.pdf


#include <HardwareSerial.h>

#include <esp_now.h>
#include <WiFi.h>
// REPLACE WITH YOUR ESP RECEIVER'S MAC ADDRESS
uint8_t broadcastAddress1[] = { 0x34, 0x85, 0x18, 0x7B, 0x6C, 0x64 };
uint8_t broadcastAddress2[] = { 0x84, 0xFC, 0xE6, 0x70, 0x61, 0xEC };
// uint8_t broadcastAddress3[] = { 0x34, 0x85, 0x18, 0x7B, 0x6C, 0x64 };

esp_now_peer_info_t peerInfo;

#define MYPORT_TX 21
#define MYPORT_RX 20


const PROGMEM uint8_t Navrate10hz[] = { 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12 };
const PROGMEM uint8_t ClearConfig[] = { 0xB5, 0x62, 0x06, 0x09, 0x0D, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x01, 0x19, 0x98 };
const PROGMEM uint8_t GPGLLOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B };
const PROGMEM uint8_t GPGSVOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39 };
const PROGMEM uint8_t GPVTGOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47 };
const PROGMEM uint8_t GPGSAOff[] = { 0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32 };

int fixPin = D10;
int batteryPin = D0;
int gpsPowerPin =D8;

bool pos_fix = false;

HardwareSerial GPS(1);

// needs to be clarified lenght
const byte numChars = 128;

char receivedChars[numChars];

boolean newData = false;

void IRAM_ATTR fonction_ISR() {
  pos_fix = true;
}

struct toSend {
  float voltage;
  float time;
  float lat;
  char n_s;
  float lon;
  char e_w;
  float SOG;
  float COG;
  int UTC_Date;
};


struct GNRMC {
  float time;
  bool valid;
  float lat;
  char n_s;
  float lon;
  char e_w;
  float SOG;
  float COG;
  int UTC_Date;
  char mode;
};
struct GNGGA {
  float time;
  float lat;
  char n_s;
  float lon;
  char e_w;
  char valid;
};
toSend data;
GNRMC v_rmc;
GNGGA v_gga;

float batteryVoltage;

void setup() {

  pinMode(fixPin, INPUT);
  pinMode(gpsPowerPin, OUTPUT);
  esp_sleep_enable_timer_wakeup(100000000);

  digitalWrite(gpsPowerPin, HIGH);

    int tempValue = analogRead(batteryPin);
    if (tempValue*1.4/1000 < 3.2 ){
      digitalWrite(gpsPowerPin, LOW);
      delay(10);
      esp_deep_sleep_start();

    }



  Serial.begin(115200);  // put your setup code here, to run once:
  GPS.begin(38400, SERIAL_8N1, MYPORT_RX, MYPORT_TX);
  delay(5000);
  GPS_SendConfig(ClearConfig, 21);
  GPS_SendConfig(GPGLLOff, 16);
  GPS_SendConfig(GPGSVOff, 16);
  GPS_SendConfig(GPVTGOff, 16);
  GPS_SendConfig(GPGSAOff, 16);
  GPS_SendConfig(Navrate10hz, 14);
  attachInterrupt(fixPin, fonction_ISR, RISING);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() == ESP_OK) {
    Serial.println("Initializing ESP-NOW sucessfuly");
  } else {
    Serial.println("ESPNow Init Failed");
    ESP.restart();
  }
  // register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  // register first peer
  memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }
  memcpy(peerInfo.peer_addr, broadcastAddress2, 6);
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
  }
  delay(1000);
}

void loop() {
  while (!pos_fix) {
    // position_fix();
    if (pos_fix == true) {
      detachInterrupt(fixPin);
      break;
    }
    // temporrary out of loop
    // break;
    delay(1000);
  }

  while (true) {
    int tempValue;
    tempValue = analogRead(batteryPin);
    if (tempValue > 0 ){
       batteryVoltage =  tempValue*1.4/1000 ;
    }


    recvWithStartEndMarkers();
    data.voltage = batteryVoltage;
    if (batteryVoltage < 3.30){
      digitalWrite(gpsPowerPin, LOW);
      esp_deep_sleep_start();

    }

    if (strlen(receivedChars) > 0) {
      processingSentence();
      esp_err_t result = esp_now_send(0, (uint8_t *)&data, sizeof(toSend));
      // Serial.println("************");
      // Serial.println(batteryVoltage);
      // Serial.print(data.n_s);
      // Serial.println(data.lat);
      // Serial.print(data.e_w);
      // Serial.println(data.lon);
      // Serial.println(data.SOG);
      // Serial.println(data.COG);
      // Serial.println(data.UTC_Date);

      // esp_err_t result = esp_now_send(0, (uint8_t *)&v_rmc, sizeof(GNRMC));
      // if (result == ESP_OK) {
      //   Serial.println("Sent with success");
      // } else {
      //   Serial.println("Error sending the data");
      // }
    }

    newData = false;

    // delay(1000);
  }
}




void GPS_SendConfig(const uint8_t *Progmem_ptr, uint8_t arraysize) {
  uint8_t byteread, index;

  Serial.print(F("GPSSend  "));

  for (index = 0; index < arraysize; index++) {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    if (byteread < 0x10) {
      Serial.print(F("0"));
    }
    Serial.print(byteread, HEX);
    Serial.print(F(" "));
  }

  Serial.println();
  Progmem_ptr = Progmem_ptr - arraysize;  //set Progmem_ptr back to start

  for (index = 0; index < arraysize; index++) {
    byteread = pgm_read_byte_near(Progmem_ptr++);
    GPS.write(byteread);
  }
  delay(1000);
}


void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '$';
  char endMarker = '\n';
  char rc;
  receivedChars[0] = '\0';

  while (GPS.available() && newData == false) {
    rc = GPS.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      } else {
        receivedChars[ndx] = '\0';  // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
  if (!checkSentance()) {
    receivedChars[0] = '\0';
    newData = false;
  }
}

bool checkSentance() {
  byte xorTemp;
  int len = strlen(receivedChars);
  char end[2];
  end[0] = receivedChars[len - 3];
  end[1] = receivedChars[len - 2];
  end[2] = '\0';
  xorTemp = byte(receivedChars[0]);
  for (int i = 1; i < len - 4; i++) {
    xorTemp ^= byte(receivedChars[i]);
  }
  String aStringObject = String(end);
  String bStringObject = String(xorTemp, HEX);
  bStringObject.toUpperCase();
  if (bStringObject == aStringObject) {
    return true;
  }
  return false;
}

void processingSentence() {
  int len = strlen(receivedChars);
  int tempIndex = 0;
  char tempArray[20];
  int sentanceNumber = 0;
  int messageType = 0;
  String name = " ";
  GNRMC rmc;
  GNGGA gga;
  float time;


  for (int i = 0; i < len - 1; i++) {


    if (receivedChars[i] != ',') {
      tempArray[tempIndex] = receivedChars[i];
      tempIndex++;
    } else if (receivedChars[i] == ',') {

      tempArray[tempIndex] = '\0';

      switch (sentanceNumber) {
        case 0:
          name = String(tempArray);
          if (name == "GNRMC") {
            messageType = 1;
          } else if (name == "GNGGA") {
            messageType = 2;
          } else {
            return;
          }
          break;
        case 1:
          time = atof(tempArray);
          if (messageType == 1) {
            rmc.time = time;
          } else {
            gga.time = time;
          }
          break;

        case 2:
          if (messageType == 1) {
            if (tempArray[0] == 'A') {
              rmc.valid = true;
            } else {
              return;
            }
          } else {
            gga.lat = atof(tempArray);
          }
          break;
        case 3:
          if (messageType == 1) {
            rmc.lat = atof(tempArray);
          } else {
            gga.n_s = tempArray[0];
          }
          break;
        case 4:
          if (messageType == 1) {
            rmc.n_s = tempArray[0];
          } else {
            gga.lon = atof(tempArray);
          }
          break;
        case 5:
          if (messageType == 1) {
            rmc.lon = atof(tempArray);
          } else {
            gga.e_w = tempArray[0];
          }
          break;
        case 6:
          if (messageType == 1) {
            rmc.e_w = tempArray[0];
          } else {
            char valid = tempArray[0];
            if (valid != '0') {
              gga.valid = valid;
            } else {
              return;
            }
          }

          break;
        case 7:
          if (messageType == 1) {
            rmc.SOG = atof(tempArray);
          }
          break;
        case 8:
          if (messageType == 1) {
            rmc.COG = atof(tempArray);
          }
          break;
        case 9:
          if (messageType == 1) {
            rmc.UTC_Date = atoi(tempArray);
          }
          break;
        case 12:
          if (messageType == 1) {
            char mode = tempArray[0];
            if (mode == 'N') { return; }
            rmc.mode = mode;
          }
          break;
      }
      sentanceNumber++;
      tempArray[0] = '\0';

      // Serial.println(tempArray);
      tempIndex = 0;
    }
  }
  if (messageType == 1) {
    v_rmc = rmc;
    data.time = v_rmc.time;
      data.lat = v_rmc.lat;
      data.n_s = v_rmc.n_s;
      data.lon = v_rmc.lon;
      data.e_w = v_rmc.e_w;
  } else if (messageType == 2) {
    v_gga = gga;
    data.time = v_gga.time;
    data.lat = v_gga.lat;
    data.n_s = v_gga.n_s;
    data.lon = v_gga.lon;
    data.e_w = v_gga.e_w;
  }
  data.SOG = v_rmc.SOG;
  data.COG = v_rmc.COG;
  data.UTC_Date = v_rmc.UTC_Date;
}

