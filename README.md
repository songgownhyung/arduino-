/*  Smart Plant + I2C LCD (Bus-Recover + Auto Failsafe + Bluetooth SPP)
 *  - TX: iot01_plant_<soilData>_<lightData>_<tempData>_<humiData>  (한 줄)
 *  - FAST: I2C 100kHz / LCD 250ms / 펌프보류 100ms
 *  - SAFE: I2C 25kHz  / LCD 1000ms / 펌프보류 500ms
 *  - LCD 연속 실패 2회 → FAST→SAFE
 *  - I2C 버스 복구(SCL 9펄스 + STOP)
 *  - HC-05 Bluetooth SPP(9600bps), 줄 단위 명령
 */

#define PERFORMANCE_MODE 1
#define VERBOSE_SERIAL  1

#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <DHT.h>
#include <SoftwareSerial.h>
#include <string.h>

/* ─ 로그 매크로 ─ */
#if VERBOSE_SERIAL
  #define VLOG(stmt) do { stmt; } while (0)
#else
  #define VLOG(stmt) do {} while (0)
#endif

/* ─ 고정 홈ID ─ */
static const char* HOME_ID = "iot01";

/* ─ LCD / I2C ─ */
#define LCD_COLS 16
#define LCD_ROWS 2
#define SDA_PIN A4
#define SCL_PIN A5

/* ─ Bluetooth(HC-05) ─ */
#define BT_RX_PIN 10   // HC-05 TXD → D10
#define BT_TX_PIN 11   // (분압) D11 → HC-05 RXD
#define BT_BAUD   9600
SoftwareSerial BT(BT_RX_PIN, BT_TX_PIN); // RX, TX

/* 런타임 프로필(FAST/SAFE) */
uint32_t     i2cClockHz;
unsigned long lcdUpdateMs;
unsigned long lcdBlockAfterPumpMs;
unsigned long lcdForceReinitMs;
uint8_t      currentProfile = (PERFORMANCE_MODE ? 1 : 0); // 1=FAST, 0=SAFE

void applyProfile(uint8_t fast){
  currentProfile = fast ? 1 : 0;
  if (fast){
    i2cClockHz          = 100000UL;
    lcdUpdateMs         = 250;
    lcdBlockAfterPumpMs = 100;
    lcdForceReinitMs    = 180000; // 3분
  }else{
    i2cClockHz          = 25000UL;
    lcdUpdateMs         = 1000;
    lcdBlockAfterPumpMs = 500;
    lcdForceReinitMs    = 60000;  // 1분
  }
  Wire.setClock(i2cClockHz);
  VLOG( Serial.print(F("[PROFILE] ")); Serial.println(fast ? F("FAST") : F("SAFE")); );
}

hd44780_I2Cexp lcd;
bool LCD_OK = false;

/* ─ Pins ─ */
#define DHTPIN   4
#define DHTTYPE  DHT11
#define SOIL_AIN A0
#define LDR_AIN  A1
#define PUMP_PIN 7
#define FAN_PIN  6
#define LED_PIN  5

/* ─ Relay polarity ─ */
#define RELAY_ACTIVE_LOW 1

/* ─ DHT ─ */
#define DHT_WARMUP_READS   3
#define DHT_RETRIES        3
#define DHT_RETRY_GAP_MS   60
const float T_OFFSET = 0.0, H_OFFSET = 0.0;
DHT dht(DHTPIN, DHTTYPE);

/* ─ Intervals ─ */
const unsigned long ANALOG_INTERVAL_MS  = (PERFORMANCE_MODE ? 150 : 300);
const float         EMA_ALPHA           = (PERFORMANCE_MODE ? 0.50f : 0.30f);
const unsigned long DHT_INTERVAL_MS     = 2000;
const unsigned long HEARTBEAT_MS        = 1000;
const unsigned long LCD_DOWN_RETRY_MS   = 2000;
const unsigned long TELEMETRY_MS        = 1000;  // 전송 주기(1초)

/* ─ Calibration ─ */
int SOIL_DRY_ADC = 1020;   // 공기
int SOIL_WET_ADC = 330;    // 젖음

/* ─ Pump thresholds (soil %) ─ */
int SOIL_PUMP_ON_BELOW  = 35;  // ≤ ON
int SOIL_PUMP_OFF_ABOVE = 55;  // ≥ OFF

/* ─ State ─ */
unsigned long lastAnalogRead=0, lastDHTRead=0, lastLCD=0;
unsigned long lastPumpSwitch=0, lastLCDInit=0, lastHeartbeat=0, lastTx=0;
bool   pumpOn=false;
bool   manualPumpHold=false;     // 수동 강제 유지
float  soilPctEMA=0.0f, lightPctEMA=0.0f, t=NAN, h=NAN;
uint8_t lcdFailStreak = 0;

/* ─ Utils ─ */
int clamp(int v, int lo, int hi){ return v<lo?lo:(v>hi?hi:v); }

void PUMP_WRITE(bool on){
  digitalWrite(PUMP_PIN, (RELAY_ACTIVE_LOW ? (on ? LOW : HIGH) : (on ? HIGH : LOW)));
  lastPumpSwitch = millis();
  pumpOn = on;
  delayMicroseconds(300);
}

/* ADC → % 변환 */
int soilPercentFromAdc(int adc){
  int v = clamp(adc, min(SOIL_WET_ADC, SOIL_DRY_ADC), max(SOIL_WET_ADC, SOIL_DRY_ADC));
  long span = (long)SOIL_DRY_ADC - (long)SOIL_WET_ADC;
  if (!span) return 0;
  long num  = (long)SOIL_DRY_ADC - v;
  long pctL = (num * 100L) / span;
  return clamp((int)pctL, 0, 100);
}
int lightPercentFromAdc(int adc){ int pct=(adc*100L)/1023L; return clamp(pct,0,100); }

/* 중앙값 필터 */
int analogReadMedian(uint8_t pin, uint8_t n=5){
  if(n<1) n=1; if(n>15) n=15; int buf[15];
  for(uint8_t i=0;i<n;++i){ buf[i]=analogRead(pin); delayMicroseconds(200); }
  for(uint8_t i=1;i<n;++i){ int k=buf[i], j=i; while(j>0 && buf[j-1]>k){ buf[j]=buf[j-1]; --j; } buf[j]=k; }
  return buf[n/2];
}

/* DHT 견고 판독 */
bool dhtValid(float tt, float hh){
  return !isnan(tt)&&!isnan(hh)&&(tt>=0&&tt<=50)&&(hh>=20&&hh<=90);
}
bool readDHTRobust(float &tC, float &hRH){
  float tt[3]={NAN,NAN,NAN}, hh[3]={NAN,NAN,NAN}; int n=0;
  for(int i=0;i<DHT_RETRIES;++i){
    float _h=dht.readHumidity(), _t=dht.readTemperature();
    if(dhtValid(_t,_h)){ tt[n]=_t; hh[n]=_h; n++; }
    delay(DHT_RETRY_GAP_MS);
  }
  if(n==0) return false;
  auto med=[](float a,float b,float c)->float{
    if(isnan(b)) return a; if(isnan(c)) return (a+b)/2.0;
    if((a<=b&&b<=c)||(c<=b&&b<=a)) return b;
    if((b<=a&&a<=c)||(c<=a&&a<=b)) return a;
    return c;
  };
  tC=(n==1)?tt[0]:(n==2?(tt[0]+tt[1])/2.0:med(tt[0],tt[1],tt[2]));
  hRH=(n==1)?hh[0]:(n==2?(hh[0]+hh[1])/2.0:med(hh[0],hh[1],hh[2]));
  tC+=T_OFFSET; hRH+=H_OFFSET; return true;
}

/* ─ I2C 버스 복구 ─ */
void i2c_bus_recover(){
  pinMode(SDA_PIN, INPUT_PULLUP); 
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(2);

  if (digitalRead(SDA_PIN) == LOW) {
    pinMode(SCL_PIN, OUTPUT);
    for (uint8_t i=0; i<9 && digitalRead(SDA_PIN)==LOW; ++i) {
      digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
      digitalWrite(SCL_PIN, LOW ); delayMicroseconds(5);
    }
  }
  pinMode(SDA_PIN, OUTPUT);
  digitalWrite(SDA_PIN, LOW);  delayMicroseconds(5);
  pinMode(SCL_PIN, OUTPUT);
  digitalWrite(SCL_PIN, HIGH); delayMicroseconds(5);
  digitalWrite(SDA_PIN, HIGH); delayMicroseconds(5);
  pinMode(SDA_PIN, INPUT_PULLUP);
  pinMode(SCL_PIN, INPUT_PULLUP);
  delay(2);
}

/* ─ LCD helpers ─ */
void lcdPrintLine(uint8_t row, const char* s){
  if(!LCD_OK) return;
  lcd.setCursor(0,row);
  for(int i=0;i<LCD_COLS;i++){ lcd.print( (i<(int)strlen(s)) ? s[i] : ' ' ); }
}
bool lcd_try_begin(uint8_t cols, uint8_t rows){
  int st = lcd.begin(cols, rows);
  if (st == 0) { lcd.backlight(); return true; }
  VLOG( Serial.print(F("[LCD] init err: ")); Serial.println(st); );
  return false;
}

/* 공용: 버스 복구 + Wire 재설정 + LCD init (실패 카운터 관리 + 자동강등) */
bool lcd_recover_and_init(){
  i2c_bus_recover();
  Wire.begin();
  Wire.setClock(i2cClockHz);
  #if defined(TWOWIRE_HAS_TIMEOUT) || defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true); // 25ms timeout→TWI reset
  #endif

  bool ok = lcd_try_begin(LCD_COLS, LCD_ROWS);
  if (ok){
    LCD_OK = true;
    lcdFailStreak = 0;
  }else{
    LCD_OK = false;
    lcdFailStreak++;
    if (currentProfile==1 && lcdFailStreak >= 2){
      VLOG( Serial.println(F("[FAILSAFE] Switch to SAFE profile")); );
      applyProfile(0);
    }
  }
  return ok;
}

void lcdReinitIfDue(){
  unsigned long now = millis();
  if(now - lastLCDInit >= lcdForceReinitMs){
    lastLCDInit = now;
    lcd_recover_and_init();
  }
}

/* ─ 공통 송신(시리얼+블루투스) ─ */
void sendLineAll(const char* s){
  Serial.println(s);
  BT.println(s);
}

/* ─ 한 줄 포맷용: 소수 1자리 문자열 변환 + 앞 공백 제거 ─ */
static void fmt1(float v, char* out){
  if (isnan(v)) v = 0.0f;            // NaN이면 0.0
  dtostrf(v, 0, 1, out);             // 소수 1자리
  size_t i = 0; while(out[i]==' ') ++i;
  if (i) memmove(out, out+i, strlen(out+i)+1);
}

/* ─ HELP/명령 화이트리스트 ─ */
void sendHelp(){
  sendLineAll("CMDS: GET | ON | OFF | FAST | SAFE | TH <on%> <off%> | CAL DRY <adc> | CAL WET <adc>");
}

/* ─ 상태 전송(한 줄): iot01_plant_<soil>_<light>_<temp>_<humi> ─ */
void sendTelemetry(){
  char s[16], l[16], tbuf[16], hbuf[16], line[96];

  // 순서: soilData → lightData → tempData → humiData
  fmt1(soilPctEMA,  s);
  fmt1(lightPctEMA, l);
  fmt1(t,           tbuf);
  fmt1(h,           hbuf);

  snprintf(line, sizeof(line), "%s_plant_%s_%s_%s_%s",
           HOME_ID, s, l, tbuf, hbuf);

  sendLineAll(line);  // Serial + BT 동시 전송
}

/* ─ 명령 파서(줄 단위) ─ */
void handleCommand(char* line){
  bool hasAlpha=false;
  for(char* p=line; *p; ++p){
    if ((*p>='a' && *p<='z') || (*p>='A' && *p<='Z')) { hasAlpha=true; break; }
  }
  if(!hasAlpha) return;

  for(char* p=line; *p; ++p) if(*p>='a' && *p<='z') *p = *p - 32;

  char* cmd = strtok(line, " \t\r\n");
  if(!cmd) return;

  if(!strcmp(cmd,"HELP")){ sendHelp(); return; }
  if(!strcmp(cmd,"GET")) { sendTelemetry(); return; }

  if(!strcmp(cmd,"ON")){
    manualPumpHold = true;
    if(!pumpOn) PUMP_WRITE(true);
    sendLineAll("OK PUMP=ON (HOLD)");
    return;
  }
  if(!strcmp(cmd,"OFF")){
    manualPumpHold = false;
    if(pumpOn) PUMP_WRITE(false);
    sendLineAll("OK PUMP=OFF (AUTO)");
    return;
  }
  if(!strcmp(cmd,"FAST")){
    applyProfile(1);
    sendLineAll("OK PROFILE=FAST");
    return;
  }
  if(!strcmp(cmd,"SAFE")){
    applyProfile(0);
    sendLineAll("OK PROFILE=SAFE");
    return;
  }
  if(!strcmp(cmd,"TH")){
    char* a = strtok(NULL," \t\r\n");
    char* b = strtok(NULL," \t\r\n");
    if(a && b){
      SOIL_PUMP_ON_BELOW  = atoi(a);
      SOIL_PUMP_OFF_ABOVE = atoi(b);
      char out[48]; snprintf(out,sizeof(out),"OK TH %d %d",SOIL_PUMP_ON_BELOW,SOIL_PUMP_OFF_ABOVE);
      sendLineAll(out);
    }else{
      sendLineAll("ERR TH usage: TH <on%> <off%>");
    }
    return;
  }
  if(!strcmp(cmd,"CAL")){
    char* kind = strtok(NULL," \t\r\n");
    char* val  = strtok(NULL," \t\r\n");
    if(kind && val){
      int v = atoi(val);
      if(!strcmp(kind,"DRY")) { SOIL_DRY_ADC = v; sendLineAll("OK CAL DRY"); }
      else if(!strcmp(kind,"WET")) { SOIL_WET_ADC = v; sendLineAll("OK CAL WET"); }
      else sendLineAll("ERR CAL kind (DRY|WET)");
    }else sendLineAll("ERR CAL usage: CAL DRY <adc> | CAL WET <adc>");
    return;
  }

  // Unknown → 무시
  return;
}

/* ─ 줄 수신기 ─ */
bool readLineFrom(Stream& s, char* buf, size_t bufsz){
  static size_t n = 0;
  bool gotLine = false;

  while (s.available()){
    int c = s.read();
    if (c < 0) break;

    if (c == '\r') continue;
    if (c == '\n'){
      buf[n] = '\0';
      bool nonspace=false;
      for(size_t i=0;i<n;++i){ if(buf[i]>' '){ nonspace=true; break; } }
      if(nonspace && n>=2) gotLine = true;
      n = 0;
      if (gotLine) return true;
      continue;
    }
    if (!((c >= 32 && c <= 126) || c == '\t')) continue;
    if (n < bufsz-1) buf[n++] = (char)c;
  }
  return false;
}

void setup(){
  Serial.begin(9600);
  BT.begin(BT_BAUD);
  delay(800);
  VLOG( Serial.println(F("[BOOT] start")); );
  BT.println(F("READY SmartPlant + BT (9600)"));

  pinMode(LED_BUILTIN, OUTPUT);

  Wire.begin();
  applyProfile(currentProfile);

  #if defined(TWOWIRE_HAS_TIMEOUT) || defined(WIRE_HAS_TIMEOUT)
    Wire.setWireTimeout(25000, true);
  #endif

  for (uint8_t i=0; i<3 && !LCD_OK; ++i) {
    if (!lcd_recover_and_init()) { delay(200); }
  }
  if (LCD_OK) {
    lcdPrintLine(0, "Smart Plant");
    lcdPrintLine(1, currentProfile ? "Mode: FAST" : "Mode: SAFE");
  } else {
    VLOG( Serial.println(F("[LCD] disabled (init failed)")); );
  }
  lastLCDInit = millis();

  dht.begin();
  for(int i=0;i<DHT_WARMUP_READS;++i){ dht.readHumidity(); dht.readTemperature(); delay(800); }

  pinMode(PUMP_PIN, OUTPUT);
  pinMode(FAN_PIN,  OUTPUT);
  pinMode(LED_PIN,  OUTPUT);
  PUMP_WRITE(false);
  digitalWrite(FAN_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  VLOG( Serial.println(F("[BOOT] ready")); );
}

void loop(){
  unsigned long now = millis();

  // heartbeat
  if (now - lastHeartbeat >= HEARTBEAT_MS){
    lastHeartbeat = now;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }

  // 1) soil/light
  if(now - lastAnalogRead >= ANALOG_INTERVAL_MS){
    lastAnalogRead = now;

    int soilAdc = analogReadMedian(SOIL_AIN);
    int ldrAdc  = analogReadMedian(LDR_AIN);

    int soilPctRaw  = soilPercentFromAdc(soilAdc);
    int lightPctRaw = lightPercentFromAdc(ldrAdc);

    soilPctEMA  = (EMA_ALPHA*soilPctRaw)  + (1.0f-EMA_ALPHA)*soilPctEMA;
    lightPctEMA = (EMA_ALPHA*lightPctRaw) + (1.0f-EMA_ALPHA)*lightPctEMA;

    int soilPct  = (int)(soilPctEMA  + 0.5f);
    int lightPct = (int)(lightPctEMA + 0.5f);

    // pump control (수동 HOLD 아닐 때만)
    if(!manualPumpHold){
      if(!pumpOn && soilPct <= SOIL_PUMP_ON_BELOW){
        PUMP_WRITE(true); VLOG( Serial.println(F("[PUMP] ON")); );
      } else if(pumpOn && soilPct >= SOIL_PUMP_OFF_ABOVE){
        PUMP_WRITE(false); VLOG( Serial.println(F("[PUMP] OFF")); );
      }
    }

    // aux
    if(!isnan(t)) digitalWrite(FAN_PIN, (t>=30.0)?HIGH:LOW);
    digitalWrite(LED_PIN, (lightPct<=20)?HIGH:LOW);

    VLOG(
      Serial.print(F("Soil="));  Serial.print(soilPct);  Serial.print(F("%  "));
      Serial.print(F("Light=")); Serial.print(lightPct);  Serial.print(F("%  "));
      Serial.print(F("Temp="));  if(!isnan(t)) Serial.print(t,1); else Serial.print(F("--"));
      Serial.print(F("C  Hum=")); if(!isnan(h)) Serial.print(h,1); else Serial.print(F("--"));
      Serial.println(F("%"));
    );
  }

  // 2) DHT
  if(now - lastDHTRead >= DHT_INTERVAL_MS){
    lastDHTRead = now;
    float tC=NAN, hRH=NAN;
    if(readDHTRobust(tC,hRH)){ t=tC; h=hRH; } else { t=NAN; h=NAN; }
  }

  // 3) LCD
  static char prev1[17]="", prev2[17]="";
  if(LCD_OK && now - lastLCD >= lcdUpdateMs && now - lastPumpSwitch >= lcdBlockAfterPumpMs){
    lastLCD = now;
    lcdReinitIfDue();

    char line1[17], line2[17];
    if(isnan(t)){ snprintf(line1, sizeof(line1), "Soil:%3d%% T:--.-", (int)(soilPctEMA+0.5f)); }
    else { char tb[8]; dtostrf(t,4,1,tb); snprintf(line1,sizeof(line1),"Soil:%3d%% T:%s",(int)(soilPctEMA+0.5f),tb); }

    const char* pstr = pumpOn ? "ON " : (manualPumpHold?"HLD":"OFF");
    if(isnan(h)){ snprintf(line2, sizeof(line2), "Hum: --%% Pump:%s", pstr); }
    else        { snprintf(line2, sizeof(line2), "Hum:%3d%% Pump:%s", (int)(h+0.5f), pstr); }

    if(strncmp(line1, prev1, 16)!=0){ lcdPrintLine(0, line1); strncpy(prev1, line1, 16); prev1[16]='\0'; }
    if(strncmp(line2, prev2, 16)!=0){ lcdPrintLine(1, line2); strncpy(prev2, line2, 16); prev2[16]='\0'; }
  }

  // 4) LCD 다운 복구
  if(!LCD_OK && now - lastLCDInit >= LCD_DOWN_RETRY_MS){
    lastLCDInit = now;
    if (lcd_recover_and_init()){
      lcdPrintLine(0, "Smart Plant");
      lcdPrintLine(1, currentProfile ? "LCD Recovered(F)" : "LCD Recovered(S)");
    }
  }

  // 5) 주기 전송(한 줄)
  if (now - lastTx >= TELEMETRY_MS){
    lastTx = now;
    sendTelemetry();
  }

  // 6) 명령 수신
  static char cmdBuf[64];
  if (readLineFrom(BT, cmdBuf, sizeof(cmdBuf)))    handleCommand(cmdBuf);
  if (readLineFrom(Serial, cmdBuf, sizeof(cmdBuf))) handleCommand(cmdBuf);
}
