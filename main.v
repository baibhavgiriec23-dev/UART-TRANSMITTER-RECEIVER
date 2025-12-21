// Compact combined ESP32 sketch: MAX30102, MPU6050, AD8232 (ECG), DS18B20
// Minimal changes: added WebServer endpoints (/, /data) and a few globals for ECG
#include <Wire.h>
#include "MAX30105.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>

MAX30105 particleSensor;
#define MAX_BUF 100
uint32_t redBuf[MAX_BUF], irBuf[MAX_BUF];
int bufIndex = 0;

// Maxim outputs (non-volatile)
int32_t spo2 = 0;
int8_t spo2_valid = 0;
int32_t hr = 0;
int8_t hr_valid = 0;

#define MPU_ADDR 0x68
const float ACCEL_SCALE = 16384.0f;
const float GYRO_SCALE  = 131.0f;
float roll=0, pitch=0;
const float alpha = 0.98f;

const int ECG_PIN = 34, LO_PLUS = 25, LO_MINUS = 26;
const double ECG_RATE = 360.0;
unsigned long nextECGmicros = 0;
float dc = 0; const float DC_ALPHA=0.995f;

// expose ECG values for web endpoint
float ecgValue = 0.0f;
bool ecgLeadOK = true;

#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
float tempC = NAN;

unsigned long lastMaxSample=0, lastMPU=0, lastTemp=0, lastStatus=0;
const unsigned long MAX_US=10000UL, MPU_MS=10, TEMP_MS=1000, STATUS_MS=1000;

/* ---------------- WiFi and Web Server ---------------- */
const char* WIFI_SSID = "iQOO NEO 7 PRO";      // <-- change to your SSID
const char* WIFI_PASS = "52f4c5788786";       // <-- change to your password
WebServer server(80);

/* Minimal forward declarations */
float removeDC(float raw);
bool readMPU(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz);
int32_t maxim_heart_rate_and_oxygen_saturation(int *irBufInt, int *redBufInt, int len,
                                               int32_t *pSpO2, int8_t *pSpO2Valid,
                                               int32_t *pHR, int8_t *pHRValid);
void handleRoot();
void handleData();

void setup(){
  Serial.begin(115200);
  delay(100);

  // --- Connect to WiFi (minimal, blocking) ---
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected ✔ IP = ");
  Serial.println(WiFi.localIP());            // prints local IP address
  if (MDNS.begin("esp32")) {
    Serial.println("mDNS responder started: http://esp32.local");
  }

  // start web server endpoints
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.begin();

  Wire.begin(21,22); Wire.setClock(100000);
  // MAX30102
  if (particleSensor.begin(Wire)) {
    particleSensor.setup(0x1F,4,2,100);
    particleSensor.setPulseAmplitudeRed(0x1F);
    particleSensor.setPulseAmplitudeIR(0x1F);
  } else Serial.println("MAX30102 not found");
  // MPU6050 wake
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0); Wire.endTransmission();
  // ECG
  pinMode(LO_PLUS, INPUT); pinMode(LO_MINUS, INPUT);
  analogReadResolution(12); analogSetPinAttenuation(ECG_PIN, ADC_11db);
  nextECGmicros = micros();
  // DS18B20
  sensors.begin(); sensors.requestTemperatures(); lastTemp=millis();
  Serial.println("ECG format: ECG,<value>,<lead_valid>,<LO+>,<LO->");
}

void loop(){
  unsigned long nowMicros = micros();
  unsigned long nowMillis = millis();

  // ECG sampling ~360Hz
  if ((long)(nowMicros - nextECGmicros) >= 0) {
    nextECGmicros += (unsigned long)(1000000.0/ECG_RATE);
    int raw = analogRead(ECG_PIN);
    float centered = removeDC((float)raw);
    ecgValue = centered;
    int loP = digitalRead(LO_PLUS), loM = digitalRead(LO_MINUS);
    ecgLeadOK = !(loP==0 || loM==0);
    // Serial ECG line for Serial Plotter (optional)
    Serial.print("ECG,"); Serial.print(centered,2); Serial.print(","); Serial.print(ecgLeadOK?1:0);
    Serial.print(","); Serial.print(loP); Serial.print(","); Serial.println(loM);
  }

  // MAX30102 sampling ~100Hz
  if ((unsigned long)(nowMicros - lastMaxSample) >= MAX_US) {
    lastMaxSample = nowMicros;
    if (particleSensor.available()) {
      redBuf[bufIndex] = particleSensor.getRed();
      irBuf[bufIndex]  = particleSensor.getIR();
      bufIndex++;
      particleSensor.nextSample();
      if (bufIndex >= MAX_BUF) {
        // compute HR/SpO2
        maxim_heart_rate_and_oxygen_saturation((int*)irBuf,(int*)redBuf,MAX_BUF,&spo2,&spo2_valid,&hr,&hr_valid);
        bufIndex = 0;
      }
    } else particleSensor.check();
  }

  // MPU6050 read ~100Hz
  if (nowMillis - lastMPU >= MPU_MS) {
    lastMPU = nowMillis;
    int16_t ax,ay,az,gx,gy,gz;
    if (readMPU(ax,ay,az,gx,gy,gz)) {
      float axf=ax/ACCEL_SCALE, ayf=ay/ACCEL_SCALE, azf=az/ACCEL_SCALE;
      float gxf=gx/GYRO_SCALE, gyf=gy/GYRO_SCALE;
      static unsigned long lastMicros=0;
      unsigned long m=micros(); float dt=(m-lastMicros)/1e6f; if(dt<=0||dt>0.1) dt=0.01f; lastMicros=m;
      float rAcc = atan2(ayf,azf)*180.0/PI;
      float pAcc = atan2(-axf, sqrt(ayf*ayf+azf*azf))*180.0/PI;
      float rGy = roll + gxf*dt, pGy = pitch + gyf*dt;
      roll = alpha*rGy + (1-alpha)*rAcc;
      pitch= alpha*pGy + (1-alpha)*pAcc;
    }
  }

  // DS18B20 every 1s
  if (nowMillis - lastTemp >= TEMP_MS) {
    lastTemp = nowMillis;
    sensors.requestTemperatures();
    float t = sensors.getTempCByIndex(0);
    if (t!=DEVICE_DISCONNECTED_C) tempC = t;
  }

  // Status print every 1s
  if (nowMillis - lastStatus >= STATUS_MS) {
    lastStatus = nowMillis;

    Serial.print("STATUS | HR:");
    if (hr_valid) Serial.print(hr/2); else Serial.print("N/A");
    Serial.print(" BPM | SpO2:");
    if (spo2_valid) Serial.print(spo2); else Serial.print("N/A");
    Serial.print("% | Temp:"); Serial.print(tempC+3,2);
    Serial.print(" C | Roll:"); Serial.print(roll,2); Serial.print(" | Pitch:"); Serial.println(pitch,2);
  }

  server.handleClient();
  delayMicroseconds(50);
}

/********************************************************************
  WEB PAGE (with ECG graph) - same as your reference page (minimal)
********************************************************************/
void handleRoot() {
static const char page[] PROGMEM = R"====(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ESP32 Vitals</title>
<style>
body{background:#111;color:#fff;text-align:center;font-family:Arial}
.card{background:#222;padding:15px;margin:10px;border-radius:10px;display:inline-block}
.value{font-size:2rem;color:#0f0}
</style>
</head>
<body>
<h2>ESP32 Live Vitals</h2>

<div class="card"><div>Heart Rate</div><div id="hr" class="value">--</div></div>
<div class="card"><div>SpO₂</div><div id="spo2" class="value">--</div></div>
<div class="card"><div>Temp</div><div id="temp" class="value">--</div></div>
<div class="card"><div>Roll | Pitch</div><div id="ori" class="value">--</div></div>

<div class="card" style="width:95%;max-width:600px;">
  <div>ECG</div>
  <canvas id="ecg" width="600" height="150" style="background:black;border-radius:10px;"></canvas>
</div>

<script>
let ecgBuf=[];
function drawECG(v){
  const c=document.getElementById("ecg");
  const x=c.getContext("2d");
  ecgBuf.push(v);
  if(ecgBuf.length>600) ecgBuf.shift();
  x.fillStyle="#000"; x.fillRect(0,0,c.width,c.height);
  x.strokeStyle="#00ff55"; x.beginPath();
  for(let i=0;i<ecgBuf.length;i++){
    let y=75 - ecgBuf[i]*0.15;
    if(i===0) x.moveTo(0,y); else x.lineTo(i,y);
  }
  x.stroke();
}

async function poll(){
  try {
    let r=await fetch("/data");
    let d=await r.json();
    document.getElementById('hr').textContent = d.hr>=0 ? d.hr/2 : "--";
    document.getElementById('spo2').textContent = d.spo2>=0 ? d.spo2 : "--";
    document.getElementById('temp').textContent = d.temp!==null ? (d.temp+3).toFixed(1) : "--";
    document.getElementById('ori').textContent = d.roll.toFixed(0)+" | "+d.pitch.toFixed(0);
    drawECG(d.ecg);
  } catch(e){
    console.log("poll error", e);
  }
}
setInterval(poll,200);
</script>

</body>
</html>
)====";

  server.send_P(200,"text/html",page);
}

/********************************************************************
  JSON API
********************************************************************/
void handleData() {
  char json[200];
  // temp may be NAN; handle by printing null
  if (isnan(tempC)) tempC = 0.0;
  snprintf(json, sizeof(json),
    "{\"hr\":%d,\"spo2\":%d,\"temp\":%.2f,\"roll\":%.1f,"
    "\"pitch\":%.1f,\"ecg\":%.2f}",
    hr_valid ? hr : -1,
    spo2_valid ? spo2 : -1,
    tempC,
    roll,
    pitch,
    ecgValue
  );
  server.send(200, "application/json", json);
}

/********************************************************************
  Helpers (same as before)
********************************************************************/
float removeDC(float raw){ dc = DC_ALPHA*dc + (1-DC_ALPHA)*raw; return raw-dc; }

bool readMPU(int16_t &ax,int16_t &ay,int16_t &az,int16_t &gx,int16_t &gy,int16_t &gz){
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B);
  if (Wire.endTransmission(false)!=0) return false;
  Wire.requestFrom(MPU_ADDR,(uint8_t)14);
  if (Wire.available()<14) return false;
  ax=(Wire.read()<<8)|Wire.read(); ay=(Wire.read()<<8)|Wire.read(); az=(Wire.read()<<8)|Wire.read();
  (void)((Wire.read()<<8)|Wire.read()); gx=(Wire.read()<<8)|Wire.read(); gy=(Wire.read()<<8)|Wire.read(); gz=(Wire.read()<<8)|Wire.read();
  return true;
}

/* ---- Maxim heart rate & SpO2 algorithm (compacted) ----
   Same core algorithm as earlier; expects int* arrays and returns via pointers.
*/
int32_t maxim_heart_rate_and_oxygen_saturation(int *irBufInt, int *redBufInt, int len,
                                               int32_t *pSpO2, int8_t *pSpO2Valid,
                                               int32_t *pHR, int8_t *pHRValid) {
  int i,k;
  double irMean=0, redMean=0;
  for(i=0;i<len;i++){ irMean+=irBufInt[i]; redMean+=redBufInt[i]; }
  irMean/=len; redMean/=len;
  double *irAC=(double*)malloc(sizeof(double)*len), *redAC=(double*)malloc(sizeof(double)*len);
  if(!irAC||!redAC){ if(irAC)free(irAC); if(redAC)free(redAC); *pSpO2=0; *pSpO2Valid=0; *pHR=0; *pHRValid=0; return -1;}
  for(i=0;i<len;i++){ irAC[i]=irBufInt[i]-irMean; redAC[i]=redBufInt[i]-redMean; }
  int *peaks=(int*)malloc(sizeof(int)*len); int nPeaks=0;
  double thr=0; for(i=0;i<len;i++) if(irAC[i]>thr) thr=irAC[i]; thr*=0.4;
  for(i=1;i<len-1;i++) if(irAC[i]>irAC[i-1] && irAC[i]>irAC[i+1] && irAC[i]>thr) {
    if(nPeaks==0 || (i-peaks[nPeaks-1])>30) peaks[nPeaks++]=i;
  }
  if(nPeaks>=2){
    double avgI=0; for(i=1;i<nPeaks;i++) avgI+= (peaks[i]-peaks[i-1]); avgI/=(nPeaks-1);
    double sample_rate=100.0; double hrCalc = 60.0*sample_rate/avgI; *pHR=(int32_t)(hrCalc+0.5); *pHRValid=1;
  } else { *pHR=0; *pHRValid=0; }
  double sumR=0; int vcnt=0;
  for(k=0;k<nPeaks;k++){
    int loc=peaks[k], L=loc-7; if(L<0)L=0; int R=loc+7; if(R>=len)R=len-1;
    double irMax=-1e9,irMin=1e9,redMax=-1e9,redMin=1e9;
    for(i=L;i<=R;i++){ if(irAC[i]>irMax)irMax=irAC[i]; if(irAC[i]<irMin)irMin=irAC[i]; if(redAC[i]>redMax)redMax=redAC[i]; if(redAC[i]<redMin)redMin=redAC[i];}
    double irA=irMax-irMin, redA=redMax-redMin;
    if(irA>1e-3 && redA>1e-3){
      double ratio = (redA/(redMean))/(irA/(irMean));
      if(ratio>0.02 && ratio<2.0){ sumR+=ratio; vcnt++; }
    }
  }
  if(vcnt>0){
    double ravg=sumR/vcnt; double sp = -45.060*ravg*ravg + 30.354*ravg + 94.845;
    if(sp>100) sp=100; if(sp<0) sp=0; *pSpO2=(int32_t)(sp+0.5); *pSpO2Valid=1;
  } else { *pSpO2=0; *pSpO2Valid=0; }
  free(irAC); free(redAC); free(peaks);
  return 0;
}
