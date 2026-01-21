#include <IRremote.hpp>
#define IR_SEND_PIN 11
#define IR_USE_AVR_TIMER1 true
#include <RBDdimmer.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//  Dimmer 
#define DIMMER_PIN 3
dimmerLamp dimmer(DIMMER_PIN);
int outVal = 0;
int lastDimmerPower = 0;
extern volatile bool pauseDimmer;

// LCD 
LiquidCrystal_I2C lcd(0x27,16,2);

// Projector 
int projStep = 0;

// RF 
#define TX_PIN 12
const unsigned long CODE_DOWN = 4329748;
const unsigned long CODE_STOP = 4329752;
const unsigned long CODE_UP   = 4329746;
const uint16_t T = 350;
const uint16_t SYNC_HIGH = T;
const uint16_t SYNC_LOW  = T*31;
const uint8_t REPEATS = 10;

// EMG PIN 
const int EMG_LAMP = A0;
const int EMG_RF   = A1;
const int THRESH_KUAT = 200;
const int SAMPLE_SIZE = 50;

// Step control
int step = 1;
int subStepLamp = 0;
int subStepRF = 0;
uint8_t stepIR = 0;

// BOBOT & BIAS NN LENGAN KANAN 
float w_ih_R[8][2] = {
  {0.647238, 0.412345}, {0.600293, -0.234567},
  {-0.911547, 0.789012}, {-0.064941, 0.345678},
  {-0.380773, -0.567890}, {-0.843068, 0.123456},
  {-0.420913, 0.678901}, {0.585665, -0.890123}
};

float b_h_R[8] = {
  0.111656, 0.010961, -0.034290, -0.077550,
  0.011397, -0.106508, 0.135733, 0.075606
};

float w_ho_R[3][8] = {
  {-0.254159, 0.395884, 0.613662, 0.316468, 0.464818, 0.267970, 0.757192, -0.517697},
  {0.005607,-0.623755,-0.230240, 0.400529,-0.534226,-0.858283, 0.680930,-0.475334},
  {0.586054, 0.341817,-0.260365, 0.643538,-0.815374, 0.177284,-0.090326, 0.785716}
};

float b_o_R[3] = {-0.119993, 0.128993, -0.087132};

//BOBOT & BIAS LENGAN KIRI
float w_ih_L[8][2] = {
  {-0.424953, 0.523167}, {0.386099, -0.678234},
  {0.545629, 0.234891}, {0.244797, -0.456123},
  {-0.393342, 0.789456}, {0.290013, -0.123789},
  {0.157613, 0.345678}, {0.244106, -0.567234}
};

float b_h_L[8] = {
  0.001579, 0.142982, -0.048289, 0.034009,
  0.140908, 0.035924, 0.136075, -0.114138
};

float w_ho_L[3][8] = {
  {-0.060290,-0.840445,-0.560231,-0.437989, 0.610308,-0.247639,-0.880391, 0.637171},
  { 0.384060, 0.184220,-0.765082,-0.547827,-0.163152,-0.666180,-0.080392,-0.009460},
  { 0.044604, 0.681146,-0.610970,-0.059474,-0.505267, 0.310715, 0.350678,-0.410863}
};

float b_o_L[3] = {0.033368, 0.006796, -0.046790};

//Ekstraksi
void getEMGFeatures(int pin, float* adc, float* rms) {
  long sumADC = 0;
  long sumSquare = 0;
  
  for (int i = 0; i < SAMPLE_SIZE; i++) {
    int value = analogRead(pin);
    sumADC += value;
    sumSquare += (long)value * value;
    delay(20);
  }
  
  *adc = (float)sumADC / SAMPLE_SIZE;
  *rms = sqrt((float)sumSquare / SAMPLE_SIZE);
}

//  FUNGSI 2 input
int predictNN(float adc, float rms, float w_ih[][2], float *b_h, float w_ho[][8], float *b_o) {
  float hidden[8];

  for (int i=0; i<8; i++) {
    float z = adc * w_ih[i][0] + rms * w_ih[i][1] + b_h[i];
    hidden[i] = (z > 0 ? z : 0);
  }

  float out[3];
  for (int o=0; o<3; o++) {
    float z = b_o[o];
    for (int h=0; h<8; h++) z += hidden[h] * w_ho[o][h];
    out[o] = z;
  }

  int maxIndex = 0;
  if (out[1] > out[maxIndex]) maxIndex = 1;
  if (out[2] > out[maxIndex]) maxIndex = 2;
  return maxIndex;
}

// RF Sender 
void txHigh(uint16_t us){ digitalWrite(TX_PIN,HIGH); delayMicroseconds(us); }
void txLow(uint16_t us){ digitalWrite(TX_PIN,LOW); delayMicroseconds(us); }
void sendBit(bool bit){ if(bit){ txHigh(T*3); txLow(T); } else { txHigh(T); txLow(T*3); } }
void sendSync(){ txHigh(SYNC_HIGH); txLow(SYNC_LOW); }
void sendFrame(unsigned long code,uint8_t bits){ for(int8_t i=bits-1;i>=0;i--) sendBit((code>>i)&1UL); sendSync(); }
void sendRF(unsigned long code, const char* label){
  noInterrupts();
  for(uint8_t r=0;r<REPEATS;r++) sendFrame(code,24);
  interrupts();
  lcd.setCursor(0,1);
  lcd.print("RF: "); lcd.print(label);
  Serial.print("[RF] "); Serial.println(label);
}

// Dimmer
void pauseDimmerForTx(){ lastDimmerPower = dimmer.getPower(); pauseDimmer = true; dimmer.setState(OFF); }
void resumeDimmerAfterTx(){ dimmer.setState(ON); dimmer.setPower(lastDimmerPower); pauseDimmer = false; }

void lampuControl(int state){
  switch(state){
    case 0: dimmer.setPower(10);  Serial.println("Lampu OFF");   break;
    case 1: dimmer.setPower(60);  Serial.println("Lampu REDUP"); break;
    case 2: dimmer.setPower(90);  Serial.println("Lampu TERANG");break;
  }
}

void setup(){
  Serial.begin(9600);
  dimmer.begin(NORMAL_MODE,ON);
  dimmer.setPower(outVal);

  IrSender.begin(IR_SEND_PIN);
  pinMode(TX_PIN,OUTPUT);
  digitalWrite(TX_PIN,LOW);

  lcd.init(); lcd.backlight();
  lcd.setCursor(0,0); lcd.print("EMG NN Control");
  Serial.println("Sistem NN Siap!");
}

void loop(){
  switch(step){

// LAMPU (KANAN)
    case 1: {
  float adc, rms;
  getEMGFeatures(EMG_LAMP, &adc, &rms);

  int level = predictNN(adc, rms, w_ih_R, b_h_R, w_ho_R, b_o_R);
  Serial.print("[KANAN NN] adc="); Serial.print(adc);
  Serial.print(" rms="); Serial.print(rms);
  Serial.print(" level="); Serial.println(level);

  switch (subStepLamp) {
    case 0:
      if (level == 1) { lampuControl(1); subStepLamp=1; }
      break;

    case 1:
      if (level == 2) { lampuControl(2); subStepLamp=2; }
      break;

    case 2:
      if (level == 1) { lampuControl(1); subStepLamp=3; }
      break;

    case 3:
      if (level == 0) { lampuControl(0); subStepLamp=4; }
      break;

    case 4:
      if (level == 0) {
        step = 2;
        subStepLamp = 0;
        Serial.println("STEP 1 DONE -> STEP 2 RF");
      }
      break;
  }

  break;
}

// RF (KIRI) 
case 2: {
  float adc, rms;
  getEMGFeatures(EMG_RF, &adc, &rms);

  int level = predictNN(adc, rms, w_ih_L, b_h_L, w_ho_L, b_o_L);
  Serial.print("[KIRI NN] adc="); Serial.print(adc);
  Serial.print(" rms="); Serial.print(rms);
  Serial.print(" level="); Serial.println(level);

  switch (subStepRF) {
    case 0:
      if (level == 0) { sendRF(CODE_DOWN,"DOWN"); subStepRF=1; }
      break;

    case 1:
      if (level == 1) { sendRF(CODE_STOP,"STOP"); subStepRF=2; }
      break;

    case 2:
      if (level == 2) { sendRF(CODE_UP,"UP"); subStepRF=3; }
      break;

    case 3:
      if (level == 1) {
        sendRF(CODE_STOP,"STOP");
        subStepRF = 0;
        step = 3;
        Serial.println("STEP 2 DONE -> STEP 3 IR");
      }
      break;
  }

  break;
}

// =================================================================
// ===================== STEP 3 : IR Proyektor (Threshold) ==========
// =================================================================
case 3: {
  int avgKanan = analogRead(EMG_LAMP);
  int avgKiri = analogRead(EMG_RF);

  bool bothContract = (avgKanan >= THRESH_KUAT && avgKiri >= THRESH_KUAT);

  Serial.print("[IR Threshold] Kanan="); Serial.print(avgKanan);
  Serial.print(" Kiri="); Serial.println(avgKiri);

  static unsigned long lastOn=0;
  const unsigned long MIN_WAIT = 70000;

  switch(stepIR){
    case 0:
      if (bothContract) {
        pauseDimmerForTx();
        IrSender.sendNEC(0x5583,0x90,5);
        resumeDimmerAfterTx();

        lastOn = millis();
        stepIR = 1;

        lcd.setCursor(0,1);
        lcd.print("Projector ON  ");
        Serial.println("Projector ON");
      }
      break;

    case 1:
      if (bothContract && millis() - lastOn > MIN_WAIT) {
        pauseDimmerForTx();
        IrSender.sendNEC(0x5583,0x92,5);
        resumeDimmerAfterTx();

        stepIR = 0;
        step   = 1;

        lcd.setCursor(0,1);
        lcd.print("Projector OFF ");
        Serial.println("Projector OFF");
      }
      break;
  }

  delay(20);
  break;
}
} 
} 