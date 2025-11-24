/*
 *  Arduino Mega 2560 – 2 Robot, 6 Step Motor + 1 Servo Test Kontrolü
 *
 *  ROBOT 1 Eksenleri:
 *    X, Y, Z, E
 *
 *  ROBOT 2 Eksenleri:
 *    R -> X2 ekseni   (diger lineer robot X)
 *    T -> Z2 ekseni   (diger lineer robot Z)
 *
 *  SERVO:
 *    S -> Servo sec (pin 47)
 *
 *  Ortak Komutlar (seri porttan, 115200 baud):
 *    x / y / z / e / r / t      -> eksen sec
 *    a                          -> secili stepper eksen ileri (ivmeli, sinirsiz)
 *    d                          -> secili stepper eksen geri (ivmeli, sinirsiz)
 *    w                          -> secili stepper eksen dur
 *    v=1500                     -> secili stepper eksen icin hiz (us/step)
 *    s=1000                     -> secili stepper eksen icin step sayisi (0 = sinirsiz)
 *    m=1000                     -> belirtilen step sayisi kadar ileri hareket
 *    n=1000                     -> belirtilen step sayisi kadar geri hareket
 *
 *  Servo icin:
 *    s                          -> servo sec
 *    p=90                       -> servo acisini 90 derece yap
 *
 *  G-code Komutlari:
 *    G0/G1 X10 Y20 Z5           -> dogrusal hareket (G0 hizli, G1 kontrollu)
 *    G28                         -> home pozisyonuna git (tum eksenleri sifirla)
 *    M104 S180                   -> servo acisini ayarla (S = aci, 0-180)
 *    M280 P0 S90                 -> servo kontrol (P = servo numarasi, S = aci)
 *    M114                        -> mevcut pozisyonu goster
 */

#include <Servo.h>

// ====== PIN TANIMLARI ======
// Bunlari RAMPS 1.4 pinlerine gore degistirebilirsin.

// ROBOT 1
// X Ekseni  (X V ~ 620)
#define X_PUL_PIN  2
#define X_DIR_PIN  3
#define X_ENA_PIN  4

// Y Ekseni  (Y V ~ 300)
#define Y_PUL_PIN  5
#define Y_DIR_PIN  6
#define Y_ENA_PIN  7

// Z Ekseni  (Z V ~ 50)
#define Z_PUL_PIN  8
#define Z_DIR_PIN  9
#define Z_ENA_PIN 10

// E Ekseni
#define E_PUL_PIN 11
#define E_DIR_PIN 12
#define E_ENA_PIN 13

// ROBOT 2
// Z2 diger lineer robot (T ekseni)
#define Z2_PUL_PIN 48
#define Z2_DIR_PIN 49
#define Z2_ENA_PIN 50

// X2 diger lineer robot (R ekseni)
#define X2_PUL_PIN 51
#define X2_DIR_PIN 52
#define X2_ENA_PIN 53

// Servo diger lineer robot
#define SERVO_PIN 47


// ====== GENEL AYARLAR ======
const uint16_t PULSE_WIDTH_US = 5;        // Step pals genisligi (us)
const float    ACCEL_TIME     = 0.5;      // Ivmelenme suresi (saniye)
const unsigned long START_INTERVAL_US = 10000;  // Ivmeye baslangic (yavas)
const unsigned long MIN_INTERVAL_US   = 200;    // En hizli (dusuk us = daha hizli)

// ====== EKSEN YAPISI ======
struct StepperAxis {
  uint8_t pulPin;
  uint8_t dirPin;
  uint8_t enaPin;

  unsigned long targetIntervalUs;   // Hedef hiz (us/step)
  unsigned long currentIntervalUs;  // O anki hiz (ivmeli)
  bool running;
  bool dirForward;
  unsigned long lastStepMicros;
  unsigned long accelStartTime;
  
  // Step sayisi kontrolu
  long targetSteps;      // Hedef step sayisi (0 = sinirsiz)
  long currentSteps;     // O anki step sayisi
};

// ROBOT 1 eksenleri
StepperAxis axisX  = { X_PUL_PIN,  X_DIR_PIN,  X_ENA_PIN,  620,  620, false, true, 0, 0, 0, 0 };  // X hizini 620 baslangic
StepperAxis axisY  = { Y_PUL_PIN,  Y_DIR_PIN,  Y_ENA_PIN,  300, 300, false, true, 0, 0, 0, 0 };  // Y hizini 300 baslangic
StepperAxis axisZ  = { Z_PUL_PIN,  Z_DIR_PIN,  Z_ENA_PIN,   50,  50, false, true, 0, 0, 0, 0 };  // Z hizini 50 baslangic
StepperAxis axisE  = { E_PUL_PIN,  E_DIR_PIN,  E_ENA_PIN, 1500,1500, false, true, 0, 0, 0, 0 };  // E standart

// ROBOT 2 eksenleri
StepperAxis axisR  = { X2_PUL_PIN, X2_DIR_PIN, X2_ENA_PIN, 1500,1500, false, true, 0, 0, 0, 0 }; // X2 (R)
StepperAxis axisT  = { Z2_PUL_PIN, Z2_DIR_PIN, Z2_ENA_PIN, 1500,1500, false, true, 0, 0, 0, 0 }; // Z2 (T)

// Servo
Servo robotServo;
int servoAngle = 90;

// Su an kontrol edilen eksen
char currentAxisName = 'X';
StepperAxis* currentAxis = &axisX;
bool currentIsServo = false;

// ========= Fonksiyon deklarasyonlari =========
StepperAxis* getAxisByName(char name);
void handleSerial();
void updateAcceleration(StepperAxis& axis);
void runStepper(StepperAxis& axis);
void handleServoCommand(const String& cmd);
void handleGCode(const String& line);
float parseGCodeValue(const String& line, char axis);
bool isAllMotorsStopped();

// ====================================
//               SETUP
// ====================================
void setup() {
  Serial.begin(115200);

  // Stepper pinleri
  pinMode(axisX.pulPin, OUTPUT);
  pinMode(axisX.dirPin, OUTPUT);
  pinMode(axisX.enaPin, OUTPUT);

  pinMode(axisY.pulPin, OUTPUT);
  pinMode(axisY.dirPin, OUTPUT);
  pinMode(axisY.enaPin, OUTPUT);

  pinMode(axisZ.pulPin, OUTPUT);
  pinMode(axisZ.dirPin, OUTPUT);
  pinMode(axisZ.enaPin, OUTPUT);

  pinMode(axisE.pulPin, OUTPUT);
  pinMode(axisE.dirPin, OUTPUT);
  pinMode(axisE.enaPin, OUTPUT);

  pinMode(axisR.pulPin, OUTPUT);
  pinMode(axisR.dirPin, OUTPUT);
  pinMode(axisR.enaPin, OUTPUT);

  pinMode(axisT.pulPin, OUTPUT);
  pinMode(axisT.dirPin, OUTPUT);
  pinMode(axisT.enaPin, OUTPUT);

  // Enable (LOW aktif ise)
  digitalWrite(axisX.enaPin, LOW);
  digitalWrite(axisY.enaPin, LOW);
  digitalWrite(axisZ.enaPin, LOW);
  digitalWrite(axisE.enaPin, LOW);
  digitalWrite(axisR.enaPin, LOW);
  digitalWrite(axisT.enaPin, LOW);

  // Varsayilan yonler
  digitalWrite(axisX.dirPin, HIGH);
  digitalWrite(axisY.dirPin, HIGH);
  digitalWrite(axisZ.dirPin, HIGH);
  digitalWrite(axisE.dirPin, HIGH);
  digitalWrite(axisR.dirPin, HIGH);
  digitalWrite(axisT.dirPin, HIGH);

  // Servo
  robotServo.attach(SERVO_PIN);
  robotServo.write(servoAngle);

  Serial.println(F("=== 2 Robot, 6 Step Motor + 1 Servo Test Kontrolu ==="));
  Serial.println(F("ROBOT 1: X, Y, Z, E"));
  Serial.println(F("ROBOT 2: R (X2), T (Z2)"));
  Serial.println(F("Servo  : S (pin 47)"));
  Serial.println();
  Serial.println(F("Komutlar:"));
  Serial.println(F("  x/y/z/e/r/t  -> stepper eksen sec"));
  Serial.println(F("  s            -> servo sec"));
  Serial.println(F("  a            -> secili stepper ileri (ivmeli, sinirsiz)"));
  Serial.println(F("  d            -> secili stepper geri (ivmeli, sinirsiz)"));
  Serial.println(F("  w            -> secili stepper dur"));
  Serial.println(F("  v=<us>       -> stepper hiz (us/adim)"));
  Serial.println(F("  s=<step>     -> stepper step sayisi ayarla (0=sinirsiz)"));
  Serial.println(F("  m=<step>     -> belirtilen step kadar ileri hareket"));
  Serial.println(F("  n=<step>     -> belirtilen step kadar geri hareket"));
  Serial.println(F("  p=<aci>      -> servo icin derece (0-180), ornek: p=90"));
  Serial.println();
  Serial.println(F("G-code Komutlari:"));
  Serial.println(F("  G0/G1 X10 Y20 -> dogrusal hareket"));
  Serial.println(F("  G28           -> home pozisyonuna git"));
  Serial.println(F("  M104 S180     -> servo acisini ayarla (S = aci, 0-180)"));
  Serial.println(F("  M280 P0 S90   -> servo kontrol (P = servo no, S = aci)"));
  Serial.println(F("  M114          -> mevcut pozisyonu goster"));
  Serial.println();
}

// ====================================
//                LOOP
// ====================================
void loop() {
  handleSerial();

  // Tum stepper eksenlerde ivmeyi guncelle
  updateAcceleration(axisX);
  updateAcceleration(axisY);
  updateAcceleration(axisZ);
  updateAcceleration(axisE);
  updateAcceleration(axisR);
  updateAcceleration(axisT);

  // Tum stepper eksenleri calistir
  runStepper(axisX);
  runStepper(axisY);
  runStepper(axisZ);
  runStepper(axisE);
  runStepper(axisR);
  runStepper(axisT);
}

// ====================================
//        EKSEN SECME FONKSIYONU
// ====================================
StepperAxis* getAxisByName(char name) {
  switch (name) {
    case 'X': return &axisX;
    case 'Y': return &axisY;
    case 'Z': return &axisZ;
    case 'E': return &axisE;
    case 'R': return &axisR;  // X2
    case 'T': return &axisT;  // Z2
    default:  return &axisX;
  }
}

// ====================================
//          SERI KOMUT OKUMA
// ====================================
void handleSerial() {
  if (!Serial.available()) return;

  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // G-code kontrolu (G veya M ile baslayan komutlar)
  if (line.charAt(0) == 'G' || line.charAt(0) == 'M') {
    handleGCode(line);
    return;
  }

  // Ilk karakter eksen secimi olabilir
  char c0 = toupper(line.charAt(0));

  if (c0 == 'X' || c0 == 'Y' || c0 == 'Z' || c0 == 'E' || c0 == 'R' || c0 == 'T' || c0 == 'S') {
    currentAxisName = c0;

    if (c0 == 'S') {
      currentIsServo = true;
      Serial.println(F("Servo secildi (S)."));
      if (line.length() == 1) return;
      line.remove(0, 1);
      line.trim();
      if (line.length() == 0) return;
    } else {
      currentIsServo = false;
      currentAxis = getAxisByName(c0);

      // Sadece eksen secildiyse (ornegin "x")
      if (line.length() == 1) {
        Serial.print(F("Secilen eksen: "));
        Serial.println(currentAxisName);
        return;
      }

      // Eksen harfini sil, kalan kisim komut olsun (ornegin "x a" -> "a")
      line.remove(0, 1);
      line.trim();
      if (line.length() == 0) return;
    }
  }

  // Servo seciliyse servo komutlarini isle
  if (currentIsServo) {
    handleServoCommand(line);
    return;
  }

  // Buradan sonra line, stepper komut stringi (a, d, w, v=, s=, m=, n=...)
  // Ve currentAxis secili
  if (line.equalsIgnoreCase("a")) {
    currentAxis->dirForward = true;
    digitalWrite(currentAxis->dirPin, HIGH);
    currentAxis->running = true;
    currentAxis->accelStartTime = millis();
    currentAxis->currentIntervalUs = START_INTERVAL_US;
    currentAxis->targetSteps = 0;  // Sinirsiz
    currentAxis->currentSteps = 0;

    Serial.print(F("Eksen "));
    Serial.print(currentAxisName);
    Serial.println(F(" -> Ileri ivmeli basladi (sinirsiz)."));
  }
  else if (line.equalsIgnoreCase("d")) {
    currentAxis->dirForward = false;
    digitalWrite(currentAxis->dirPin, LOW);
    currentAxis->running = true;
    currentAxis->accelStartTime = millis();
    currentAxis->currentIntervalUs = START_INTERVAL_US;
    currentAxis->targetSteps = 0;  // Sinirsiz
    currentAxis->currentSteps = 0;

    Serial.print(F("Eksen "));
    Serial.print(currentAxisName);
    Serial.println(F(" -> Geri ivmeli basladi (sinirsiz)."));
  }
  else if (line.equalsIgnoreCase("w")) {
    currentAxis->running = false;
    currentAxis->targetSteps = 0;
    currentAxis->currentSteps = 0;
    Serial.print(F("Eksen "));
    Serial.print(currentAxisName);
    Serial.println(F(" -> Durdu."));
  }
  else if (line.startsWith("v=") || line.startsWith("V=")) {
    long v = line.substring(2).toInt();
    v = constrain(v, (long)MIN_INTERVAL_US, (long)START_INTERVAL_US);

    currentAxis->targetIntervalUs = (unsigned long)v;
    Serial.print(F("Eksen "));
    Serial.print(currentAxisName);
    Serial.print(F(" -> Hedef hiz (us/adim): "));
    Serial.println(currentAxis->targetIntervalUs);
  }
  else if (line.startsWith("s=") || line.startsWith("S=")) {
    long steps = line.substring(2).toInt();
    if (steps < 0) steps = 0;
    currentAxis->targetSteps = steps;
    Serial.print(F("Eksen "));
    Serial.print(currentAxisName);
    Serial.print(F(" -> Hedef step sayisi: "));
    Serial.println(currentAxis->targetSteps);
  }
  else if (line.startsWith("m=") || line.startsWith("M=")) {
    long steps = line.substring(2).toInt();
    if (steps < 0) steps = 0;
    
    currentAxis->dirForward = true;
    digitalWrite(currentAxis->dirPin, HIGH);
    currentAxis->targetSteps = steps;
    currentAxis->currentSteps = 0;
    currentAxis->running = true;
    currentAxis->accelStartTime = millis();
    currentAxis->currentIntervalUs = START_INTERVAL_US;

    Serial.print(F("Eksen "));
    Serial.print(currentAxisName);
    Serial.print(F(" -> Ileri "));
    Serial.print(steps);
    Serial.println(F(" step basladi."));
  }
  else if (line.startsWith("n=") || line.startsWith("N=")) {
    long steps = line.substring(2).toInt();
    if (steps < 0) steps = 0;
    
    currentAxis->dirForward = false;
    digitalWrite(currentAxis->dirPin, LOW);
    currentAxis->targetSteps = steps;
    currentAxis->currentSteps = 0;
    currentAxis->running = true;
    currentAxis->accelStartTime = millis();
    currentAxis->currentIntervalUs = START_INTERVAL_US;

    Serial.print(F("Eksen "));
    Serial.print(currentAxisName);
    Serial.print(F(" -> Geri "));
    Serial.print(steps);
    Serial.println(F(" step basladi."));
  }
  else {
    Serial.print(F("Tanimlanmamis komut: "));
    Serial.println(line);
  }
}

// ====================================
//        SERVO KOMUTLARI
// ====================================
void handleServoCommand(const String& cmd) {
  // Beklenen format: p=aci  (0-180)
  if (cmd.startsWith("p=") || cmd.startsWith("P=")) {
    int angle = cmd.substring(2).toInt();
    angle = constrain(angle, 0, 180);
    servoAngle = angle;
    robotServo.write(servoAngle);

    Serial.print(F("Servo acisi -> "));
    Serial.println(servoAngle);
  } else {
    Serial.print(F("Servo icin tanimsiz komut: "));
    Serial.println(cmd);
    Serial.println(F("Ornek: p=90"));
  }
}

// ====================================
//         IVMELENME GUNCELLEME
// ====================================
void updateAcceleration(StepperAxis& axis) {
  if (!axis.running) return;

  unsigned long elapsed = millis() - axis.accelStartTime;
  unsigned long accelDurationMs = (unsigned long)(ACCEL_TIME * 1000.0);

  if (elapsed >= accelDurationMs) {
    axis.currentIntervalUs = axis.targetIntervalUs; // ramp tamam
    return;
  }

  float progress = (float)elapsed / accelDurationMs;
  unsigned long startInterval = START_INTERVAL_US;
  unsigned long target = axis.targetIntervalUs;

  long diff = (long)startInterval - (long)target;
  axis.currentIntervalUs = startInterval - (unsigned long)(progress * diff);

  // Guvenlik
  if (axis.currentIntervalUs < target) {
    axis.currentIntervalUs = target;
  }
}

// ====================================
//           MOTOR CALISTIRMA
// ====================================
void runStepper(StepperAxis& axis) {
  if (!axis.running) return;

  // Step sayisi kontrolu - eger targetSteps > 0 ise ve currentSteps >= targetSteps ise dur
  if (axis.targetSteps > 0 && axis.currentSteps >= axis.targetSteps) {
    axis.running = false;
    axis.targetSteps = 0;
    axis.currentSteps = 0;
    return;
  }

  unsigned long now = micros();
  if (now - axis.lastStepMicros >= axis.currentIntervalUs) {
    digitalWrite(axis.pulPin, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(axis.pulPin, LOW);

    axis.lastStepMicros = now;
    
    // Step sayisini artir (eger targetSteps > 0 ise)
    if (axis.targetSteps > 0) {
      axis.currentSteps++;
    }
  }
}

// ====================================
//           G-CODE PARSER
// ====================================
float parseGCodeValue(const String& line, char axis) {
  int axisIndex = line.indexOf(axis);
  if (axisIndex < 0) return -999.0;  // Eksik deger
  
  int start = axisIndex + 1;
  int end = start;
  
  // Sayiyi bul
  while (end < line.length() && 
         ((line.charAt(end) >= '0' && line.charAt(end) <= '9') || line.charAt(end) == '.' || line.charAt(end) == '-')) {
    end++;
  }
  
  if (end > start) {
    return line.substring(start, end).toFloat();
  }
  
  return -999.0;
}

bool isAllMotorsStopped() {
  return !axisX.running && !axisY.running && !axisZ.running && 
         !axisE.running && !axisR.running && !axisT.running;
}

void handleGCode(const String& line) {
  String cmd = line;
  cmd.toUpperCase();
  cmd.trim();
  
  // G0 veya G1: Dogrusal hareket
  if (cmd.startsWith("G0") || cmd.startsWith("G1")) {
    // Tum motorlar durana kadar bekle
    while (!isAllMotorsStopped()) {
      delay(10);
    }
    
    // Eksen degerlerini parse et
    float xVal = parseGCodeValue(cmd, 'X');
    float yVal = parseGCodeValue(cmd, 'Y');
    float zVal = parseGCodeValue(cmd, 'Z');
    float eVal = parseGCodeValue(cmd, 'E');
    float rVal = parseGCodeValue(cmd, 'R');
    float tVal = parseGCodeValue(cmd, 'T');
    
    // F (feed rate) degerini parse et
    float fVal = parseGCodeValue(cmd, 'F');
    
    // Step/mm donusum faktoru (varsayilan: 1 mm = 100 step)
    // Bu degeri ihtiyaciniza gore ayarlayabilirsiniz
    const float STEPS_PER_MM = 100.0;
    
    bool movement = false;
    
    // X ekseni
    if (xVal != -999.0) {
      long steps = (long)(xVal * STEPS_PER_MM);
      if (steps != 0) {
        currentAxis = &axisX;
        currentAxis->targetIntervalUs = (fVal > 0) ? (unsigned long)(60000000.0 / (fVal * STEPS_PER_MM)) : 620;
        if (currentAxis->targetIntervalUs < MIN_INTERVAL_US) currentAxis->targetIntervalUs = MIN_INTERVAL_US;
        if (currentAxis->targetIntervalUs > START_INTERVAL_US) currentAxis->targetIntervalUs = START_INTERVAL_US;
        
        currentAxis->dirForward = (steps > 0);
        digitalWrite(currentAxis->dirPin, currentAxis->dirForward ? HIGH : LOW);
        currentAxis->targetSteps = abs(steps);
        currentAxis->currentSteps = 0;
        currentAxis->running = true;
        currentAxis->accelStartTime = millis();
        currentAxis->currentIntervalUs = START_INTERVAL_US;
        movement = true;
      }
    }
    
    // Y ekseni
    if (yVal != -999.0) {
      long steps = (long)(yVal * STEPS_PER_MM);
      if (steps != 0) {
        currentAxis = &axisY;
        currentAxis->targetIntervalUs = (fVal > 0) ? (unsigned long)(60000000.0 / (fVal * STEPS_PER_MM)) : 300;
        if (currentAxis->targetIntervalUs < MIN_INTERVAL_US) currentAxis->targetIntervalUs = MIN_INTERVAL_US;
        if (currentAxis->targetIntervalUs > START_INTERVAL_US) currentAxis->targetIntervalUs = START_INTERVAL_US;
        
        currentAxis->dirForward = (steps > 0);
        digitalWrite(currentAxis->dirPin, currentAxis->dirForward ? HIGH : LOW);
        currentAxis->targetSteps = abs(steps);
        currentAxis->currentSteps = 0;
        currentAxis->running = true;
        currentAxis->accelStartTime = millis();
        currentAxis->currentIntervalUs = START_INTERVAL_US;
        movement = true;
      }
    }
    
    // Z ekseni
    if (zVal != -999.0) {
      long steps = (long)(zVal * STEPS_PER_MM);
      if (steps != 0) {
        currentAxis = &axisZ;
        currentAxis->targetIntervalUs = (fVal > 0) ? (unsigned long)(60000000.0 / (fVal * STEPS_PER_MM)) : 50;
        if (currentAxis->targetIntervalUs < MIN_INTERVAL_US) currentAxis->targetIntervalUs = MIN_INTERVAL_US;
        if (currentAxis->targetIntervalUs > START_INTERVAL_US) currentAxis->targetIntervalUs = START_INTERVAL_US;
        
        currentAxis->dirForward = (steps > 0);
        digitalWrite(currentAxis->dirPin, currentAxis->dirForward ? HIGH : LOW);
        currentAxis->targetSteps = abs(steps);
        currentAxis->currentSteps = 0;
        currentAxis->running = true;
        currentAxis->accelStartTime = millis();
        currentAxis->currentIntervalUs = START_INTERVAL_US;
        movement = true;
      }
    }
    
    // E ekseni
    if (eVal != -999.0) {
      long steps = (long)(eVal * STEPS_PER_MM);
      if (steps != 0) {
        currentAxis = &axisE;
        currentAxis->targetIntervalUs = (fVal > 0) ? (unsigned long)(60000000.0 / (fVal * STEPS_PER_MM)) : 1500;
        if (currentAxis->targetIntervalUs < MIN_INTERVAL_US) currentAxis->targetIntervalUs = MIN_INTERVAL_US;
        if (currentAxis->targetIntervalUs > START_INTERVAL_US) currentAxis->targetIntervalUs = START_INTERVAL_US;
        
        currentAxis->dirForward = (steps > 0);
        digitalWrite(currentAxis->dirPin, currentAxis->dirForward ? HIGH : LOW);
        currentAxis->targetSteps = abs(steps);
        currentAxis->currentSteps = 0;
        currentAxis->running = true;
        currentAxis->accelStartTime = millis();
        currentAxis->currentIntervalUs = START_INTERVAL_US;
        movement = true;
      }
    }
    
    // R ekseni (X2)
    if (rVal != -999.0) {
      long steps = (long)(rVal * STEPS_PER_MM);
      if (steps != 0) {
        currentAxis = &axisR;
        currentAxis->targetIntervalUs = (fVal > 0) ? (unsigned long)(60000000.0 / (fVal * STEPS_PER_MM)) : 1500;
        if (currentAxis->targetIntervalUs < MIN_INTERVAL_US) currentAxis->targetIntervalUs = MIN_INTERVAL_US;
        if (currentAxis->targetIntervalUs > START_INTERVAL_US) currentAxis->targetIntervalUs = START_INTERVAL_US;
        
        currentAxis->dirForward = (steps > 0);
        digitalWrite(currentAxis->dirPin, currentAxis->dirForward ? HIGH : LOW);
        currentAxis->targetSteps = abs(steps);
        currentAxis->currentSteps = 0;
        currentAxis->running = true;
        currentAxis->accelStartTime = millis();
        currentAxis->currentIntervalUs = START_INTERVAL_US;
        movement = true;
      }
    }
    
    // T ekseni (Z2)
    if (tVal != -999.0) {
      long steps = (long)(tVal * STEPS_PER_MM);
      if (steps != 0) {
        currentAxis = &axisT;
        currentAxis->targetIntervalUs = (fVal > 0) ? (unsigned long)(60000000.0 / (fVal * STEPS_PER_MM)) : 1500;
        if (currentAxis->targetIntervalUs < MIN_INTERVAL_US) currentAxis->targetIntervalUs = MIN_INTERVAL_US;
        if (currentAxis->targetIntervalUs > START_INTERVAL_US) currentAxis->targetIntervalUs = START_INTERVAL_US;
        
        currentAxis->dirForward = (steps > 0);
        digitalWrite(currentAxis->dirPin, currentAxis->dirForward ? HIGH : LOW);
        currentAxis->targetSteps = abs(steps);
        currentAxis->currentSteps = 0;
        currentAxis->running = true;
        currentAxis->accelStartTime = millis();
        currentAxis->currentIntervalUs = START_INTERVAL_US;
        movement = true;
      }
    }
    
    if (movement) {
      Serial.print(F("G-code: "));
      Serial.println(cmd);
    } else {
      Serial.println(F("G-code: Hareket yok"));
    }
  }
  // G28: Home pozisyonuna git
  else if (cmd.startsWith("G28")) {
    // Tum motorlari durdur
    axisX.running = false;
    axisY.running = false;
    axisZ.running = false;
    axisE.running = false;
    axisR.running = false;
    axisT.running = false;
    
    Serial.println(F("G28: Home pozisyonuna gidiliyor..."));
    Serial.println(F("NOT: Home sensörleri eklenmeli!"));
  }
  // M104: Servo açısını ayarla (S parametresi ile)
  else if (cmd.startsWith("M104")) {
    float sVal = parseGCodeValue(cmd, 'S');
    if (sVal != -999.0) {
      int angle = (int)sVal;
      angle = constrain(angle, 0, 180);
      servoAngle = angle;
      robotServo.write(servoAngle);
      Serial.print(F("M104: Servo acisi -> "));
      Serial.println(servoAngle);
    } else {
      Serial.println(F("M104: Hata - S parametresi gerekli (ornek: M104 S90)"));
    }
  }
  // M280: Servo kontrol (standart G-code komutu)
  // Format: M280 P<servo_no> S<açı>
  else if (cmd.startsWith("M280")) {
    float pVal = parseGCodeValue(cmd, 'P');
    float sVal = parseGCodeValue(cmd, 'S');
    
    if (sVal != -999.0) {
      int angle = (int)sVal;
      angle = constrain(angle, 0, 180);
      servoAngle = angle;
      robotServo.write(servoAngle);
      
      if (pVal != -999.0) {
        Serial.print(F("M280: Servo "));
        Serial.print((int)pVal);
        Serial.print(F(" acisi -> "));
        Serial.println(servoAngle);
      } else {
        Serial.print(F("M280: Servo acisi -> "));
        Serial.println(servoAngle);
      }
    } else {
      Serial.println(F("M280: Hata - S parametresi gerekli (ornek: M280 P0 S90)"));
    }
  }
  // M114: Mevcut pozisyonu göster
  else if (cmd.startsWith("M114")) {
    Serial.print(F("M114: Pozisyon - "));
    Serial.print(F("X:0 Y:0 Z:0 E:0 R:0 T:0"));
    Serial.print(F(" Servo:"));
    Serial.print(servoAngle);
    Serial.println();
  }
  else {
    Serial.print(F("G-code taninmadi: "));
    Serial.println(cmd);
  }
}
