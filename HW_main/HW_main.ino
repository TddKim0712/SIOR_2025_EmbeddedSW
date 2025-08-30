// ------------------------
// NEMA17 스텝모터 제어 핀
// ------------------------
const int dirPin = 4;
const int stepPin = 3;
const int enablePin = 5;

// ------------------------
// DC 모터 제어 (L298N, L293D 등)
// ------------------------
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 6; // PWM 핀

// ---- 가변저항 (스로틀) ----
const int POT = A0;
const int ADC_MAX = 1023;
const int DEAD_PCT = 2; // 2% 데드밴드

// ---- 시리얼 ----
const unsigned long BAUD = 115200; // 파이썬과 동일하게

// ---- 상태 ----
char command = 'S'; // 기본: 정지
bool dc_on = false; // DC 모터 on/off 토글 상태
int dc_pwm = 0;     // ENA에 쓸 듀티(0~255)

// 스텝 파라미터 (원샷)
const int STEP_COUNT_ONE_REV = 200; // 1.8° 스텝 기준 200 = 1회전
int step_half_us = 800;             // 펄스 하프주기

// ------------------------
// 2색 LED (RED/BLUE)
// ------------------------
const int LED_R = 10;
const int LED_B = 11;
// 공통 캐소드(CC)=true, 공통 애노드(CA)=false
const bool LED_COMMON_CATHODE = true;

inline void ledWrite(int pin, bool on) {
  if (LED_COMMON_CATHODE) digitalWrite(pin, on ? LOW : HIGH);
  else                    digitalWrite(pin, on ? HIGH : LOW);
}
inline void ledOffAll() { ledWrite(LED_R, false); ledWrite(LED_B, false); }
inline void ledRed()    { ledOffAll(); ledWrite(LED_R, true); }
inline void ledBlue()   { ledOffAll(); ledWrite(LED_B, true); }

void setLedByPred(int pred) {
  if (pred == 0) ledBlue();
  else           ledRed();
}

// LED 프레임 파서용 상태
static String led_line;
static bool led_capturing = false; // 'L' 감지 시부터 개행까지 LED만 캡처

// ------------------------
// 스톨 킥(T####) 관련
// ------------------------
const unsigned long STALL_KICK_MS = 2000; // 스톨 인정 대기/강제 구동 시간(2초)
bool kick_active = false;
unsigned long kick_until_ms = 0;
int kick_pwm = 0;     // 0~255
int last_dir_for_kick = 1; // 1: FWD, -1: REV (최근 방향 유지)

// T#### 파싱용
bool t_capturing = false;
char t_buf[8];  // 최대 5자리(+널), 여유
uint8_t t_len = 0;

void setup() {
  Serial.begin(BAUD);

  // 스텝모터 핀 설정
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // DC 모터 핀 설정
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(POT, INPUT);

  // 2색 LED 핀 설정
  pinMode(LED_R, OUTPUT);
  pinMode(LED_B, OUTPUT);
  ledOffAll();

  // 초기 상태
  digitalWrite(dirPin, HIGH);   // 시계 방향
  digitalWrite(enablePin, LOW); // 스텝모터 활성화(모듈에 따라 반대면 HIGH로)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  Serial.println(F("# Ready: O/F=FWD, I/B=REV, S=Stop, o=DC toggle, N=Step once, E=step enable toggle, LED:0/1/O, T####=stall kick"));
}

void loop() {
  // ===== 1) 시리얼 수신 처리 =====
  while (Serial.available() > 0) {
    char c = Serial.read();

// Serial.available() 루프 안에서, led_capturing 진입 전에 바로 아래 체크 추가
if (c == 'L' || c == 'l') {
  // 다음 문자가 바로 0/1로 오는 단문도 지원
  delayMicroseconds(200);  // 입력 버퍼 settle
  if (Serial.available() > 0) {
    char n = Serial.peek();
    if (n == '0' || n == '1') {
      Serial.read();                // consume digit
      setLedByPred(n == '1' ? 1 : 0);
      Serial.println(n == '1' ? F("# LED=BLUE (1)") : F("# LED=RED (0)"));
      // 남은 개행은 일반 루틴에서 소비
      continue;
    }
  }
  // 아니면 기존 라인 캡처(LED:0/LED:1 등)로
  led_capturing = true;
  led_line = "L";
  continue;
}


    // --- LED 프레임 명령 캡처 모드 ---
    if (led_capturing) {
      if (c == '\n' || c == '\r') {
        parseLedLine(led_line);
        led_line = "";
        led_capturing = false;
      } else {
        if (led_line.length() < 32) led_line += c;
      }
      continue;
    }

    // --- T#### 캡처 모드 (스톨 킥) ---
    if (t_capturing) {
      if (c >= '0' && c <= '9') {
        if (t_len < sizeof(t_buf) - 1) {
          t_buf[t_len++] = c;
          t_buf[t_len] = '\0';
        }
        continue;
      }
      // 숫자 종료(비숫자/개행 등) → 파싱
      handleTFrame();
      t_capturing = false;
      t_len = 0;
      // 비숫자 문자는 이후 일반 명령 처리로 흘림
    }

    // LED 프레임 시작 트리거: 'L'로 시작
    if (c == 'L' || c == 'l') {
      led_capturing = true;
      led_line = "L";
      continue;
    }

    // T 프레임 시작 트리거: 'T' 다음 연속 숫자
    if (c == 'T' || c == 't') {
      t_capturing = true;
      t_len = 0;
      t_buf[0] = '\0';
      continue;
    }

    // --- 기존 한 글자 명령 처리 (대소문자 허용) ---
    char uc = c;
    if ('a' <= c && c <= 'z') uc = c - 32; // to upper

    switch (uc) {
      case 'O': // Forward ON
      case 'F':
        command = 'O';
        dc_on = true;
        last_dir_for_kick = 1;
        Serial.println(F("# DC FWD"));
        break;

      case 'I': // Reverse ON
      case 'B':
        command = 'I';
        dc_on = true;
        last_dir_for_kick = -1;
        Serial.println(F("# DC REV"));
        break;

      case 'S': // Stop
        command = 'S';
        dc_on = false;
        Serial.println(F("# DC STOP"));
        break;

      case 'N': // Step one revolution
        Serial.println(F("# STEP one revolution"));
        stepOneRevolution();
        break;

      case 'E': // Step enable 토글
      {
        int cur = digitalRead(enablePin);
        int next = (cur == LOW) ? HIGH : LOW;
        digitalWrite(enablePin, next);
        Serial.println(next == LOW ? F("# STEP ENABLE") : F("# STEP DISABLE"));
      }
      break;

      default:
        // 소문자 'o' 는 토글용
        if (c == 'o') {
          dc_on = !dc_on;
          if (dc_on && command == 'S') {
            command = (last_dir_for_kick >= 0) ? 'O' : 'I';
          }
          Serial.println(dc_on ? F("# DC ON (toggle)") : F("# DC OFF (toggle)"));
        }
        break;
    }
  }

  // ===== 2) 명령/킥에 따른 DC 모터 제어 =====
  if (kick_active && (long)(millis() - kick_until_ms) >= 0) {
    kick_active = false;
    Serial.println(F("# KICK END"));
  }

  if (kick_active) {
    if (last_dir_for_kick >= 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else                         { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
  } else {
    if (command == 'O' && dc_on)      { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); }
    else if (command == 'I' && dc_on) { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }
    else                              { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
  }

  // 듀티 계산
  int raw = analogRead(POT);
  if (raw < (ADC_MAX * DEAD_PCT / 100)) raw = 0;
  int duty_from_pot = map(raw, 0, ADC_MAX, 0, 255);

  if (kick_active) dc_pwm = kick_pwm;
  else             dc_pwm = (command == 'S' || !dc_on) ? 0 : duty_from_pot;
  analogWrite(ENA, dc_pwm);

  // ===== 3) POT 값을 정수 한 줄로 송신 (파이썬 파싱 쉽게) =====
  Serial.println(raw);

  delay(10); // 100Hz 루프
}

// ------------------------
// LED 프레임 파서
// ------------------------
void parseLedLine(const String& s) {
  String up = s; up.trim(); up.toUpperCase();

  if (up == "L0") { setLedByPred(0); return; }
  if (up == "L1") { setLedByPred(1); return; }

  if (up.startsWith("LED:")) {
    if (up.endsWith(":0") || up.endsWith(":B")) { setLedByPred(0); return; } // 0=BLUE
    if (up.endsWith(":1") || up.endsWith(":R")) { setLedByPred(1); return; } // 1=RED
    if (up.endsWith(":O")) { ledOffAll(); Serial.println(F("# LED=OFF")); return; }
  }
}


// ------------------------
// 스텝모터 한바퀴 회전 함수 (블로킹)
// ------------------------
void stepOneRevolution() {
  digitalWrite(enablePin, LOW); // enable
  for (int i = 0; i < STEP_COUNT_ONE_REV; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(step_half_us);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(step_half_us);
  }
  digitalWrite(enablePin, HIGH); // 절전용 disable
}

// ------------------------
// T#### 처리
// ------------------------
void handleTFrame() {
  if (t_len == 0) return;
  long val = 0;
  for (uint8_t i = 0; i < t_len; ++i) {
    val = val * 10 + (t_buf[i] - '0');
    if (val > 9999) break;
  }
  if (val < 0)    val = 0;
  if (val > 1023) val = 1023;

  if (val == 0) {
    kick_active = false;
    kick_pwm = 0;
    Serial.println(F("# KICK CANCEL (T0)"));
    return;
  }

  int duty = map((int)val, 0, 1023, 0, 255);
  kick_pwm = duty;
  kick_active = true;
  kick_until_ms = millis() + STALL_KICK_MS;

  Serial.print(F("# KICK START raw="));
  Serial.print((int)val);
  Serial.print(F(" duty="));
  Serial.print(duty);
  Serial.print(F(" ms="));
  Serial.println(STALL_KICK_MS);
}
