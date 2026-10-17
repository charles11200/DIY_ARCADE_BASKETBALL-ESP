// =====================================================================
//  Arcade Basketball - ESP32 firmware
//
//  Coin slot (Allan 1/5/10/20 PHP) -> credits, start button -> 2 rounds,
//  A02YYUW UART ultrasonic -> shot detection, MG996R servo -> ball gate,
//  3x MAX7219 32x8 (FC16) -> Time / Score / High Score.
//
//  The ESP32 also runs its own WiFi access point ("ArcadeHoops") and a
//  small REST API at http://192.168.4.1 for the esp32_arcade_app.
//  The app can read status / sensors / sessions and edit settings.
//  It can NOT start a game: games start only from the physical button.
//
//  Everything is non-blocking (millis() only, no delay() in the game).
// =====================================================================

#include <Arduino.h>
#include <MD_Parola.h>
#include <MD_MAX72xx.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>

#define FW_VERSION      "1.1.0"

// ---------------------------------------------------------------------
//  Pins
// ---------------------------------------------------------------------
#define PIN_COIN        4    // INPUT_PULLUP, falling-edge interrupt
#define PIN_START       22   // INPUT_PULLUP, pressed = LOW
#define PIN_BUZZER      32
#define PIN_SERVO       13
#define PIN_US_TX       25   // ESP32 TX2 -> sensor RX
#define PIN_US_RX       35   // ESP32 RX2 <- sensor TX (GPIO35 is input-only)
#define PIN_MAX_DIN     23   // shared by all 3 boards (VSPI MOSI)
#define PIN_MAX_CLK     18   // shared by all 3 boards (VSPI SCK)
#define PIN_CS_TIME     27
#define PIN_CS_SCORE    19
#define PIN_CS_HIGH     21

// ---------------------------------------------------------------------
//  WiFi access point (the phone joins this network, no IP entry needed)
// ---------------------------------------------------------------------
#define AP_SSID         "ArcadeHoops"
#define AP_PASS         "hoops1234"   // WPA2 needs at least 8 characters
#define HTTP_PORT       80            // AP address is always 192.168.4.1

// ---------------------------------------------------------------------
//  Key constants
// ---------------------------------------------------------------------
#define PULSE_DIVISOR        2      // pulses doubled after adding the 100 nF cap
#define PULSE_GAP_MS         180    // in-coin gap ~120 ms; 100 split coins, 300 merged coins
#define DEBOUNCE_MS          30

#define SHOT_MIN_CM          5
#define SHOT_MAX_CM          35
#define SHOT_COOLDOWN_MS     400
// Optional: also require one out-of-range reading between shots, so a ball
// resting in front of the sensor can't score repeatedly. Off = original behavior.
#define SHOT_REARM_ON_CLEAR  false

#define SERVO_REST_ANGLE     98     // gate up (blocks balls) - idle / between rounds
#define SERVO_OPEN_ANGLE     0      // gate down (releases balls) - during rounds

#define DISPLAY_REFRESH_MS   5000   // light redraw
#define DEEP_REINIT_MS       30000  // full begin() re-init

// Game rules (fixed)
#define ROUND1_POINTS        2
#define ROUND1_PASS_SCORE    50
#define ROUND2_POINTS        3

// Editable from the app, saved in flash. These are the defaults.
#define DEFAULT_PESO_PER_CREDIT  20
#define DEFAULT_ROUND1_SECONDS   60
#define DEFAULT_ROUND2_SECONDS   30
#define MIN_ROUND_SECONDS        10
#define MAX_ROUND_SECONDS        300
#define MIN_PESO_PER_CREDIT      1
#define MAX_PESO_PER_CREDIT      1000

// Pre-round / post-round timing
#define LABEL_MAX_MS         8000   // safety cap for the "ROUND x" scroll
#define READY_MS             1000
#define COUNTDOWN_STEP_MS    1000
#define START_MS             800
#define ROUND_GAP_MS         3000
#define GAME_OVER_MS         5000

// Displays
#define HARDWARE_TYPE        MD_MAX72XX::FC16_HW
#define MAX_DEVICES          4
#define DISPLAY_INTENSITY    5
#define SCROLL_SPEED_MS      40
#define BUTTON_DEBOUNCE_MS   30

// Ultrasonic
#define US_BAUD              9600
#define US_DEBUG_MS          500

// Health checks for the two-way inputs (reported by /api/sensors)
#define US_TIMEOUT_MS        1500   // A02YYUW auto mode sends a frame every ~100 ms
#define COIN_STUCK_MS        2000   // a real coin pulse is ~50 ms LOW
#define START_STUCK_MS       10000  // button held LOW this long = stuck / shorted

// Finished games kept in RAM until the app claims them for the leaderboard
#define MAX_SESSIONS         8

// ---------------------------------------------------------------------
//  Types
// ---------------------------------------------------------------------
enum GameState {
  IDLE_CREDITS,
  PRE_ROUND_LABEL,
  PRE_READY,
  PRE_COUNTDOWN,
  PRE_START,
  PLAYING,
  ROUND_GAP_WAIT,
  GAME_OVER_WAIT
};

enum PanelId { PANEL_TIME = 0, PANEL_SCORE = 1, PANEL_HIGH = 2, PANEL_COUNT = 3 };

enum PanelMode { MODE_STATIC, MODE_SCROLL_LOOP, MODE_SCROLL_ONCE };

struct Session {
  uint16_t id;        // 0 = empty slot
  uint32_t score;
  uint8_t  roundReached;
  uint32_t endedMs;
  bool     claimed;
};

// ---------------------------------------------------------------------
//  Function prototypes (PlatformIO compiles main.cpp as plain C++)
// ---------------------------------------------------------------------
void IRAM_ATTR coinISR();
void animateDisplays();
void handleCoins();
void handleBuzzer();
void handleDisplayWatchdog();
void handleUltrasonicUART();
void debugPrintUltrasonic();
void monitorInputLines();
void runStateMachine();
bool readStartButtonPressed();
void enterState(GameState s);
void startRound(uint8_t round);
void endRound();
void onValidDistance(int cm);
void setPanel(PanelId id, const char *text, PanelMode mode, bool force = false);
void setPanelNumber(PanelId id, long value);
void applyPanel(PanelId id);
bool panelOneShotDone(PanelId id);
void lightRefreshDisplays();
void deepReinitDisplays();
void showIdleScreen();
void beep(uint16_t onMs, uint8_t count = 1, uint16_t offMs = 80);
void setGate(int angle);
void saveCredits();
void saveHighScore();
void saveSettings();
void recordSession();
const char *stateName(GameState s);
void setupWeb();
void sendJson(int code, const String &body);
void handleApiStatus();
void handleApiSensors();
void handleApiUltrasonic();
void handleApiSessionsUnclaimed();
void handleApiSessionsClaim();
void handleApiSettingsGet();
void handleApiSettingsPost();
void handleNotFound();

// ---------------------------------------------------------------------
//  Globals
// ---------------------------------------------------------------------
MD_Parola panels[PANEL_COUNT] = {
  MD_Parola(HARDWARE_TYPE, PIN_CS_TIME,  MAX_DEVICES),
  MD_Parola(HARDWARE_TYPE, PIN_CS_SCORE, MAX_DEVICES),
  MD_Parola(HARDWARE_TYPE, PIN_CS_HIGH,  MAX_DEVICES)
};

// Text buffers MUST be static/global: Parola keeps the pointer, it does not copy.
// (A stack buffer caused garbage characters on "ROUND 1" / "ROUND 2".)
static char      panelText[PANEL_COUNT][32];
static PanelMode panelMode[PANEL_COUNT];
static bool      panelDone[PANEL_COUNT];

Servo       gateServo;
int         gateAngle = -1;
Preferences prefs;
WebServer   server(HTTP_PORT);

// Coins (ISR shared)
volatile uint32_t isrPulseCount  = 0;
volatile uint32_t isrLastPulseMs = 0;
volatile bool     isrCoinSeen    = false;
portMUX_TYPE      coinMux        = portMUX_INITIALIZER_UNLOCKED;

uint32_t creditBalance = 0;   // pesos, saved in flash
uint32_t highScore     = 0;   // saved in flash

// Settings, saved in flash
uint32_t pesoPerCredit = DEFAULT_PESO_PER_CREDIT;
uint16_t round1Seconds = DEFAULT_ROUND1_SECONDS;
uint16_t round2Seconds = DEFAULT_ROUND2_SECONDS;

// Game
GameState state         = IDLE_CREDITS;
uint32_t  stateEnteredMs = 0;
uint8_t   currentRound  = 1;
uint32_t  score         = 0;
uint32_t  roundEndMs    = 0;
int       lastShownSecs = -1;
uint8_t   countdownValue = 3;
bool      newHighScore  = false;

// Shot detection
uint32_t lastShotMs  = 0;
bool     shotArmed   = true;

// Ultrasonic parser / debug
uint8_t  usFrame[4];
uint8_t  usIndex          = 0;
uint32_t usBytesSeen      = 0;
uint32_t usValidFrames    = 0;
uint32_t usChecksumFails  = 0;
int      latestDistanceCm = -1;
uint32_t lastUsDebugMs    = 0;
uint32_t usLastByteMs     = 0;
uint32_t usLastValidMs    = 0;

// Buzzer (non-blocking pattern)
uint8_t  buzzRemaining = 0;
uint16_t buzzOnMs      = 0;
uint16_t buzzOffMs     = 0;
bool     buzzIsOn      = false;
uint32_t buzzToggleAt  = 0;

// Display watchdog
uint32_t lastLightRefreshMs = 0;
uint32_t lastDeepReinitMs   = 0;

// Start button
bool     btnStable    = HIGH;
bool     btnLastRead  = HIGH;
uint32_t btnChangedMs = 0;
bool     startSeen       = false;
uint32_t lastStartPressMs = 0;
uint32_t startPressCount  = 0;

// Coin line health
uint32_t coinCount       = 0;   // coins counted since boot
bool     coinLineLow     = false;
uint32_t coinLowSinceMs  = 0;

// Sessions
Session  sessions[MAX_SESSIONS];
uint16_t nextSessionId = 1;
char     bootId[9];

// =====================================================================
//  Setup / loop
// =====================================================================
void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println("[BOOT] Arcade Basketball " FW_VERSION);

  pinMode(PIN_BUZZER, OUTPUT);
  digitalWrite(PIN_BUZZER, LOW);
  pinMode(PIN_START, INPUT_PULLUP);
  pinMode(PIN_COIN, INPUT_PULLUP);

  // Saved values
  prefs.begin("arcade", false);
  creditBalance = prefs.getUInt("credits", 0);
  highScore     = prefs.getUInt("hiscore", 0);
  pesoPerCredit = prefs.getUInt("ppc", DEFAULT_PESO_PER_CREDIT);
  round1Seconds = prefs.getUShort("r1s", DEFAULT_ROUND1_SECONDS);
  round2Seconds = prefs.getUShort("r2s", DEFAULT_ROUND2_SECONDS);
  Serial.printf("[BOOT] credits=%lu highScore=%lu ppc=%lu r1=%us r2=%us\n",
                (unsigned long)creditBalance, (unsigned long)highScore,
                (unsigned long)pesoPerCredit, round1Seconds, round2Seconds);

  // A random id per boot lets the app tell sessions from different boots apart.
  snprintf(bootId, sizeof(bootId), "%08lx", (unsigned long)esp_random());

  // Servo: gate up when idle
  ESP32PWM::allocateTimer(0);
  gateServo.setPeriodHertz(50);
  gateServo.attach(PIN_SERVO, 500, 2400);
  setGate(SERVO_REST_ANGLE);

  // Displays
  for (uint8_t i = 0; i < PANEL_COUNT; i++) {
    panels[i].begin();
    panels[i].setIntensity(DISPLAY_INTENSITY);
    panels[i].displayClear();
    panelText[i][0] = '\0';
    panelMode[i] = MODE_STATIC;
    panelDone[i] = true;
  }

  // Ultrasonic: Serial2 RX=35, TX=25
  Serial2.begin(US_BAUD, SERIAL_8N1, PIN_US_RX, PIN_US_TX);

  // Coin slot
  attachInterrupt(digitalPinToInterrupt(PIN_COIN), coinISR, FALLING);

  setupWeb();

  uint32_t now = millis();
  lastLightRefreshMs = now;
  lastDeepReinitMs   = now;

  enterState(IDLE_CREDITS);
  beep(80, 2);
}

void loop() {
  animateDisplays();
  handleCoins();
  handleBuzzer();
  handleDisplayWatchdog();
  handleUltrasonicUART();
  debugPrintUltrasonic();
  monitorInputLines();
  runStateMachine();
  server.handleClient();
}

// =====================================================================
//  Coins
// =====================================================================
void IRAM_ATTR coinISR() {
  uint32_t now = millis();
  portENTER_CRITICAL_ISR(&coinMux);
  if (now - isrLastPulseMs >= DEBOUNCE_MS) {
    isrPulseCount++;
    isrLastPulseMs = now;
    isrCoinSeen    = true;
  }
  portEXIT_CRITICAL_ISR(&coinMux);
}

// Groups the raw pulses of one coin (pulses closer than PULSE_GAP_MS) into one value.
void handleCoins() {
  uint32_t pulses = 0;

  portENTER_CRITICAL(&coinMux);
  if (isrPulseCount > 0 && (millis() - isrLastPulseMs) >= PULSE_GAP_MS) {
    pulses = isrPulseCount;
    isrPulseCount = 0;
  }
  portEXIT_CRITICAL(&coinMux);

  if (pulses == 0) return;

  uint32_t pesos = (pulses + PULSE_DIVISOR / 2) / PULSE_DIVISOR;
  Serial.printf("[COIN] rawPulses=%lu -> %lu PHP\n", (unsigned long)pulses, (unsigned long)pesos);
  if (pesos == 0) return;

  coinCount++;
  creditBalance += pesos;
  saveCredits();
  beep(60);
  Serial.printf("[COIN] balance=%lu PHP\n", (unsigned long)creditBalance);

  if (state == IDLE_CREDITS) showIdleScreen();
}

void saveCredits()   { prefs.putUInt("credits", creditBalance); }
void saveHighScore() { prefs.putUInt("hiscore", highScore); }

void saveSettings() {
  prefs.putUInt("ppc", pesoPerCredit);
  prefs.putUShort("r1s", round1Seconds);
  prefs.putUShort("r2s", round2Seconds);
}

// =====================================================================
//  Input line health (coin + start button)
//  Both lines idle HIGH through a pull-up. A line held LOW for much longer
//  than a real pulse / press means a short or a stuck switch.
// =====================================================================
void monitorInputLines() {
  uint32_t now = millis();
  bool low = digitalRead(PIN_COIN) == LOW;
  if (low && !coinLineLow) coinLowSinceMs = now;
  coinLineLow = low;
}

// =====================================================================
//  Buzzer (active piezo)
// =====================================================================
void beep(uint16_t onMs, uint8_t count, uint16_t offMs) {
  buzzOnMs      = onMs;
  buzzOffMs     = offMs;
  buzzRemaining = count;
  buzzIsOn      = true;
  buzzToggleAt  = millis() + onMs;
  digitalWrite(PIN_BUZZER, HIGH);
}

void handleBuzzer() {
  if (buzzRemaining == 0) return;
  uint32_t now = millis();
  if ((int32_t)(now - buzzToggleAt) < 0) return;

  if (buzzIsOn) {
    digitalWrite(PIN_BUZZER, LOW);
    buzzIsOn = false;
    buzzRemaining--;
    buzzToggleAt = now + buzzOffMs;
  } else {
    digitalWrite(PIN_BUZZER, HIGH);
    buzzIsOn = true;
    buzzToggleAt = now + buzzOnMs;
  }
}

// =====================================================================
//  Servo gate
// =====================================================================
void setGate(int angle) {
  if (angle == gateAngle) return;
  gateAngle = angle;
  gateServo.write(angle);
  Serial.printf("[GATE] %s (%d deg)\n", angle == SERVO_REST_ANGLE ? "UP" : "DOWN", angle);
}

// =====================================================================
//  Displays
// =====================================================================
void animateDisplays() {
  for (uint8_t i = 0; i < PANEL_COUNT; i++) {
    if (panels[i].displayAnimate()) {
      if (panelMode[i] == MODE_SCROLL_LOOP) {
        panels[i].displayReset();
      } else {
        panelDone[i] = true;
      }
    }
  }
}

// Only re-applies when the content actually changes, so it is safe to call every loop.
void setPanel(PanelId id, const char *text, PanelMode mode, bool force) {
  if (!force && panelMode[id] == mode && strcmp(panelText[id], text) == 0) return;
  strncpy(panelText[id], text, sizeof(panelText[id]) - 1);
  panelText[id][sizeof(panelText[id]) - 1] = '\0';
  panelMode[id] = mode;
  applyPanel(id);
}

void setPanelNumber(PanelId id, long value) {
  char tmp[16];
  snprintf(tmp, sizeof(tmp), "%ld", value);
  setPanel(id, tmp, MODE_STATIC);
}

void applyPanel(PanelId id) {
  MD_Parola &p = panels[id];
  panelDone[id] = false;
  if (panelMode[id] == MODE_STATIC) {
    p.displayText(panelText[id], PA_CENTER, SCROLL_SPEED_MS, 0, PA_PRINT, PA_NO_EFFECT);
  } else {
    p.displayText(panelText[id], PA_LEFT, SCROLL_SPEED_MS, 0, PA_SCROLL_LEFT, PA_SCROLL_LEFT);
  }
  p.displayReset();
}

bool panelOneShotDone(PanelId id) { return panelDone[id]; }

// Light redraw: re-send the MAX7219 config registers (noise can corrupt them)
// and redraw static content. Does not interrupt scrolling text.
void lightRefreshDisplays() {
  for (uint8_t i = 0; i < PANEL_COUNT; i++) {
    MD_MAX72XX *mx = panels[i].getGraphicObject();
    mx->control(MD_MAX72XX::TEST, MD_MAX72XX::OFF);
    mx->control(MD_MAX72XX::DECODE, MD_MAX72XX::OFF);
    mx->control(MD_MAX72XX::SCANLIMIT, ROW_SIZE - 1);
    mx->control(MD_MAX72XX::SHUTDOWN, MD_MAX72XX::OFF);
    panels[i].setIntensity(DISPLAY_INTENSITY);
    if (panelMode[i] == MODE_STATIC) panels[i].displayReset();
  }
}

// Full re-init with begin(), then restore what each panel was showing.
void deepReinitDisplays() {
  for (uint8_t i = 0; i < PANEL_COUNT; i++) {
    panels[i].begin();
    panels[i].setIntensity(DISPLAY_INTENSITY);
    panels[i].displayClear();
    applyPanel((PanelId)i);
  }
  Serial.println("[DISP] deep re-init");
}

void handleDisplayWatchdog() {
  uint32_t now = millis();

  if (now - lastDeepReinitMs >= DEEP_REINIT_MS) {
    // Don't restart a one-shot scroll the state machine is waiting on.
    bool busy = false;
    for (uint8_t i = 0; i < PANEL_COUNT; i++) {
      if (panelMode[i] == MODE_SCROLL_ONCE && !panelDone[i]) busy = true;
    }
    if (!busy) {
      deepReinitDisplays();
      lastDeepReinitMs   = now;
      lastLightRefreshMs = now;
      return;
    }
  }

  if (now - lastLightRefreshMs >= DISPLAY_REFRESH_MS) {
    lightRefreshDisplays();
    lastLightRefreshMs = now;
  }
}

void showIdleScreen() {
  char buf[16];
  snprintf(buf, sizeof(buf), "P%lu", (unsigned long)creditBalance);
  setPanel(PANEL_TIME, buf, MODE_STATIC);

  if (creditBalance >= pesoPerCredit) {
    setPanel(PANEL_SCORE, "PRESS START", MODE_SCROLL_LOOP);
  } else {
    snprintf(buf, sizeof(buf), "INSERT P%lu", (unsigned long)pesoPerCredit);
    setPanel(PANEL_SCORE, buf, MODE_SCROLL_LOOP);
  }

  setPanelNumber(PANEL_HIGH, highScore);
}

// =====================================================================
//  Ultrasonic (A02YYUW, UART auto mode)
//  Frame: [0xFF][high][low][checksum], checksum = (0xFF + high + low) & 0xFF
//  distance mm = high*256 + low ; cm = mm / 10
// =====================================================================
void handleUltrasonicUART() {
  while (Serial2.available() > 0) {
    uint8_t b = (uint8_t)Serial2.read();
    usBytesSeen++;
    usLastByteMs = millis();

    if (usIndex == 0) {
      if (b == 0xFF) usFrame[usIndex++] = b;
      continue;
    }

    usFrame[usIndex++] = b;
    if (usIndex < 4) continue;

    usIndex = 0;
    uint8_t sum = (uint8_t)((usFrame[0] + usFrame[1] + usFrame[2]) & 0xFF);
    if (sum == usFrame[3]) {
      usValidFrames++;
      usLastValidMs = millis();
      int mm = ((int)usFrame[1] << 8) | usFrame[2];
      latestDistanceCm = mm / 10;
      onValidDistance(latestDistanceCm);
    } else {
      usChecksumFails++;
      // Resync: a 0xFF inside the bad frame may be the real header.
      if (usFrame[3] == 0xFF)      { usFrame[0] = 0xFF; usIndex = 1; }
      else if (usFrame[2] == 0xFF) { usFrame[0] = 0xFF; usFrame[1] = usFrame[3]; usIndex = 2; }
    }
  }
}

// Scoring only runs in PLAYING.
void onValidDistance(int cm) {
  bool inRange = (cm >= SHOT_MIN_CM && cm <= SHOT_MAX_CM);

  if (!inRange) {
    shotArmed = true;
    return;
  }
  if (state != PLAYING) return;
  if (SHOT_REARM_ON_CLEAR && !shotArmed) return;

  uint32_t now = millis();
  if (now - lastShotMs < SHOT_COOLDOWN_MS) return;

  lastShotMs = now;
  shotArmed  = false;
  score += (currentRound == 1) ? ROUND1_POINTS : ROUND2_POINTS;
  setPanelNumber(PANEL_SCORE, score);
  if (score > highScore) setPanelNumber(PANEL_HIGH, score);
  Serial.printf("[SHOT] %d cm -> score=%lu\n", cm, (unsigned long)score);
}

void debugPrintUltrasonic() {
  uint32_t now = millis();
  if (now - lastUsDebugMs < US_DEBUG_MS) return;
  lastUsDebugMs = now;
  Serial.printf("[US-DEBUG] bytesSeen=%lu validFrames=%lu checksumFails=%lu latestDistanceCm=%d\n",
                (unsigned long)usBytesSeen, (unsigned long)usValidFrames,
                (unsigned long)usChecksumFails, latestDistanceCm);
}

// =====================================================================
//  Start button
// =====================================================================
bool readStartButtonPressed() {
  uint32_t now = millis();
  bool r = digitalRead(PIN_START);
  if (r != btnLastRead) {
    btnLastRead  = r;
    btnChangedMs = now;
  }
  if (r != btnStable && (now - btnChangedMs) >= BUTTON_DEBOUNCE_MS) {
    btnStable = r;
    if (btnStable == LOW) {   // pressed edge
      startSeen        = true;
      lastStartPressMs = now;
      startPressCount++;
      return true;
    }
  }
  return false;
}

// =====================================================================
//  Game state machine
// =====================================================================
const char *stateName(GameState s) {
  switch (s) {
    case IDLE_CREDITS:    return "IDLE_CREDITS";
    case PRE_ROUND_LABEL: return "PRE_ROUND_LABEL";
    case PRE_READY:       return "PRE_READY";
    case PRE_COUNTDOWN:   return "PRE_COUNTDOWN";
    case PRE_START:       return "PRE_START";
    case PLAYING:         return "PLAYING";
    case ROUND_GAP_WAIT:  return "ROUND_GAP_WAIT";
    case GAME_OVER_WAIT:  return "GAME_OVER_WAIT";
  }
  return "?";
}

void enterState(GameState s) {
  state = s;
  stateEnteredMs = millis();
  Serial.printf("[STATE] -> %s (round %u, score %lu)\n",
                stateName(s), currentRound, (unsigned long)score);

  switch (s) {
    case IDLE_CREDITS:
      setGate(SERVO_REST_ANGLE);
      showIdleScreen();
      break;

    case PRE_ROUND_LABEL: {
      char buf[16];
      snprintf(buf, sizeof(buf), "ROUND %u", currentRound);
      setPanel(PANEL_TIME, buf, MODE_SCROLL_ONCE, true);
      setPanelNumber(PANEL_SCORE, score);
      setPanelNumber(PANEL_HIGH, highScore);
      beep(120);
      break;
    }

    case PRE_READY:
      setPanel(PANEL_TIME, "READY", MODE_STATIC);
      beep(120);
      break;

    case PRE_COUNTDOWN:
      countdownValue = 3;
      setPanelNumber(PANEL_TIME, countdownValue);
      beep(120);
      break;

    case PRE_START:
      setPanel(PANEL_TIME, "GO!", MODE_STATIC);
      beep(400);
      break;

    case PLAYING:
      startRound(currentRound);
      break;

    case ROUND_GAP_WAIT:
      setPanel(PANEL_TIME, "PASS", MODE_STATIC);
      break;

    case GAME_OVER_WAIT:
      setPanel(PANEL_TIME, newHighScore ? "NEW HIGH SCORE" : "GAME OVER", MODE_SCROLL_LOOP);
      setPanelNumber(PANEL_SCORE, score);
      setPanelNumber(PANEL_HIGH, highScore);
      beep(newHighScore ? 150 : 600, newHighScore ? 4 : 1);
      break;
  }
}

void startRound(uint8_t round) {
  uint16_t secs = (round == 1) ? round1Seconds : round2Seconds;
  roundEndMs    = millis() + (uint32_t)secs * 1000UL;
  lastShownSecs = -1;
  lastShotMs    = 0;
  shotArmed     = true;
  setGate(SERVO_OPEN_ANGLE);   // gate down during rounds
  setPanelNumber(PANEL_TIME, secs);
  setPanelNumber(PANEL_SCORE, score);
}

void endRound() {
  setGate(SERVO_REST_ANGLE);   // gate back up after each round
  Serial.printf("[ROUND] round %u over, score=%lu\n", currentRound, (unsigned long)score);

  if (currentRound == 1 && score >= ROUND1_PASS_SCORE) {
    currentRound = 2;
    enterState(ROUND_GAP_WAIT);
    return;
  }

  newHighScore = (score > highScore);
  if (newHighScore) {
    highScore = score;
    saveHighScore();
    Serial.printf("[HISCORE] new high score %lu\n", (unsigned long)highScore);
  }
  recordSession();
  enterState(GAME_OVER_WAIT);
}

// Keeps the finished game until the app claims it. When full, the oldest is replaced.
void recordSession() {
  uint8_t slot = 0;
  for (uint8_t i = 0; i < MAX_SESSIONS; i++) {
    if (sessions[i].id == 0) { slot = i; break; }
    if (sessions[i].id < sessions[slot].id) slot = i;
  }
  sessions[slot].id           = nextSessionId++;
  sessions[slot].score        = score;
  sessions[slot].roundReached = currentRound;
  sessions[slot].endedMs      = millis();
  sessions[slot].claimed      = false;
  Serial.printf("[SESSION] #%u score=%lu\n", sessions[slot].id, (unsigned long)score);
}

void runStateMachine() {
  bool startPressed = readStartButtonPressed();
  uint32_t now     = millis();
  uint32_t elapsed = now - stateEnteredMs;

  switch (state) {
    case IDLE_CREDITS:
      if (startPressed) {
        if (creditBalance >= pesoPerCredit) {
          creditBalance -= pesoPerCredit;
          saveCredits();
          score        = 0;
          currentRound = 1;
          newHighScore = false;
          Serial.printf("[GAME] start, balance now %lu\n", (unsigned long)creditBalance);
          enterState(PRE_ROUND_LABEL);
        } else {
          beep(60, 3, 60);   // not enough credits
        }
      }
      break;

    case PRE_ROUND_LABEL:
      if (panelOneShotDone(PANEL_TIME) || elapsed >= LABEL_MAX_MS) enterState(PRE_READY);
      break;

    case PRE_READY:
      if (elapsed >= READY_MS) enterState(PRE_COUNTDOWN);
      break;

    case PRE_COUNTDOWN: {
      uint8_t step = elapsed / COUNTDOWN_STEP_MS;   // 0,1,2 -> 3,2,1
      if (step >= 3) {
        enterState(PRE_START);
      } else if (3 - step != countdownValue) {
        countdownValue = 3 - step;
        setPanelNumber(PANEL_TIME, countdownValue);
        beep(120);
      }
      break;
    }

    case PRE_START:
      if (elapsed >= START_MS) enterState(PLAYING);
      break;

    case PLAYING: {
      int32_t remainingMs = (int32_t)(roundEndMs - now);
      if (remainingMs <= 0) {
        setPanelNumber(PANEL_TIME, 0);
        beep(700);
        endRound();
        break;
      }
      int secs = (remainingMs + 999) / 1000;
      if (secs != lastShownSecs) {
        lastShownSecs = secs;
        setPanelNumber(PANEL_TIME, secs);
        if (secs <= 3) beep(100);   // last 3 seconds
      }
      break;
    }

    case ROUND_GAP_WAIT:
      if (elapsed >= ROUND_GAP_MS) enterState(PRE_ROUND_LABEL);
      break;

    case GAME_OVER_WAIT:
      if (elapsed >= GAME_OVER_MS) enterState(IDLE_CREDITS);
      break;
  }
}

// =====================================================================
//  WiFi access point + REST API
//
//  GET  /api/status             game state, credits, high score
//  GET  /api/sensors            health of the two-way inputs
//  GET  /api/sensors/ultrasonic live distance reading
//  GET  /api/sessions/unclaimed finished games not yet on the leaderboard
//  POST /api/sessions/claim     id=<n>  -> marks a session claimed
//  GET  /api/settings           round durations, peso per credit
//  POST /api/settings           round1Seconds / round2Seconds / pesoPerCredit
//
//  Output-only parts (buzzer, gate servo, MAX7219 boards) can't report
//  back, so they are not listed. There is no endpoint that starts a game.
// =====================================================================
void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setSleep(false);
  Serial.printf("[WIFI] AP \"%s\" at %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  server.on("/api/status",             HTTP_GET,  handleApiStatus);
  server.on("/api/sensors",            HTTP_GET,  handleApiSensors);
  server.on("/api/sensors/ultrasonic", HTTP_GET,  handleApiUltrasonic);
  server.on("/api/sessions/unclaimed", HTTP_GET,  handleApiSessionsUnclaimed);
  server.on("/api/sessions/claim",     HTTP_POST, handleApiSessionsClaim);
  server.on("/api/settings",           HTTP_GET,  handleApiSettingsGet);
  server.on("/api/settings",           HTTP_POST, handleApiSettingsPost);
  server.onNotFound(handleNotFound);
  server.begin();
}

void sendJson(int code, const String &body) {
  server.sendHeader("Cache-Control", "no-store");
  server.send(code, "application/json", body);
}

// -1 when the event never happened since boot.
static long msAgo(bool seen, uint32_t whenMs) {
  return seen ? (long)(millis() - whenMs) : -1L;
}

void handleApiStatus() {
  uint32_t now = millis();
  bool inProgress = (state != IDLE_CREDITS);
  long timeLeftS = 0;
  if (state == PLAYING) {
    int32_t remainingMs = (int32_t)(roundEndMs - now);
    timeLeftS = remainingMs > 0 ? (remainingMs + 999) / 1000 : 0;
  }
  uint8_t unclaimed = 0;
  for (uint8_t i = 0; i < MAX_SESSIONS; i++) {
    if (sessions[i].id != 0 && !sessions[i].claimed) unclaimed++;
  }

  char buf[512];
  snprintf(buf, sizeof(buf),
    "{\"device\":\"%s\",\"firmware\":\"%s\",\"bootId\":\"%s\",\"uptimeMs\":%lu,"
    "\"state\":\"%s\",\"inProgress\":%s,\"round\":%u,\"score\":%lu,\"timeLeftS\":%ld,"
    "\"highScore\":%lu,\"balancePesos\":%lu,\"pesoPerCredit\":%lu,\"credits\":%lu,"
    "\"nextCreditPesos\":%lu,\"gate\":\"%s\",\"unclaimedSessions\":%u}",
    AP_SSID, FW_VERSION, bootId, (unsigned long)now,
    stateName(state), inProgress ? "true" : "false", inProgress ? currentRound : 0,
    (unsigned long)(inProgress ? score : 0), timeLeftS,
    (unsigned long)highScore, (unsigned long)creditBalance, (unsigned long)pesoPerCredit,
    (unsigned long)(creditBalance / pesoPerCredit),
    (unsigned long)(creditBalance % pesoPerCredit),
    gateAngle == SERVO_REST_ANGLE ? "up" : "down", unclaimed);
  sendJson(200, buf);
}

static const char *ultrasonicStatus(const char **detail) {
  uint32_t now = millis();
  if (usValidFrames > 0 && now - usLastValidMs < US_TIMEOUT_MS) {
    *detail = "Valid frames arriving";
    return "connected";
  }
  if (usBytesSeen > 0 && now - usLastByteMs < US_TIMEOUT_MS) {
    *detail = "Data arriving but frames are invalid (check baud / TX-RX)";
    return "fault";
  }
  *detail = "No data from sensor (check wiring, 5 V and ground)";
  return "disconnected";
}

void handleApiSensors() {
  uint32_t now = millis();

  // Coin slot
  bool coinSeen;
  uint32_t coinLastMs;
  portENTER_CRITICAL(&coinMux);
  coinSeen   = isrCoinSeen;
  coinLastMs = isrLastPulseMs;
  portEXIT_CRITICAL(&coinMux);
  const char *coinStatus;
  const char *coinDetail;
  if (coinLineLow && now - coinLowSinceMs >= COIN_STUCK_MS) {
    coinStatus = "fault";
    coinDetail = "Coin line held LOW (short or stuck signal)";
  } else if (coinSeen) {
    coinStatus = "connected";
    coinDetail = "Coin pulses received";
  } else {
    coinStatus = "untested";
    coinDetail = "No coin since boot. Insert a coin to verify";
  }

  // Start button
  const char *startStatus;
  const char *startDetail;
  if (btnStable == LOW && now - btnChangedMs >= START_STUCK_MS) {
    startStatus = "fault";
    startDetail = "Button held LOW (stuck or shorted)";
  } else if (startSeen) {
    startStatus = "connected";
    startDetail = "Button presses received";
  } else {
    startStatus = "untested";
    startDetail = "No press since boot. Press START to verify";
  }

  // Ultrasonic
  const char *usDetail;
  const char *usStatus = ultrasonicStatus(&usDetail);

  char buf[1024];
  snprintf(buf, sizeof(buf),
    "{\"uptimeMs\":%lu,\"sensors\":["
    "{\"id\":\"coin\",\"name\":\"Coin Slot\",\"pin\":%d,\"status\":\"%s\",\"detail\":\"%s\","
    "\"lastSignalMsAgo\":%ld,\"count\":%lu},"
    "{\"id\":\"start\",\"name\":\"Start Button\",\"pin\":%d,\"status\":\"%s\",\"detail\":\"%s\","
    "\"lastSignalMsAgo\":%ld,\"count\":%lu},"
    "{\"id\":\"ultrasonic\",\"name\":\"Ultrasonic Sensor\",\"pin\":%d,\"status\":\"%s\",\"detail\":\"%s\","
    "\"lastSignalMsAgo\":%ld,\"count\":%lu,\"distanceCm\":%d,\"checksumFails\":%lu}"
    "]}",
    (unsigned long)now,
    PIN_COIN, coinStatus, coinDetail, msAgo(coinSeen, coinLastMs), (unsigned long)coinCount,
    PIN_START, startStatus, startDetail, msAgo(startSeen, lastStartPressMs),
    (unsigned long)startPressCount,
    PIN_US_RX, usStatus, usDetail, msAgo(usBytesSeen > 0, usLastByteMs),
    (unsigned long)usValidFrames, latestDistanceCm, (unsigned long)usChecksumFails);
  sendJson(200, buf);
}

// The sensor streams every ~100 ms, so the latest frame is a live reading.
void handleApiUltrasonic() {
  const char *detail;
  const char *status = ultrasonicStatus(&detail);
  bool fresh = usValidFrames > 0 && millis() - usLastValidMs < US_TIMEOUT_MS;

  char buf[256];
  snprintf(buf, sizeof(buf),
    "{\"status\":\"%s\",\"detail\":\"%s\",\"fresh\":%s,\"distanceCm\":%d,\"ageMs\":%ld,"
    "\"shotMinCm\":%d,\"shotMaxCm\":%d}",
    status, detail, fresh ? "true" : "false", fresh ? latestDistanceCm : -1,
    msAgo(usValidFrames > 0, usLastValidMs), SHOT_MIN_CM, SHOT_MAX_CM);
  sendJson(200, buf);
}

void handleApiSessionsUnclaimed() {
  uint32_t now = millis();
  String body;
  body.reserve(512);
  body += "{\"bootId\":\"";
  body += bootId;
  body += "\",\"sessions\":[";
  bool first = true;
  for (uint8_t i = 0; i < MAX_SESSIONS; i++) {
    const Session &s = sessions[i];
    if (s.id == 0 || s.claimed) continue;
    char item[128];
    snprintf(item, sizeof(item),
      "%s{\"id\":%u,\"score\":%lu,\"roundReached\":%u,\"endedAgoS\":%lu}",
      first ? "" : ",", s.id, (unsigned long)s.score, s.roundReached,
      (unsigned long)((now - s.endedMs) / 1000));
    body += item;
    first = false;
  }
  body += "]}";
  sendJson(200, body);
}

void handleApiSessionsClaim() {
  if (!server.hasArg("id")) {
    sendJson(400, "{\"ok\":false,\"error\":\"missing id\"}");
    return;
  }
  long id = server.arg("id").toInt();
  for (uint8_t i = 0; i < MAX_SESSIONS; i++) {
    if (sessions[i].id != 0 && sessions[i].id == id) {
      sessions[i].claimed = true;
      sendJson(200, "{\"ok\":true}");
      return;
    }
  }
  sendJson(404, "{\"ok\":false,\"error\":\"session not found\"}");
}

void handleApiSettingsGet() {
  char buf[192];
  snprintf(buf, sizeof(buf),
    "{\"round1Seconds\":%u,\"round2Seconds\":%u,\"pesoPerCredit\":%lu,"
    "\"minRoundSeconds\":%d,\"maxRoundSeconds\":%d,\"minPesoPerCredit\":%d,\"maxPesoPerCredit\":%d}",
    round1Seconds, round2Seconds, (unsigned long)pesoPerCredit,
    MIN_ROUND_SECONDS, MAX_ROUND_SECONDS, MIN_PESO_PER_CREDIT, MAX_PESO_PER_CREDIT);
  sendJson(200, buf);
}

// Accepts any subset of the three fields. Nothing is saved if one is out of range.
// New round durations apply from the next round that starts.
void handleApiSettingsPost() {
  long r1  = server.hasArg("round1Seconds") ? server.arg("round1Seconds").toInt() : round1Seconds;
  long r2  = server.hasArg("round2Seconds") ? server.arg("round2Seconds").toInt() : round2Seconds;
  long ppc = server.hasArg("pesoPerCredit") ? server.arg("pesoPerCredit").toInt() : pesoPerCredit;

  if (r1 < MIN_ROUND_SECONDS || r1 > MAX_ROUND_SECONDS ||
      r2 < MIN_ROUND_SECONDS || r2 > MAX_ROUND_SECONDS) {
    sendJson(400, "{\"ok\":false,\"error\":\"round duration must be 10-300 s\"}");
    return;
  }
  if (ppc < MIN_PESO_PER_CREDIT || ppc > MAX_PESO_PER_CREDIT) {
    sendJson(400, "{\"ok\":false,\"error\":\"peso per credit must be 1-1000\"}");
    return;
  }

  round1Seconds = (uint16_t)r1;
  round2Seconds = (uint16_t)r2;
  pesoPerCredit = (uint32_t)ppc;
  saveSettings();
  Serial.printf("[SETTINGS] r1=%us r2=%us ppc=%lu\n",
                round1Seconds, round2Seconds, (unsigned long)pesoPerCredit);
  if (state == IDLE_CREDITS) showIdleScreen();
  handleApiSettingsGet();
}

void handleNotFound() {
  sendJson(404, "{\"ok\":false,\"error\":\"not found\"}");
}
