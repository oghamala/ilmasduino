/* TODO:

 *  * GITHUB KÄYTTÖÖN --- OK

 *  * TUULETTIMEN LOGIIKKA UUSIKSI (katso tarkemmat huomiot sieltä funktiosta) Tarkista myös jääkö turhia muuttujia --- OK

 *  * 100 - 1000 ms TAUKO VAIHTOJEN VÄLIIN (Sähköhäiriöiden minimointi) --- OK

 *  * ASETUKSET EEPROMILLE --- OK

 *  * ASETTUSRUUTU --- OK

 *  * BME280 HEITTÄÄ ERRORIN JOKA TOISELLA BOOTILLA - KORJAA? --- lisätty aikaa ja toimii vähän paremmim, muttei vieläkää 100%

 *  * NAPPEIHIN "LONGPRESS"-TOIMINTO

 *  * HISTORIARUUTU (24H Alkuun muistitaulukosta)
      > Myös arvojen aaltoilu nähtävä keskiarvojen lisäksi, että releronklaus saadaan parametreja viilaamalla minimiin

 *  * WS2812 - statusvalot (ja virransäästö näyttöön?)
      > Erilaisia värikoodauksia erroreille ja tavoitearvoissa pysymiselle - ehkä vähän sellain pidemmältäkin jaksolta?
        - Esim. <90% viim. vrk tavoitearvossa vihreä (ja maks. poikkeama pieni) / yli 60% keltainen / yli 30% oranssi / <30% (ja/tai maks. poikkeama iso) punainen
        - Esim. Alle 12 vaihteen vaihtoa päivässä = vihreä / <24 keltainen / <48 oranssi / >48 punainen

 *  * RELELOGIIKAN TARKISTUS - Onko nykyinen vikasietoinen?
      > Mitä jos arduinosta katkeaa virta?
      > Mitä jos jokin rele hirttää kiinni?
      > Entäs jos relekortista katkeaa virta?


 *  * GRAAFINEN ESITYSMUOTO HISTORIALLE (Custom characters)

 *  * EEPROMILLE JÄRKEVÄ FORMAATTI (kts. seuraava)
      > Tilaa asetuksille (ja grafiikkaelementeille?)
      > Varaus myös toiseelle BME280-anturille
      > (Varaus volttiarvoille /) virheilmoituksille
        -Vikakoodeja ehkä vain yksi tuntiin logiin tai jonkinlainen koodaus, että mikä koodin aiheutti että kaikki mahdolliset saa 1-2 tavuun
      > Ehkä jopa laskee releiden kulumista ja ilmoittaa kun alkaa olla aika vaihtaa??? - Jos siis saa järkevästi jotenkin ilman, että tarvii laskea ja kirjata joka naksu
        -Ehkä joku koodaus että 5-10v tulee kevyelläkin täyteen ja kirjaa päivä/viikkotasolla kevyt/keskiraskas/raskas/äärimmäinen
      > Tiedostojärjestelmän formaattiversio kirjataan ylös ja kysyttäessä formatoidaan, jos vanha formaatti ei yhteensopiva/tunnistettu
        -Hallittu siirtymä uuteen formaattiin!!! (vaikka jollain apuohjelmalla tai Serial-yhteyden avulla toteutetulla siirtoformatoinnilla)

 *  * PIDEMMÄN AJAN HISTORIA EEPROMILLE (Rengaspuskuri)
      > Mahdollisuus tyhjentämiseen ja siirtoon koneelle?
        - Älä kuitenkaan nollaa reledataa samalla


 *  * VOLTTIMITTARI (RELEIDEN TOIMINNAN SEURANTA)
      >Seuraa, että vaiteet vaihtuu oikein ja poistaa tarvittaessa rikkinäisen releen käytöstä & tallentaa vikakoodin.

 *  * Bluetooth-moduuli toisen BM280:n tilalle tai rinnalle

*/





#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Adafruit_BME280.h>
#include <GPTbounce.h>
#include <EEPROM.h>

#define VERSION_MAJOR 0
#define VERSION_MINOR 3
#define VERSION_PATCH 1

/* =========================================================================
   I2C-laitteet
   -------------------------------------------------------------------------
   BME280 on juotettu "toiseen" osoitteeseen -> 0x76 (vakio 0x77).
   hd44780_I2Cexp driveri SKANNAA LCD:n osoitteen automaattisesti, eli
   erillistä LCD-osoitemakroa ei tarvita.
   ========================================================================= */
#define BME280_ADDRESS  0x76    // Vaihda 0x77 jos anturisi on vakio-osoitteessa

/* =========================================================================
   LCD 20x4
   ========================================================================= */
hd44780_I2Cexp lcd;


/* =========================================================================
   BME280 anturi
   ========================================================================= */
Adafruit_BME280 bme;

bool bmeOK = false;

// Mittausarvot
float currentTempC = 0.0;      // Mitattu lämpötila (C)
float currentHumRH = 0.0;      // Mitattu kosteus (%RH)

// Kuinka usein luetaan anturi
const unsigned long SENSOR_INTERVAL_MS = 2000UL;    // 2 s

unsigned long lastSensorTime      = 0;

/* =========================================================================
   NAPPULAT (GPTbounce)
   -------------------------------------------------------------------------
   Neljä nappia, jotka on kytketty maahan, ja käytetään INPUT_PULLUP:ia.
   - nappi painettuna -> pinni LOW
   - nappi levossa    -> pinni HIGH

   GPTbounce hoitaa millis-pohjaisen debouncen ja reunantunnistuksen:
   - .fell()  -> yksi tapahtuma per oikea painallus (HIGH -> LOW)
   - .rose()  -> vapautus (LOW -> HIGH)
   - .isPressed() -> stabiili tila LOW (pohjassa)
   ========================================================================= */


const uint8_t BTN_BACK_PIN    = A1;  // Nappi: Peruuta
const uint8_t BTN_MINUS_PIN   = A0;  // Nappi: Pienennä/Ylös valikossa
const uint8_t BTN_PLUS_PIN    = A3;  // Nappi: Suurenna/Alas valikossa
const uint8_t BTN_ENTER_PIN   = A2;  // Nappi: Enter

// Nappien käsittely tekoälyn luomalla kirjastolla
GPTbounce btnBack;
GPTbounce btnPlus;
GPTbounce btnMinus;
GPTbounce btnEnter;

const double SETPOINT_MIN_C = 5.0;
const double SETPOINT_MAX_C = 35.0;

const double HYST_MIN_C     = 0.1;
const double HYST_MAX_C     = 5.0;

const uint8_t RELAY_MIN_DELAY = 1;
const uint8_t RELAY_MAX_DELAY = 60;

/* =========================================================================
   RELEKORTTI / PUHALTIMEN PORTAAT
   -------------------------------------------------------------------------
   8-kanavainen relekortti, joista 6 kpl käytetään muuntajasäätimelle.
   Tässä oletetaan:
   - Relet kortti on "active LOW": pinni LOW -> rele vetää, HIGH -> rele pois.
   Jos korttisi toimii päinvastoin, muuta RELAY_ACTIVE_LEVEL ja RELAY_INACTIVE_LEVEL.
   ========================================================================= */

// Käytettävissä olevien releiden määrä

const uint8_t totalRelaySteps = 6;


const uint8_t relayPins[totalRelaySteps] = {
  6,  // Porras 1
  7,  // Porras 2
  8,  // Porras 3
  9,  // Porras 4
  10, // Porras 5
  11  // Porras 6
};

#define RELAY_ACTIVE_LEVEL    LOW
#define RELAY_INACTIVE_LEVEL  HIGH

// Nykyinen "portaallinen teho" 0–6 (0 = puhallin pois)
int currentFanStep = 0;

/* =========================================================================
   LÄMPÖTILA-ASETUS JA SÄÄDÖN INTERVALLI
   ========================================================================= */

const double EmergencyThresholdC = 4.0;

struct Settings {
  float setpointC;
  float hysteresisC;
  uint8_t relayDelay; // minuutteina
  uint8_t allowFanOff;
  uint8_t versionMajor;
  uint8_t versionMinor;
  uint8_t versionPatch;
  uint8_t crc;
};

Settings cfg = {
  25.0,
  1.0,
  1,
  VERSION_MAJOR,
  VERSION_MINOR,
  VERSION_PATCH,
  0
};


// Kuinka usein arvioidaan uusi "haluttu porras"
// (tässä ei vielä vaihdeta releitä, vain lasketaan logiikkaa)
const unsigned long CONTROL_INTERVAL_MS = 60000UL;  // 1 min (Vaikuttaa hätäsäädön jojoilunesto-toimintoon. Aiempi 10s nollasi sen ehkä vähän turhan helposti)

inline unsigned long getRelayDelayMs() {
  return (unsigned long)cfg.relayDelay * 60000UL;
}

// Kumpaan suuntaan viimeisin säätö tehtiin, jotta estetään jojoilu hätäsäädössä
int lastEmergencyRelayChangeDirection = 0;

unsigned long lastControlTime     = 0;
unsigned long lastRelayChangeTime = 0;

/* =========================================================================
   ASETUKSET LCD:N PÄIVITYSVÄLI
   ========================================================================= */
const unsigned long LCD_UPDATE_INTERVAL_MS = 1000;
unsigned long lastLcdUpdateTime = 0;

/* =========================================================================
   GUI
   ========================================================================= */
// --- Näkymät ---
enum UiScreen {
  SCREEN_HOME = 0,   // kotiruutu
  SCREEN_MENU,       // asetuslista
  SCREEN_HISTORY     // historiaruutu (stub toistaiseksi)
};

UiScreen uiScreen = SCREEN_HOME;

// --- Menun asetukset ---

const uint8_t MENU_VISIBLE_ROWS = 4;
const uint8_t MENU_VALUE_COL    = 14;

// --- Ylimmän näkyvän menukohtan indeksi (scroll-ikkuna) ---
uint8_t menuFirstVisible = 0;

// --- Menun rivit ---
enum MenuItem {
  MENU_ITEM_HISTORY = 0,   // "Siirry Historia-ruutuun"
  MENU_ITEM_SETPOINT,      // tavoitelämpö
  MENU_ITEM_HYSTERESIS,    // hystereesi
  MENU_ITEM_MIN_DELAY,     // min. releviive
  MENU_ITEM_ALLOW_FAN_OFF  // Salli tuulettimen sammutus
};

uint8_t menuIndex = 0;      // mikä rivi valittuna listassa

// --- Editointitila ---
bool menuEditMode = false;  // false = selaus, true = muokataan numeroarvoa

double editValue = 0.0;     // väliaikainen arvo double-tyyppisille (setpoint, hyst)

// --- Menu-otsikot PROGMEMissa (ei syö RAMia) ---
const char m0[] PROGMEM = "Historia";         // MENU_ITEM_HISTORY
const char m1[] PROGMEM = "Tavoite";          // MENU_ITEM_SETPOINT
const char m2[] PROGMEM = "Vaihtelu";         // MENU_ITEM_HYSTERESIS
const char m3[] PROGMEM = "Releviive";        // MENU_ITEM_MIN_DELAY
const char m4[] PROGMEM = "Salli Fan Off";    // MENU_ITEM_ALLOW_FAN_OFF

const char* const menuItems[] PROGMEM = {
  m0, m1, m2, m3, m4
};

const uint8_t MENU_ITEM_COUNT = sizeof(menuItems) / sizeof(menuItems[0]);

char lcdLine[21]; // 20 merkkiä + 0-terminaattori

bool blinkOn = true;
unsigned long lastBlinkToggle = 0;
const unsigned long BLINK_INTERVAL_MS = 500;


// --- LCD:n päivitys ---
bool needFullRedraw  = true;  // koko ruutu uusiksi (layout + arvot)
bool needValuesRedraw = true; // vain dynaamiset arvot / nuoli

unsigned long now = 0; // Aika-muuttuja


/* =========================================================================
   ALUSTUS
   ========================================================================= */
void setup() {
  Serial.begin(9600);
  delay(100);

  Wire.begin(); // Nano: A4 = SDA, A5 = SCL (vakiot I2C-pinnit)

  // LCD
  lcd.begin(20, 4);       // leveys, rivit
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Ilmasduino v"));
  lcd.print(VERSION_MAJOR);
  lcd.print(F("."));
  lcd.print(VERSION_MINOR);
  lcd.print(F("."));
  lcd.print(VERSION_PATCH);
  lcd.setCursor(0, 1);
  lcd.print(F("Kaynnistyy..."));

  // BME280
  lcd.setCursor(0, 2);
  lcd.print(F("BME280"));
  for (int i = 0; i < 5; i++) {
    if (bme.begin(BME280_ADDRESS)) {
      bmeOK = true;
      break;
    }
    lcd.print(F("."));
    delay(1000);
  }
  if (bmeOK) {
    // Asetetaan BME280 forced-moodiin:
    // - MODE_FORCED: ei jatkuvaa mittausta, vaan yksi mittaus kerrallaan
    // - Kohtuullinen oversampling -> ei turhaa kuumenemista
    bme.setSampling(
      Adafruit_BME280::MODE_FORCED,        // sensori mittaa vain kun käsketään
      Adafruit_BME280::SAMPLING_X2,        // lämpötilan oversampling (riittävän tarkka)
      Adafruit_BME280::SAMPLING_X1,        // kosteus
      Adafruit_BME280::SAMPLING_X1,        // paine (ei käytössä, mutta pakollinen parametri)
      Adafruit_BME280::FILTER_OFF          // ei sisäistä suodatinta -> nopea reaktio
    );

    lcd.print(F(" OK! "));
    lcd.print(F("0x"));
    lcd.print(BME280_ADDRESS, HEX);
  } else {
    lcd.print(F("VIRHE!"));
    while (1);
  }

  // Nappulat INPUT_PULLUP
  btnBack.begin(BTN_BACK_PIN, 25);     // 25 ms debounce
  btnPlus.begin(BTN_PLUS_PIN, 25);
  btnMinus.begin(BTN_MINUS_PIN, 25);
  btnEnter.begin(BTN_ENTER_PIN, 25);

  // Releet
  for (uint8_t i = 0; i < totalRelaySteps; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], RELAY_INACTIVE_LEVEL); // alkuun kaikki pois
  }

  if (!loadSettings()) {
    saveSettings(); // kirjoita oletukset EEPROMiin koska CRC puuttuu
  }

  delay(5000);
  lcd.clear();
}

/* =========================================================================
   PÄÄSILMUKKA
   ========================================================================= */
void loop() {
  now = millis();

  updateButtons();

  handleUI();

  handleSensor();

  handleFanControl();

  // updateHistory();

  updateDisplay();

}

/* =========================================================================
   SENSORIN LUKU
   ========================================================================= */
void handleSensor() {
  if (now - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = now;
    if (!bmeOK) {
      // Ei anturia -> jätetään arvot ennalleen
      return;
    }
    // FORCE-tila:
    // 1) Käsketään sensoria tekemään yksi mittaus
    // 2) takeForcedMeasurement() blokkaa kunnes data on valmis
    // 3) Tämän ansiosta sensori ei rouski mittauksia koko ajan taustalla (ja lämmitä itseään)

    bme.takeForcedMeasurement();

    currentTempC = bme.readTemperature();
    currentHumRH = bme.readHumidity();
  }
}

/* =========================================================================
   TUULETTIMEN SÄÄTÖ
   ========================================================================= */

void handleFanControl() {
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = now;

    // Erotus setpointtiin
    double diff = currentTempC - cfg.setpointC;  // >0 = liian kuuma, <0 = liian kylmä

    // 1) Tarkastetaan säädön tarve
    int requestedChange = 0;

    if (diff > EmergencyThresholdC && lastEmergencyRelayChangeDirection >= 0) {
      requestedChange = 2;
      lastEmergencyRelayChangeDirection = 1;
    }
    else if (diff > cfg.hysteresisC) {
      requestedChange = 1;
    }
    else if (diff < (EmergencyThresholdC * -1) && lastEmergencyRelayChangeDirection <= 0) {
      requestedChange = -2;
      lastEmergencyRelayChangeDirection = -1;
    }
    else if (diff < (cfg.hysteresisC * -1)) {
      requestedChange = -1;
    }
    else {
      requestedChange = 0;
      lastEmergencyRelayChangeDirection = 0;
    }

    // 2) Tarkastetaan ettei mennä säätörajan ulkopuolelle

    int targetStep = currentFanStep + requestedChange;

    int minStep = cfg.allowFanOff ? 0 : 1;

    if (targetStep < minStep)        targetStep = minStep;
    if (targetStep > totalRelaySteps) targetStep = totalRelaySteps;

    // Jos logiikka ei halua muutosta (ja clämpäys ei muuta mitään), ei tehdä mitään
    if (targetStep == currentFanStep) {
      return;
    }



    // 3) Tarkastetaan onko kulunut tarpeeksi aikaa viimeisimmästä säädöstä
    bool enoughTimePassed = (now - lastRelayChangeTime) >= getRelayDelayMs();

    // 4) Tehdään säätö tarvittaessa


    if (enoughTimePassed && currentFanStep != targetStep) {
      currentFanStep = targetStep;

      applyFanStep(currentFanStep);  // pyydetään alifunktiota suorittamaan säätö
      lastRelayChangeTime = now;
    }
  }
}

/* =========================================================================
   RELEIDEN OHJAUS PORTAISIIN
   -------------------------------------------------------------------------
   TÄMÄ ON OLETUSLOGIIKKA:
   - step = 0  -> kaikki releet pois = puhallin pois
   - step = 1  -> relePins[0] päällä
   - step = 2  -> relePins[1] päällä
   - ...
   - step = 6  -> relePins[5] päällä
   -------------------------------------------------------------------------
   Jos muuntajasäätimesi tarvitsee jonkin toisen kombinaation (esim. vain
   yksi rele kerrallaan saa olla päällä, tai tietyt parit päälle samaan aikaan),
   MUUTA tätä funktiota vastaavasti.
   ========================================================================= */
void applyFanStep(int step) {
  // Aina ensin kaikki releet pois
  for (uint8_t i = 0; i < totalRelaySteps; i++) {
    digitalWrite(relayPins[i], RELAY_INACTIVE_LEVEL);
  }

  // step 0 -> kaikki pois, ei tarvi tehdä mitään
  if (step == 0) {
    return;
  }

  // Varoaika kytkentöjen väliin
  delay(500);

  // Muuten kytke yksi rele päälle (step-1)
  uint8_t idx = step - 1;
  if (idx < totalRelaySteps) {
    digitalWrite(relayPins[idx], RELAY_ACTIVE_LEVEL);
  }
}

/* =========================================================================
   NAPPIEN KÄSITTELY (ASETUKSET)
   -------------------------------------------------------------------------
   - GPTbounce hoitaa debouncen.
   - Tässä luetaan vain .fell() = “yksi klikki” per painallus.
   - Pitkiä painalluksia tai autorepeatia voi lisätä myöhemmin .isPressed():lla.
   ========================================================================= */
void updateButtons() {

  btnBack.update();
  btnPlus.update();
  btnMinus.update();
  btnEnter.update();

}

void handleUI() {
  // Luetaan yhden kierroksen "edge"-tapahtumat
  bool backPressed  = btnBack.fell();
  bool plusPressed  = btnPlus.fell();
  bool minusPressed = btnMinus.fell();
  bool enterPressed = btnEnter.fell();

  switch (uiScreen) {
    case SCREEN_HOME:
      handleUiHome(backPressed, plusPressed, minusPressed, enterPressed);
      break;

    case SCREEN_MENU:
      handleUiMenu(backPressed, plusPressed, minusPressed, enterPressed);
      break;

    case SCREEN_HISTORY:
      handleUiHistory(backPressed, plusPressed, minusPressed, enterPressed);
      break;
  }
}

void handleUiHome(bool back, bool plus, bool minus, bool enter) {
  // Back ei tee kotiruudussa mitään (voi lisätä myöhemmin jonkun toiminnon)

  if (minus) {
    cfg.setpointC -= 0.5;
    if (cfg.setpointC < SETPOINT_MIN_C) cfg.setpointC = SETPOINT_MIN_C;
    needValuesRedraw = true;   // vain arvot
  }

  if (plus) {
    cfg.setpointC += 0.5;
    if (cfg.setpointC > SETPOINT_MAX_C) cfg.setpointC = SETPOINT_MAX_C;
    needValuesRedraw = true;
  }

  if (enter) {
    uiScreen = SCREEN_MENU;
    menuIndex = 0;
    menuEditMode = false;
    blinkOn = true;
    lastBlinkToggle = now;

    menuFirstVisible = calcMenuFirstVisible(); // alusta scroll-ikkuna
    needFullRedraw  = true;                    // piirretään koko menuruutu
  }
}

void handleUiMenu(bool back, bool plus, bool minus, bool enter) {
  if (!menuEditMode) {
    // --- SELAUSMoodi ---
    if (back) {
      uiScreen = SCREEN_HOME;
      needFullRedraw = true;
      return;
    }

    bool indexChanged = false;

    // MINUS = "ylös" listalla
    if (minus) {
      if (menuIndex == 0) {
        menuIndex = MENU_ITEM_COUNT - 1;
      } else {
        menuIndex--;
      }
      indexChanged = true;
    }

    // PLUS = "alas" listalla
    if (plus) {
      menuIndex++;
      if (menuIndex >= MENU_ITEM_COUNT) {
        menuIndex = 0;
      }
      indexChanged = true;
    }

    if (indexChanged) {
      updateMenuScrollWindow();   // päättää itse full vs. values redraw
    }

    if (enter) {
      if (menuIndex == MENU_ITEM_HISTORY) {
        uiScreen = SCREEN_HISTORY;
        needFullRedraw = true;
      } else {
        startEditForCurrentMenuItem();
        menuEditMode = true;
        blinkOn = true;
        lastBlinkToggle = now;
        needValuesRedraw = true;  // arvot näkyviin vilkutusta varten
      }
    }

  } else {
    // --- EDIT-moodi ---
    handleUiMenuEdit(back, plus, minus, enter);
  }
}


void startEditForCurrentMenuItem() {
  switch (menuIndex) {
    case MENU_ITEM_SETPOINT:
      editValue = cfg.setpointC;   // tai cfg.setpointC
      break;

    case MENU_ITEM_HYSTERESIS:
      editValue = cfg.hysteresisC;     // tai cfg.hysteresisC
      break;

    case MENU_ITEM_MIN_DELAY:
      editValue = cfg.relayDelay;  // suoraan minuutit
      break;

    case MENU_ITEM_ALLOW_FAN_OFF:
      editValue = (cfg.allowFanOff ? 1.0 : 0.0);
      break;
  }
}

void handleUiMenuEdit(bool back, bool plus, bool minus, bool enter) {
  // Numeroarvon säätö


  if (minus) {
    switch (menuIndex) {

      case MENU_ITEM_SETPOINT:
        editValue -= 0.5;
        if (editValue < SETPOINT_MIN_C) editValue = SETPOINT_MIN_C;
        break;

      case MENU_ITEM_HYSTERESIS:
        editValue -= 0.1;
        if (editValue < HYST_MIN_C) editValue = HYST_MIN_C;
        break;

      case MENU_ITEM_MIN_DELAY:
        if (editValue > RELAY_MIN_DELAY) {
          editValue -= 1;
        }
        break;

      case MENU_ITEM_ALLOW_FAN_OFF:
        editValue = (editValue > 0.5) ? 0.0 : 1.0;  // toggle ON→OFF / OFF→ON
        break;
    }
    needValuesRedraw = true;
  }

  if (plus) {
    switch (menuIndex) {

      case MENU_ITEM_SETPOINT:
        editValue += 0.5;
        if (editValue > SETPOINT_MAX_C) editValue = SETPOINT_MAX_C;
        break;

      case MENU_ITEM_HYSTERESIS:
        editValue += 0.1;
        if (editValue > HYST_MAX_C) editValue = HYST_MAX_C;
        break;

      case MENU_ITEM_MIN_DELAY:
        if (editValue < RELAY_MAX_DELAY) {
          editValue += 1;
        }
        break;

      case MENU_ITEM_ALLOW_FAN_OFF:
        editValue = (editValue > 0.5) ? 0.0 : 1.0;  // toggle myös PLUS:lla
        break;
    }
    needValuesRedraw = true;
  }

  if (back) {
    menuEditMode = false;
    needValuesRedraw = true;   // palautetaan "normaalit" arvot näkyviin
    return;
  }

  if (enter) {
    switch (menuIndex) {

      case MENU_ITEM_SETPOINT:
        cfg.setpointC = editValue;
        break;

      case MENU_ITEM_HYSTERESIS:
        cfg.hysteresisC = editValue;
        break;

      case MENU_ITEM_MIN_DELAY:
        cfg.relayDelay = (uint8_t)(editValue + 0.5);
        break;

      case MENU_ITEM_ALLOW_FAN_OFF:
        cfg.allowFanOff = (editValue > 0.5) ? 1 : 0;
        break;
    }

    menuEditMode = false;
    saveSettings();
    needValuesRedraw = true;
  }
}

void handleUiHistory(bool back, bool plus, bool minus, bool enter) {
  if (back || enter) {
    uiScreen = SCREEN_HOME;
    needFullRedraw = true;
  }
}

void getMenuLabel(uint8_t index, char *dst, uint8_t dstSize) {
  if (index >= MENU_ITEM_COUNT) {
    if (dstSize > 0) dst[0] = '\0';
    return;
  }
  strncpy_P(dst, (PGM_P)pgm_read_word(&menuItems[index]), dstSize);
  dst[dstSize - 1] = '\0';
}



void updateDisplay() {
  // Vilkutus edit-tilassa menussa (vain valitun rivin arvo)
  if (uiScreen == SCREEN_MENU && menuEditMode) {
    if (now - lastBlinkToggle >= BLINK_INTERVAL_MS) {
      lastBlinkToggle = now;
      blinkOn = !blinkOn;
      drawMenuSelectedValueOnly();   // piirtää vain editValue-alueen
    }
  }

  // KOKO RUUDUN PÄIVITYS (layout + arvot)
  if (needFullRedraw) {
    lastLcdUpdateTime = now;
    needFullRedraw  = false;
    needValuesRedraw = false;
    lcd.clear();
    switch (uiScreen) {
      case SCREEN_HOME:
        drawHomeScreen();
        break;

      case SCREEN_MENU:
        drawMenuScreen();   // static + dynamic
        break;

      case SCREEN_HISTORY:
        drawHistoryScreen();
        break;
    }
    return;
  }

  if (needValuesRedraw) {
    lastLcdUpdateTime = now;
    needValuesRedraw = false;

    switch (uiScreen) {
      case SCREEN_HOME:
        drawHomeValues();
        break;

      case SCREEN_MENU:
        drawMenuDynamic();   // vain nuoli + arvot
        break;

      default:
        break;
    }
    return;
  }

  // HOME-ruudun ajastettu päivitys (lämpö, RH)
  if (uiScreen == SCREEN_HOME &&
      (now - lastLcdUpdateTime) >= LCD_UPDATE_INTERVAL_MS) {
    lastLcdUpdateTime = now;
    drawHomeValues();
  }
}

/* =========================================================================
   LCD:N PÄIVITYS
   ========================================================================= */
void drawHomeScreen() {
  lcd.setCursor(0, 0);
  lcd.print(F("Temp:    "));
  lcd.print((char)223);   // aste-merkki
  lcd.print(F("C  "));
  lcd.print(F("RH:   "));
  lcd.print(F("%"));

  lcd.setCursor(0, 1);
  lcd.print(F("Fan step: "));
  lcd.print(F("      "));   // tyhjennä loppu

  lcd.setCursor(0, 3);
  lcd.print(F("SetT:    "));

  lcd.print((char)223);
  lcd.print(F("C "));
  lcd.print(F("Hy:   "));

  lcd.print((char)223);
  lcd.print(F("C"));
  drawHomeValues();
}

void drawHomeValues() {
  lcd.setCursor(5, 0);
  if (currentTempC < 10) lcd.print(" ");
  lcd.print(currentTempC, 1);

  lcd.setCursor(16, 0);
  if (currentHumRH < 100) lcd.print(" ");
  if (currentHumRH < 10) lcd.print(" ");
  lcd.print(currentHumRH, 0);

  lcd.setCursor(10, 1);
  lcd.print(currentFanStep);

  lcd.setCursor(5, 4);
  if (cfg.setpointC < 10) lcd.print(" ");
  lcd.print(cfg.setpointC, 1);

  lcd.setCursor(15, 4);
  lcd.print(cfg.hysteresisC, 1);
}



void drawMenuScreen() {
  drawMenuStatic();
  drawMenuDynamic();
}


void drawMenuStatic() {
  for (uint8_t row = 0; row < MENU_VISIBLE_ROWS; row++) {
    uint8_t idx = menuFirstVisible + row;

    // Aloita nuolikolumnista
    lcd.setCursor(0, row);

    if (idx >= MENU_ITEM_COUNT) {
      // Tyhjä rivi: tyhjennetään koko rivi
      lcd.print(F("                    "));
      continue;
    }

    // Nuolipaikka tyhjänä (nuoli tulee dynaamisessa piirrossa)
    lcd.print(' ');

    // Label
    getMenuLabel(idx, lcdLine, sizeof(lcdLine));
    lcd.print(lcdLine);

    // Tyhjennä label-alueen loppu arvokolumniin asti
    uint8_t labelLen = strlen(lcdLine);
    uint8_t col = 1 + labelLen;
    while (col < MENU_VALUE_COL) {
      lcd.print(' ');
      col++;
    }
  }
}

void drawMenuDynamic() {
  for (uint8_t row = 0; row < MENU_VISIBLE_ROWS; row++) {
    uint8_t idx = menuFirstVisible + row;

    // Nuoli
    lcd.setCursor(0, row);
    if (idx < MENU_ITEM_COUNT && idx == menuIndex) {
      lcd.print('>');
    } else {
      lcd.print(' ');
    }

    // Arvo
    lcd.setCursor(MENU_VALUE_COL, row);

    // Tyhjennetään arvoalue ensin
    lcd.print(F("      "));
    lcd.setCursor(MENU_VALUE_COL, row);

    if (idx >= MENU_ITEM_COUNT) {
      continue;
    }

    bool useEditVal = (menuEditMode && idx == menuIndex);

    switch (idx) {
      case MENU_ITEM_HISTORY:
        lcd.print(F("->"));
        break;

      case MENU_ITEM_SETPOINT: {
          double v = useEditVal ? editValue : cfg.setpointC;
          int ip = (int)v;
          if (ip < 10) lcd.print(' ');
          lcd.print(v, 1);
          lcd.print((char)223);
          lcd.print('C');
          break;
        }

      case MENU_ITEM_HYSTERESIS: {
          double v = useEditVal ? editValue : cfg.hysteresisC;
          int ip = (int)v;
          if (ip < 10) lcd.print(' ');
          lcd.print(v, 1);
          lcd.print((char)223);
          lcd.print('C');
          break;
        }

      case MENU_ITEM_MIN_DELAY: {
          uint8_t v = useEditVal
                      ? (uint8_t)(editValue + 0.5)
                      : cfg.relayDelay;
          if (v < 10) lcd.print(' ');
          lcd.print(v);
          lcd.print(F(" min"));
          break;
        }
      
      case MENU_ITEM_ALLOW_FAN_OFF: {
          uint8_t v = useEditVal
                      ? (editValue > 0.5 ? 1 : 0)
                      : cfg.allowFanOff;

          if (v) {
            lcd.print(F("   YES"));
          } else {
            lcd.print(F("    NO"));
          }
          break;
        }

    }
  }
}



void drawHistoryScreen() {
  lcd.setCursor(0, 0);
  lcd.print(F("Historia (tulossa)"));
  lcd.setCursor(0, 2);
  lcd.print(F("BACK: kotiin"));
}

uint8_t calcMenuFirstVisible() {
  if (MENU_ITEM_COUNT <= MENU_VISIBLE_ROWS) {
    return 0;
  }

  if (menuIndex <= 1) {
    return 0;
  } else if (menuIndex >= MENU_ITEM_COUNT - 2) {
    return MENU_ITEM_COUNT - MENU_VISIBLE_ROWS;
  } else {
    return menuIndex - 1;
  }
}

void updateMenuScrollWindow() {
  uint8_t newFirst = calcMenuFirstVisible();
  if (newFirst != menuFirstVisible) {
    menuFirstVisible = newFirst;
    // näkyvät tekstirivit muuttuvat -> staattinen + dynaaminen uusiksi
    needFullRedraw = true;
  } else {
    // samat rivit näkyvissä -> riittää nuoli + arvot
    needValuesRedraw = true;
  }
}


void drawMenuSelectedValueOnly() {
  int8_t row = (int8_t)menuIndex - (int8_t)menuFirstVisible;
  if (row < 0 || row >= (int8_t)MENU_VISIBLE_ROWS) return;

  const uint8_t col = MENU_VALUE_COL;
  lcd.setCursor(col, row);

  if (!blinkOn) {
    // tyhjennä arvoalue vilkkua varten
    lcd.print(F("      "));
    return;
  }

  switch (menuIndex) {
    case MENU_ITEM_SETPOINT:
      if (editValue < 10) lcd.print(' ');
      lcd.print(editValue, 1);
      lcd.print((char)223);
      lcd.print('C');
      break;

    case MENU_ITEM_HYSTERESIS:
      if (editValue < 10) lcd.print(' ');
      lcd.print(editValue, 1);
      lcd.print((char)223);
      lcd.print('C');
      break;

    case MENU_ITEM_MIN_DELAY: {
        uint8_t v = (uint8_t)(editValue + 0.5);
        if (v < 10) lcd.print(' ');
        lcd.print(v);
        lcd.print(F(" min"));
        break;
      }

    case MENU_ITEM_ALLOW_FAN_OFF: {
        uint8_t v = (editValue > 0.5 ? 1 : 0);
        if (v) {
          lcd.print(F("   YES"));
        } else {
          lcd.print(F("    NO"));
        }
        break;
      }


    default:
      break;
  }
}



void saveSettings() {
  //  dumpSettings(cfg, F("Ennen CRC:tä (RAM)"));

  cfg.crc = calcCRC((uint8_t*)&cfg, sizeof(Settings) - 1); // CRC viimeiseen tavuun

  //  Serial.print(F("Tallennus alkaa - crc = "));
  //  Serial.println(cfg.crc);

  EEPROM.put(0, cfg);

  /*  Serial.println(F("Tallennus valmis"));

    // Lue takaisin heti ja dumppaa, niin näet mitä oikeasti meni EEPROMiin
    Settings verify;
    EEPROM.get(0, verify);
    dumpSettings(verify, F("Luettu heti EEPROMista"));
  */
}

/*
  void dumpSettings(const Settings &s, const __FlashStringHelper *tag) {
  Serial.println(tag);
  Serial.print(F("  setpointC   = ")); Serial.println(s.setpointC, 3);
  Serial.print(F("  hysteresisC = ")); Serial.println(s.hysteresisC, 3);
  Serial.print(F("  relayDelay  = ")); Serial.println(s.relayDelay);
  Serial.print(F("  version     = "));
  Serial.print(s.versionMajor); Serial.print('.');
  Serial.print(s.versionMinor); Serial.print('.');
  Serial.println(s.versionPatch);
  Serial.print(F("  crc         = ")); Serial.println(s.crc);
  }*/

bool loadSettings() {
  Settings tmp;
  EEPROM.get(0, tmp);

  uint8_t calc = calcCRC((uint8_t*)&tmp, sizeof(Settings) - 1);

  if (calc != tmp.crc) {
    Serial.println("EEPROM CRC FAIL -> Using defaults");
    return false;       // käytetään cfg defaultteja
  }

  cfg = tmp;            // validi, kopioi käyttöön
  return true;
}

uint8_t calcCRC(uint8_t *data, uint16_t len) {
  uint8_t crc = 0;
  while (len--) crc ^= *data++; // XOR CRC
  return crc;
}
