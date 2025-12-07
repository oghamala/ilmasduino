/* TODO:
 *  
 *  * GITHUB KÄYTTÖÖN --- OK
 *  
 *  * TUULETTIMEN LOGIIKKA UUSIKSI (katso tarkemmat huomiot sieltä funktiosta) Tarkista myös jääkö turhia muuttujia
 *  
 *  * 100 - 1000 ms TAUKO VAIHTOJEN VÄLIIN (Sähköhäiriöiden minimointi)
 *  
 *  * ASETUKSET EEPROMILLE
 *  
 *  * ASETTUSRUUTU
 *  
 *  * NAPPEIHIN "LONGPRESS"-TOIMINTO
 *  
 *  * HISTORIARUUTU (24H Alkuun muistitaulukosta)
 *    >Myös arvojen aaltoilu nähtävä keskiarvojen lisäksi, että releronklaus saadaan parametreja viilaamalla minimiin
 *  
 *  * WS2812 - statusvalot (ja virransäästö näyttöön?)
 *    >Erilaisia värikoodauksia erroreille ja tavoitearvoissa pysymiselle - ehkä vähän sellain pidemmältäkin jaksolta?
 *      -Esim. <90% viim. vrk tavoitearvossa vihreä (ja maks. poikkeama pieni) / yli 60% keltainen / yli 30% oranssi / <30% (ja/tai maks. poikkeama iso) punainen 
 *      -Esim. Alle 12 vaihteen vaihtoa päivässä = vihreä / <24 keltainen / <48 oranssi / >48 punainen

 *  
 *  * GRAAFINEN ESITYSMUOTO HISTORIALLE (Custom characters)
 *  
 *  * EEPROMILLE JÄRKEVÄ FORMAATTI (kts. seuraava)
 *    >Tilaa asetuksille (ja grafiikkaelementeille?)
 *    >Varaus myös toiseelle BME280-anturille
 *    >(Varaus volttiarvoille /) virheilmoituksille
 *      -Vikakoodeja ehkä vain yksi tuntiin logiin tai jonkinlainen koodaus, että mikä koodin aiheutti että kaikki mahdolliset saa 1-2 tavuun 
 *    >Ehkä jopa laskee releiden kulumista ja ilmoittaa kun alkaa olla aika vaihtaa??? - Jos siis saa järkevästi jotenkin ilman, että tarvii laskea ja kirjata joka naksu
 *      -Ehkä joku koodaus että 5-10v tulee kevyelläkin täyteen ja kirjaa päivä/viikkotasolla kevyt/keskiraskas/raskas/äärimmäinen
 *    >Tiedostojärjestelmän formaattiversio kirjataan ylös ja kysyttäessä formatoidaan, jos vanha formaatti ei yhteensopiva/tunnistettu
 *      -Hallittu siirtymä uuteen formaattiin!!! (vaikka jollain apuohjelmalla tai Serial-yhteyden avulla toteutetulla siirtoformatoinnilla)  
 *  
 *  * PIDEMMÄN AJAN HISTORIA EEPROMILLE (Rengaspuskuri)
 *    >Mahdollisuus tyhjentämiseen ja siirtoon koneelle?
 *      -Älä kuitenkaan nollaa reledataa samalla
 *  
 *  
 *  * VOLTTIMITTARI (RELEIDEN TOIMINNAN SEURANTA)
 *    >Seuraa, että vaiteet vaihtuu oikein ja poistaa tarvittaessa rikkinäisen releen käytöstä & tallentaa vikakoodin.
 *    
 *  * Bluetooth-moduuli toisen BM280:n tilalle tai rinnalle
 *    
 */      





#include <Wire.h>
#include <hd44780.h>
#include <hd44780ioClass/hd44780_I2Cexp.h>
#include <Adafruit_BME280.h>
#include <GPTbounce.h>

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


const uint8_t BTN_SET_UP_PIN      = A0;  // Nappi: nosta tavoitelämpöä
const uint8_t BTN_SET_DOWN_PIN    = A1;  // Nappi: laske tavoitelämpöä
const uint8_t BTN_HYST_UP_PIN     = A2;  // Nappi: nosta hystereesiä
const uint8_t BTN_HYST_DOWN_PIN   = A3;  // Nappi: laske hystereesiä

// Yksinkertainen nappien käsittely ilman raskasta debouncausta
GPTbounce btnSetUp;
GPTbounce btnSetDown;
GPTbounce btnHystUp;
GPTbounce btnHystDown;

const double SETPOINT_MIN_C = 5.0;
const double SETPOINT_MAX_C = 35.0;

const double HYST_MIN_C     = 0.1;
const double HYST_MAX_C     = 5.0;

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

/* LÄMPÖTILA-ASETUS JA SÄÄDÖN INTERVALLI*/

// Nämä näkyvät myös LCD:llä ja napit muuttavat Setpoint ja Hysteresis –arvoa.
double SetpointTempC = 25.0;      // Tavoitelämpö (C)
double HysteresisC   = 1.0;       // Hystereesi (C)
double EmergencyThresholdC = 4.0;

// Kuinka usein arvioidaan uusi "haluttu porras"
// (tässä ei vielä vaihdeta releitä, vain lasketaan logiikkaa)
const unsigned long CONTROL_INTERVAL_MS = 10000UL;  // 10 s

// Kuinka usein SAA oikeasti vaihtaa releen tilaa
const unsigned long MIN_RELAY_INTERVAL_MS = 60000UL;  // 1 min, tai 600000UL = 10 min

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
  lcd.print("Ilmasduino");
  lcd.setCursor(0, 1);
  lcd.print("Kaynnistyy...");

  // BME280
  if (!bme.begin(BME280_ADDRESS)) {
    bmeOK = false;
    lcd.setCursor(0, 2);
    lcd.print("BME280 ei loydy!");
    Serial.println(F("BME280 ei loydy annetusta osoitteesta."));
  } else {
    bmeOK = true;
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

    lcd.setCursor(0, 2);
    lcd.print("BME280 OK osoite:");
    lcd.setCursor(0, 3);
    lcd.print("0x");
    lcd.print(BME280_ADDRESS, HEX);
  }

  // Nappulat INPUT_PULLUP
  btnSetUp.begin(BTN_SET_UP_PIN, 25);     // 25 ms debounce
  btnSetDown.begin(BTN_SET_DOWN_PIN, 25);
  btnHystUp.begin(BTN_HYST_UP_PIN, 25);
  btnHystDown.begin(BTN_HYST_DOWN_PIN, 25);

  // Releet
  for (uint8_t i = 0; i < totalRelaySteps; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], RELAY_INACTIVE_LEVEL); // alkuun kaikki pois
  }

  delay(2000);
  lcd.clear();
}

/* =========================================================================
   PÄÄSILMUKKA
   ========================================================================= */
void loop() {
  unsigned long now = millis();


  // 1) Lue nappulat ja päivitä asetukset
  btnSetUp.update();
  btnSetDown.update();
  btnHystUp.update();
  btnHystDown.update();

  handleButtons();

  // 2) Anturiluku BME280 forced -tilassa harvakseltaan
  if (now - lastSensorTime >= SENSOR_INTERVAL_MS) {
    lastSensorTime = now;
    readSensor();    // käyttää bme.takeForcedMeasurement() + readTemperature/readHumidity
  }

  // 3) Ohjauslogiikka (määrittelee halutun portaan, mutta ei vielä räpsi releitä liian usein)
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = now;
    updateFanControl();   // uusi funktio, katso alla
  }

  // 4) LCD päivitys
  if (now - lastLcdUpdateTime >= LCD_UPDATE_INTERVAL_MS) {
    lastLcdUpdateTime = now;
    updateLCD();
  }
}

/* =========================================================================
   SENSORIN LUKU
   ========================================================================= */
void readSensor() {
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

/* =========================================================================
   TUULETTIMEN SÄÄTÖ

   Tämä toimii nyt ihan pitkin vittua. Logiikka siis tällä hetkellä se, että 
   tavoite on aina päästä 0 vaihteelle ja säädetään vaan, että millä nopeudella
   sinne mennään.

   KORJATAAN:

   *Vaihtaa yhden vaihteen kerrallaan, jos ero on pienehkö
   *Vaihtaa kaksi vaihdetta kerrallaan, jos ero on iso (= Hätäsäätö)
   *Ei vaihda vastakkaiseeen suutaan kahta vaihdetta seuraavalla kerralla, että
   ei tule "aaltoilua".
   ========================================================================= */

void updateFanControl() {
  // Erotus setpointtiin
  double diff = currentTempC - SetpointTempC;  // >0 = liian kuuma, <0 = liian kylmä

  // 1) Lasketaan, mikä porras olisi järkevä lämpötilapoikkeaman perusteella.
  //    TÄMÄ ON VAIN EHDOTUS (soft-logiikka), sitä ei vielä toteuteta releisiin.


  /* TÄMÄ SE POISTETTAVA LOGIIKKA
  int requestedStep = 0;

  // Esimerkki: säädä nämä portaat omaan makuun.

 if (diff > 3.0) {
    requestedStep = 6;
  } else if (diff > 2.0) {
    requestedStep = 5;
  } else if (diff > 1.5) {
    requestedStep = 4;
  } else if (diff > 1.0) {
    requestedStep = 3;
  } else if (diff > 0.5) {
    requestedStep = 2;
  } else if (diff > 0.0) {
    requestedStep = 1;
  } else {
    // diff <= 0: ollaan setpointin alapuolella tai siellä
    requestedStep = 0;  // tai 1, jos haluat että se ei koskaan täysin pysähdy
  }*/

  // 1) Tarkastetaan säädön tarve

  int requestedChange = 0;

  if (diff > EmergencyThresholdC && lastEmergencyRelayChangeDirection >= 0) {
    requestedChange = 2;
    lastEmergencyRelayChangeDirection = 1;
    }
  else if (diff > HysteresisC) {
    requestedChange = 1;
  }
  else if (diff < (EmergencyThresholdC * -1) && lastEmergencyRelayChangeDirection <= 0) {
    requestedChange = -2;
    lastEmergencyRelayChangeDirection = -1;
  }
  else if (diff < (HysteresisC * -1)) {
    requestedChange = -1;  
  }
  else {
    requestedChange = 0;
    lastEmergencyRelayChangeDirection = 0;    
  }

  // 2) Tarkastetaan ettei mennä säätörajan ulkopuolelle
  
  while ((currentFanStep + requestedChange) > totalRelaySteps) {
    requestedChange--;
  }
  while ((currentFanStep + requestedChange) < 0) {
    requestedChange++;
  }
  


  // 3) Tarkastetaan onko kulunut tarpeeksi aikaa viimeisimmästä säädöstä
  
  unsigned long now = millis();
  bool enoughTimePassed = (now - lastRelayChangeTime) >= MIN_RELAY_INTERVAL_MS;

  // 4) Tehdään säätö tarvittaessa
  
  if (enoughTimePassed && requestedChange != 0) {
    currentFanStep = currentFanStep + requestedChange;
    applyFanStep(currentFanStep);  // pyydetään alifunktiota suorittamaan säätö
    lastRelayChangeTime = now;
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
void handleButtons() {
  // Painallus = yksi "fell" per todellinen painallus, vaikka nappi tärisisi
  if (btnSetUp.fell()) {
    SetpointTempC += 0.5;
  }

  if (btnSetDown.fell()) {
    SetpointTempC -= 0.5;
  }

  // Clamp setpoint
  if (SetpointTempC < SETPOINT_MIN_C) SetpointTempC = SETPOINT_MIN_C;
  if (SetpointTempC > SETPOINT_MAX_C) SetpointTempC = SETPOINT_MAX_C;

  if (btnHystUp.fell()) {
    HysteresisC += 0.1;
  }

  if (btnHystDown.fell()) {
    HysteresisC -= 0.1;
    if (HysteresisC < 0.1) {
      HysteresisC = 0.1;
    }
  }

  // Clamp hystereesi
  if (HysteresisC < HYST_MIN_C) HysteresisC = HYST_MIN_C;
  if (HysteresisC > HYST_MAX_C) HysteresisC = HYST_MAX_C;
}


// Halutessasi voit käyttää myös "painallus pohjassa" -toimintoa:
// if (btnSetUp.isPressed()) { ... nopeampi muutos pitkällä painalluksella ... }}

/* =========================================================================
   LCD:N PÄIVITYS
   -------------------------------------------------------------------------
   20x4-näyttö:
   Rivi 0: Mitattu lämpö ja RH
   Rivi 1: Setpoint ja hystereesi
   Rivi 2: Puhaltimen porras
   Rivi 3: dT (ero setpointtiin) ja hystereesi
   ========================================================================= */
void updateLCD() {
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(currentTempC, 1);
  lcd.print((char)223);   // aste-merkki
  lcd.print("C  ");
  lcd.print("RH:");
  lcd.print(currentHumRH, 1);
  lcd.print("%   ");

  lcd.setCursor(0, 1);
  lcd.print("Set:");
  lcd.print(SetpointTempC, 1);
  lcd.print((char)223);
  lcd.print("C ");
  lcd.print("Hy:");
  lcd.print(HysteresisC, 1);
  lcd.print("C   ");

  lcd.setCursor(0, 2);
  lcd.print("Fan step: ");
  lcd.print(currentFanStep);
  lcd.print("      ");   // tyhjennä loppu

  lcd.setCursor(0, 3);
  lcd.print("dT:");
  lcd.print(currentTempC - SetpointTempC, 1);
  lcd.print((char)223);
  lcd.print("C Hy:");
  lcd.print(HysteresisC, 1);
  lcd.print("C  ");
}
