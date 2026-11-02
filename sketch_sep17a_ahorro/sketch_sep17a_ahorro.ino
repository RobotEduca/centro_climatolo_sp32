/*
  Estación de medición con AHORRO DE ENERGÍA (deep sleep) -> ThingSpeak
  ---------------------------------------------------------------------------
  El ESP32 duerme casi todo el tiempo. Se despierta:
    - cada INTERVALO_ENVIO_S segundos (por defecto 5 min) para medir y enviar
    - cada vez que el SW-420 detecta una vibración (despierta, la cuenta y
      vuelve a dormir)

  CAMBIO DE CABLEADO (obligatorio):
    SW-420 DO  ->  GPIO 27   (antes GPIO 18; solo los pines RTC pueden
                              despertar al ESP32 del deep sleep)
    Todo lo demás igual: DHT22 GPIO 4, SDA GPIO 21, SCL GPIO 22.

  Campos de ThingSpeak (igual que antes):
    Field 1 = Riesgo de lluvia (0 a 100 %)
    Field 2 = Temperatura (°C)
    Field 3 = Humedad (%)
    Field 4 = Sensación térmica (°C)
    Field 5 = Punto de rocío (°C)
    Field 7 = Vibraciones desde el último envío
    Field 8 = Presión (hPa)

  Librerías: ThingSpeak, DHT sensor library (Adafruit),
             Adafruit BMP280 Library (+ BusIO y Unified Sensor)
*/
#include <WiFi.h>
#include <ThingSpeak.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <sys/time.h>
#include <math.h>

// =========================================================================
// 1. CONFIGURACIÓN DE RED Y THINGSPEAK
// =========================================================================
const char* ssid = "DIANA SOTO";
const char* password = "cams1108"; // Si no tiene clave: ""

unsigned long myChannelNumber = 3498097;
const char* myWriteAPIKey = "SHW5K0X8QBZGFW0Y";

WiFiClient client;

// =========================================================================
// 2. PINES Y AJUSTES
// =========================================================================
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

const int PIN_VIBRACION = 27;         // Debe ser un pin RTC (27, 33, 32, 25, 26, 34, 35, 36, 39, 4, 2, 12-15)
const int PIN_LED_VIBRACION = 2;      // LED azul de la placa
const int PIN_SDA = 21;
const int PIN_SCL = 22;

const uint32_t INTERVALO_ENVIO_S = 300;      // Enviar cada 5 minutos
const unsigned long REBOTE_MS = 50;
const unsigned long LED_DURACION_MS = 300;
const unsigned long TIMEOUT_WIFI_MS = 15000;

// Riesgo de lluvia (mismos umbrales de antes)
const float HUM_MIN = 60.0, HUM_MAX = 95.0;
const float DIF_ROCIO_MAX = 8.0, DIF_ROCIO_MIN = 1.0;
const float CAIDA_MIN_HPA = 2.0, CAIDA_MAX_HPA = 4.0;
const int HISTORIAL_N = 19;                    // 19 muestras cada 10 min = 3 horas
const uint32_t INTERVALO_HISTORIAL_S = 570;    // ~10 min (con tolerancia)
const float MIN_MINUTOS_TENDENCIA = 60.0;

// =========================================================================
// 3. MEMORIA QUE SOBREVIVE AL DEEP SLEEP (RTC)
// =========================================================================
RTC_DATA_ATTR uint32_t ultimoEnvioSeg = 0;
RTC_DATA_ATTR uint32_t ultimaMuestraSeg = 0;
RTC_DATA_ATTR unsigned long vibracionesRTC = 0;
RTC_DATA_ATTR int nivelReposo = LOW;
RTC_DATA_ATTR int historialCount = 0;
RTC_DATA_ATTR float historialPresion[HISTORIAL_N];
RTC_DATA_ATTR uint32_t historialTiempo[HISTORIAL_N];

// =========================================================================
// 4. VARIABLES NORMALES
// =========================================================================
Adafruit_BMP280 bmp(&Wire);
volatile unsigned long contadorISR = 0;
volatile unsigned long ultimoEventoMs = 0;
portMUX_TYPE muxVibracion = portMUX_INITIALIZER_UNLOCKED;

float temperatura = 0, humedad = 0, sensacionTermica = 0, puntoRocio = 0, presion = 0;
bool presionValida = false;
float tendenciaPresion3h = 0;

// Declaración anticipada de la interrupción (se usa en setup)
void IRAM_ATTR isrVibracion();

// =========================================================================
// SETUP (se ejecuta en cada despertar; loop() nunca se usa)
// =========================================================================
void setup() {
  setCpuFrequencyMhz(80);               // Menos consumo (el WiFi funciona desde 80 MHz)
  Serial.begin(115200);

  esp_sleep_wakeup_cause_t causa = esp_sleep_get_wakeup_cause();
  bool primerArranque = (causa != ESP_SLEEP_WAKEUP_EXT0 && causa != ESP_SLEEP_WAKEUP_TIMER);

  pinMode(PIN_LED_VIBRACION, OUTPUT);
  digitalWrite(PIN_LED_VIBRACION, LOW);
  pinMode(PIN_VIBRACION, INPUT);

  if (primerArranque) {
    // Detecta el nivel de reposo del SW-420 (mantenlo quieto al encender)
    int altos = 0;
    for (int i = 0; i < 20; i++) { altos += digitalRead(PIN_VIBRACION); delay(10); }
    nivelReposo = (altos >= 10) ? HIGH : LOW;
    Serial.print("\n[SW-420] Nivel en reposo: ");
    Serial.println(nivelReposo == HIGH ? "HIGH" : "LOW");
  }

  // Cuenta vibraciones mientras el ESP32 está despierto
  int flanco = (nivelReposo == HIGH) ? FALLING : RISING;
  attachInterrupt(digitalPinToInterrupt(PIN_VIBRACION), isrVibracion, flanco);

  if (causa == ESP_SLEEP_WAKEUP_EXT0) {
    // Nos despertó una vibración: contarla, encender el LED y esperar a que pase
    vibracionesRTC++;
    Serial.println("[SW-420] Vibración detectada");
    digitalWrite(PIN_LED_VIBRACION, HIGH);
    esperarReposo();
    unsigned long t0 = millis();
    while (millis() - t0 < LED_DURACION_MS) delay(10);
    digitalWrite(PIN_LED_VIBRACION, LOW);
  }

  uint32_t ahora = segundosAhora();
  if (primerArranque || (ahora - ultimoEnvioSeg) + 10 >= INTERVALO_ENVIO_S) {
    cicloMedicionYEnvio();
  }

  dormir();
}

void loop() {
  // No se usa: el programa termina en dormir() y reinicia al despertar.
}

// =========================================================================
// CICLO PRINCIPAL: medir, calcular y enviar
// =========================================================================
void cicloMedicionYEnvio() {
  Serial.println("--- Ciclo de medición ---");

  // DHT22: la 1ª lectura tras un reposo largo puede ser vieja; se hace una de "calentamiento"
  dht.begin();
  dht.readTemperature();
  unsigned long tDht = millis();

  // Presión
  Wire.begin(PIN_SDA, PIN_SCL);
  leerPresion();

  // WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < TIMEOUT_WIFI_MS) {
    delay(100);
  }

  // DHT22: lectura real (debe pasar >= 2 s desde la anterior)
  while (millis() - tDht < 2100) delay(50);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  bool dhtValido = !isnan(h) && !isnan(t);
  int riesgo = 0;
  if (dhtValido) {
    temperatura = t;
    humedad = h;
    sensacionTermica = calcularSensacionTermica(t, h);
    puntoRocio = calcularPuntoRocio(t, h);
    Serial.print("Temp: "); Serial.print(temperatura); Serial.print(" C | Hum: "); Serial.println(humedad);
    riesgo = calcularRiesgoLluvia();
  } else {
    Serial.println("Error leyendo el DHT22");
  }

  // Envío
  uint32_t ahora = segundosAhora();
  if (WiFi.status() == WL_CONNECTED) {
    ThingSpeak.begin(client);

    unsigned long vib;
    portENTER_CRITICAL(&muxVibracion);
    vib = vibracionesRTC + contadorISR;
    portEXIT_CRITICAL(&muxVibracion);

    if (dhtValido) {
      ThingSpeak.setField(1, riesgo);
      ThingSpeak.setField(2, temperatura);
      ThingSpeak.setField(3, humedad);
      ThingSpeak.setField(4, sensacionTermica);
      ThingSpeak.setField(5, puntoRocio);
    }
    ThingSpeak.setField(7, (long) vib);
    if (presionValida) ThingSpeak.setField(8, presion);

    int codigo = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    if (codigo == 200) {
      Serial.println("¡Actualización exitosa en ThingSpeak!");
      portENTER_CRITICAL(&muxVibracion);
      vibracionesRTC = 0;
      contadorISR = 0;
      portEXIT_CRITICAL(&muxVibracion);
      ultimoEnvioSeg = ahora;
    } else {
      Serial.print("Error subiendo datos. Código HTTP: "); Serial.println(codigo);
      ultimoEnvioSeg = ahora - (INTERVALO_ENVIO_S - 60); // reintenta en ~1 min
    }
  } else {
    Serial.println("Sin WiFi: se reintenta en ~1 minuto");
    ultimoEnvioSeg = ahora - (INTERVALO_ENVIO_S - 60);
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
}

// =========================================================================
// DEEP SLEEP
// =========================================================================
void dormir() {
  // Pasa las vibraciones contadas por la interrupción a la memoria RTC
  detachInterrupt(digitalPinToInterrupt(PIN_VIBRACION));
  portENTER_CRITICAL(&muxVibracion);
  vibracionesRTC += contadorISR;
  contadorISR = 0;
  portEXIT_CRITICAL(&muxVibracion);

  digitalWrite(PIN_LED_VIBRACION, LOW);

  uint32_t ahora = segundosAhora();
  uint32_t transcurrido = ahora - ultimoEnvioSeg;
  uint32_t restante = (transcurrido >= INTERVALO_ENVIO_S) ? 1 : (INTERVALO_ENVIO_S - transcurrido);

  // Despertar por temporizador y por vibración (el pin sale de su nivel de reposo)
  esp_sleep_enable_timer_wakeup((uint64_t) restante * 1000000ULL);
  esp_sleep_enable_ext0_wakeup((gpio_num_t) PIN_VIBRACION, (nivelReposo == HIGH) ? 0 : 1);

  Serial.print("Durmiendo "); Serial.print((unsigned long) restante); Serial.println(" s...");
  Serial.flush();
  esp_deep_sleep_start();
}

// =========================================================================
// SENSOR DE VIBRACIONES
// =========================================================================
void IRAM_ATTR isrVibracion() {
  unsigned long t = millis();
  if (t - ultimoEventoMs > REBOTE_MS) {
    ultimoEventoMs = t;
    portENTER_CRITICAL_ISR(&muxVibracion);
    contadorISR++;
    portEXIT_CRITICAL_ISR(&muxVibracion);
  }
}

// Espera (máx. 3 s) a que el sensor vuelva a su nivel de reposo
void esperarReposo() {
  unsigned long t0 = millis(), estableDesde = 0;
  while (millis() - t0 < 3000) {
    if (digitalRead(PIN_VIBRACION) == nivelReposo) {
      if (estableDesde == 0) estableDesde = millis();
      if (millis() - estableDesde >= 200) break;
    } else {
      estableDesde = 0;
    }
    delay(5);
  }
}

// =========================================================================
// SENSOR DE PRESIÓN BME280 / BMP280
// =========================================================================
uint8_t leerIdChip(uint8_t direccion) {
  Wire.beginTransmission(direccion);
  Wire.write(0xD0);
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom(direccion, (uint8_t) 1) != 1) return 0;
  return Wire.read();
}

bool iniciarSensorPresion() {
  const uint8_t direcciones[] = {0x76, 0x77};
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t id = leerIdChip(direcciones[i]);
    if (id == 0x56 || id == 0x57 || id == 0x58 || id == 0x60) {
      if (bmp.begin(direcciones[i], id)) {
        // Modo forzado: mide una vez y vuelve a dormir (menos consumo)
        bmp.setSampling(Adafruit_BMP280::MODE_FORCED,
                        Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16,
                        Adafruit_BMP280::FILTER_OFF,
                        Adafruit_BMP280::STANDBY_MS_1);
        return true;
      }
    }
  }
  Serial.println("[BMx280] No se encontró el sensor de presión. Revisa SDA, SCL, VCC, GND, CSB y SDO.");
  return false;
}

void leerPresion() {
  presionValida = false;
  if (!iniciarSensorPresion()) return;
  bmp.takeForcedMeasurement();
  float p = bmp.readPressure() / 100.0; // Pa -> hPa
  if (!isnan(p) && p > 300.0 && p < 1100.0) {
    presion = p;
    presionValida = true;
    guardarMuestraPresion(segundosAhora());
    Serial.print("Presión: "); Serial.print(presion, 1); Serial.println(" hPa");
  } else {
    Serial.println("Error leyendo presión");
  }
}

// =========================================================================
// RIESGO DE LLUVIA
// =========================================================================
float limitar01(float x) {
  if (x < 0) return 0;
  if (x > 1) return 1;
  return x;
}

void guardarMuestraPresion(uint32_t ahora) {
  if (historialCount > 0 && (ahora - ultimaMuestraSeg) < INTERVALO_HISTORIAL_S) return;
  ultimaMuestraSeg = ahora;
  if (historialCount < HISTORIAL_N) {
    historialPresion[historialCount] = presion;
    historialTiempo[historialCount] = ahora;
    historialCount++;
  } else {
    for (int i = 0; i < HISTORIAL_N - 1; i++) {
      historialPresion[i] = historialPresion[i + 1];
      historialTiempo[i] = historialTiempo[i + 1];
    }
    historialPresion[HISTORIAL_N - 1] = presion;
    historialTiempo[HISTORIAL_N - 1] = ahora;
  }
}

int calcularRiesgoLluvia() {
  float compHum = 35.0 * limitar01((humedad - HUM_MIN) / (HUM_MAX - HUM_MIN));
  float dif = temperatura - puntoRocio;
  float compRocio = 30.0 * limitar01((DIF_ROCIO_MAX - dif) / (DIF_ROCIO_MAX - DIF_ROCIO_MIN));

  float compPresion = 0;
  bool hayTendencia = false;
  if (historialCount >= 2) {
    float minutos = (historialTiempo[historialCount - 1] - historialTiempo[0]) / 60.0;
    if (minutos >= MIN_MINUTOS_TENDENCIA) {
      float cambio = historialPresion[historialCount - 1] - historialPresion[0];
      tendenciaPresion3h = cambio * (180.0 / minutos);
      compPresion = 35.0 * limitar01((-tendenciaPresion3h - CAIDA_MIN_HPA) / (CAIDA_MAX_HPA - CAIDA_MIN_HPA));
      hayTendencia = true;
    }
  }

  float riesgo = hayTendencia ? (compHum + compRocio + compPresion)
                              : (compHum + compRocio) * 100.0 / 65.0;
  int resultado = (int) round(riesgo);
  if (resultado < 0) resultado = 0;
  if (resultado > 100) resultado = 100;

  Serial.print("[Lluvia] Riesgo: "); Serial.print(resultado); Serial.print("% (hum ");
  Serial.print(compHum, 0); Serial.print(", rocío "); Serial.print(compRocio, 0);
  if (hayTendencia) {
    Serial.print(", presión "); Serial.print(compPresion, 0);
    Serial.print(" | tendencia "); Serial.print(tendenciaPresion3h, 1); Serial.print(" hPa/3h");
  } else {
    Serial.print(", presión sin historial");
  }
  Serial.println(")");
  return resultado;
}

// =========================================================================
// UTILIDADES
// =========================================================================
// Segundos desde el encendido; el reloj RTC sigue contando durante el deep sleep
uint32_t segundosAhora() {
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return (uint32_t) tv.tv_sec;
}

float calcularPuntoRocio(float tC, float hr) {
  float a = ((17.27 * tC) / (237.7 + tC)) + log(hr / 100.0);
  return (237.7 * a) / (17.27 - a);
}

float calcularSensacionTermica(float tC, float hr) {
  if (tC < 27.0) return tC;
  float tF = tC * 9.0 / 5.0 + 32.0;
  float hi = -42.379 + 2.04901523 * tF + 10.14333127 * hr
             - 0.22475541 * tF * hr - 0.00683783 * tF * tF
             - 0.05481717 * hr * hr + 0.00122874 * tF * tF * hr
             + 0.00085282 * tF * hr * hr - 0.00000199 * tF * tF * hr * hr;
  return (hi - 32.0) * 5.0 / 9.0;
}
