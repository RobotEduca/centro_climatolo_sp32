/*
  Estación de medición conectada a ThingSpeak
  ---------------------------------------------------------------------------
  Sensores:
    - DHT22          -> temperatura y humedad (GPIO 4)
    - SW-420         -> vibraciones           (GPIO 18)
    - GY-BME/BMP280  -> presión atmosférica   (I2C: SDA GPIO 21, SCL GPIO 22)
    - LED azul de la placa (GPIO 2) -> se enciende solo cuando hay vibración

  Campos de ThingSpeak:
    Field 1 = Riesgo de lluvia (0 a 100 %)
    Field 2 = Temperatura (°C)
    Field 3 = Humedad (%)
    Field 4 = Sensación térmica (°C)
    Field 5 = Punto de rocío (°C)
    Field 7 = Vibraciones (eventos detectados desde el último envío)
    Field 8 = Presión (hPa)
    (Field 6 ya no se usa)

  Riesgo de lluvia (índice, no es una probabilidad calibrada):
    Humedad               hasta 35 puntos
    Temperatura - Rocío   hasta 30 puntos
    Caída de presión 3 h  hasta 35 puntos
  Mientras no haya al menos 1 hora de historial de presión, el índice se
  calcula solo con humedad y punto de rocío (reescalado a 0-100).

  Librerías necesarias (Administrador de bibliotecas):
    - ThingSpeak
    - DHT sensor library (Adafruit)
    - Adafruit BMP280 Library (instala también Adafruit Unified Sensor y BusIO)
*/
#include <WiFi.h>
#include <ThingSpeak.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <math.h>

// =========================================================================
// 1. CONFIGURACIÓN DE RED Y THINGSPEAK (¡Modifica esto con tus datos!)
// =========================================================================
const char* ssid = "DIANA SOTO";
const char* password = "cams1108"; // Si no tiene clave, déjalo así: ""

unsigned long myChannelNumber = 3498097;         // REEMPLAZA por tu Channel ID
const char* myWriteAPIKey = "SHW5K0X8QBZGFW0Y";  // REEMPLAZA por tu Write API Key

WiFiClient client; // Cliente de red para el ESP32

// =========================================================================
// 2. CONFIGURACIÓN DE SENSORES
// =========================================================================
// DHT22 (se deja igual que antes)
#define DHTPIN 4
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

// Sensor de vibraciones SW-420 (salida digital DO)
const int PIN_VIBRACION = 18;
const unsigned long REBOTE_MS = 50;   // Ignora pulsos separados menos de 50 ms
volatile unsigned long contadorVibraciones = 0;
volatile unsigned long ultimoEventoMs = 0;
volatile bool ledPendiente = false;   // La interrupción pide encender el LED
portMUX_TYPE muxVibracion = portMUX_INITIALIZER_UNLOCKED;
int nivelReposo = HIGH;               // Se detecta solo al arrancar

// LED indicador de vibración (LED azul de la placa, GPIO 2)
const int PIN_LED_VIBRACION = 2;
const unsigned long LED_DURACION_MS = 300; // Tiempo que queda encendido
bool ledEncendido = false;
unsigned long ledEncendidoDesde = 0;

// Sensor de presión GY-BME280 / GY-BMP280 (I2C)
const int PIN_SDA = 21;
const int PIN_SCL = 22;
Adafruit_BMP280 bmp(&Wire);
bool presionDisponible = false;
bool presionValida = false;           // true si la última lectura fue correcta

// Variables para almacenar los datos
float temperatura = 0;
float humedad = 0;
float sensacionTermica = 0;
float puntoRocio = 0;
float presion = 0;                    // hPa
bool dhtValido = false;               // true si la última lectura del DHT22 fue correcta

// ---- Riesgo de lluvia ----
// Ajusta estos valores para calibrar el índice con tus propios datos.
const float HUM_MIN = 60.0;           // % de humedad donde empieza a sumar
const float HUM_MAX = 95.0;           // % de humedad donde suma el máximo
const float DIF_ROCIO_MAX = 8.0;      // °C (T - rocío) donde empieza a sumar
const float DIF_ROCIO_MIN = 1.0;      // °C (T - rocío) donde suma el máximo
const float CAIDA_MIN_HPA = 2.0;      // caída en 3 h donde empieza a sumar
const float CAIDA_MAX_HPA = 4.0;      // caída en 3 h donde suma el máximo
                                      // (en el trópico la presión baja 1-2 hPa
                                      //  cada tarde por la marea atmosférica)

// Historial de presión: una muestra cada 10 min, últimas 3 horas
const int HISTORIAL_N = 19;
const unsigned long INTERVALO_HISTORIAL = 600000UL; // 10 minutos
const unsigned long MIN_MINUTOS_TENDENCIA = 60;     // mínimo de historial para usar la tendencia
float historialPresion[HISTORIAL_N];
unsigned long historialTiempo[HISTORIAL_N];
int historialCount = 0;
unsigned long ultimaMuestraHistorial = 0;

int riesgoLluvia = 0;                 // 0 a 100
bool riesgoValido = false;
float tendenciaPresion3h = 0;         // hPa en 3 h (negativo = está bajando)

// Temporizadores (No usar delay() para no bloquear el ESP32)
unsigned long ultimaLecturaSensores = 0;
const unsigned long INTERVALO_SENSORES = 2000; // Leer sensores cada 2 segs

unsigned long ultimaSubidaThingSpeak = 0;
const unsigned long INTERVALO_THINGSPEAK = 20000; // Subir datos cada 20 segs (Mínimo 15s en ThingSpeak)

// =========================================================================
// SETUP Y LOOP
// =========================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);

  dht.begin();

  // LED indicador (apagado en reposo)
  pinMode(PIN_LED_VIBRACION, OUTPUT);
  digitalWrite(PIN_LED_VIBRACION, LOW);

  // I2C y sensor de presión
  Wire.begin(PIN_SDA, PIN_SCL);
  iniciarSensorPresion();

  // Sensor de vibraciones
  configurarVibracion();

  // Conectar a WiFi
  Serial.println();
  Serial.print("Conectando a la red WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n¡Conectado exitosamente al WiFi!");
  Serial.print("Dirección IP asignada: ");
  Serial.println(WiFi.localIP());

  // Inicializar la librería de ThingSpeak
  ThingSpeak.begin(client);
}

void loop() {
  unsigned long ahora = millis();

  // 0. Mantener el LED de vibración (se enciende al vibrar y se apaga solo)
  actualizarLedVibracion();

  // 1. Leer sensores cada 2 segundos
  if (ahora - ultimaLecturaSensores > INTERVALO_SENSORES) {
    ultimaLecturaSensores = ahora;
    leerTodosLosSensores();
  }

  // 2. Enviar datos a ThingSpeak cada 20 segundos
  if (ahora - ultimaSubidaThingSpeak > INTERVALO_THINGSPEAK) {
    ultimaSubidaThingSpeak = ahora;
    enviarDatosAThingSpeak();
  }
}

// =========================================================================
// FUNCIONES AUXILIARES
// =========================================================================

void leerTodosLosSensores() {
  // ---- Leer Presión (BME/BMP280) ----
  if (!presionDisponible) {
    iniciarSensorPresion(); // Reintenta si no se encontró al arrancar
  }
  presionValida = false;
  if (presionDisponible) {
    float p = bmp.readPressure() / 100.0; // Pa -> hPa
    if (!isnan(p) && p > 300.0 && p < 1100.0) {
      presion = p;
      presionValida = true;
      guardarMuestraPresion(millis());
      Serial.print("[Sensor] Presión: "); Serial.print(presion, 1); Serial.print(" hPa | ");
    } else {
      Serial.print("Error leyendo presión | ");
    }
  } else {
    Serial.print("Sin sensor de presión | ");
  }

  // ---- Vibraciones (solo informativo; el conteo lo hace la interrupción) ----
  unsigned long vib;
  portENTER_CRITICAL(&muxVibracion);
  vib = contadorVibraciones;
  portEXIT_CRITICAL(&muxVibracion);
  Serial.print("Vibraciones acumuladas: "); Serial.print(vib); Serial.print(" | ");

  // ---- Leer DHT22 ----
  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (!isnan(h) && !isnan(t)) {
    temperatura = t;
    humedad = h;
    sensacionTermica = calcularSensacionTermica(t, h);
    puntoRocio = calcularPuntoRocio(t, h);
    dhtValido = true;

    Serial.print("Temp: "); Serial.print(temperatura); Serial.print("°C | ");
    Serial.print("Hum: "); Serial.print(humedad); Serial.println("%");

    // ---- Riesgo de lluvia ----
    riesgoLluvia = calcularRiesgoLluvia();
    riesgoValido = true;
  } else {
    dhtValido = false;
    riesgoValido = false;
    Serial.println("Error leyendo el DHT22");
  }
}

void enviarDatosAThingSpeak() {
  Serial.println("--- Subiendo datos a ThingSpeak... ---");

  // Toma el número de vibraciones del periodo y reinicia el contador
  unsigned long vibraciones;
  portENTER_CRITICAL(&muxVibracion);
  vibraciones = contadorVibraciones;
  contadorVibraciones = 0;
  portEXIT_CRITICAL(&muxVibracion);

  // Asignar cada variable a su Field correspondiente
  if (riesgoValido) {
    ThingSpeak.setField(1, riesgoLluvia); // Si falla el DHT22, el campo se envía vacío
  }
  ThingSpeak.setField(2, temperatura);
  ThingSpeak.setField(3, humedad);
  ThingSpeak.setField(4, sensacionTermica);
  ThingSpeak.setField(5, puntoRocio);
  ThingSpeak.setField(7, (long) vibraciones);
  if (presionValida) {
    ThingSpeak.setField(8, presion); // Si el sensor falla, el campo se envía vacío
  }

  // Enviar (writeFields envía todos los campos de una sola vez)
  int codigoRespuesta = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);

  if (codigoRespuesta == 200) {
    Serial.println("¡Actualización exitosa en ThingSpeak!");
  } else {
    Serial.print("Error subiendo datos. Código HTTP: ");
    Serial.println(codigoRespuesta);
  }
}

// ---- Riesgo de lluvia ----
float limitar01(float x) {
  if (x < 0) return 0;
  if (x > 1) return 1;
  return x;
}

// Guarda una muestra de presión cada 10 minutos (últimas 3 horas)
void guardarMuestraPresion(unsigned long ahora) {
  if (historialCount > 0 && (ahora - ultimaMuestraHistorial) < INTERVALO_HISTORIAL) return;
  ultimaMuestraHistorial = ahora;

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
  // Humedad: 0 a 35 puntos
  float compHum = 35.0 * limitar01((humedad - HUM_MIN) / (HUM_MAX - HUM_MIN));

  // Diferencia temperatura - punto de rocío: 0 a 30 puntos
  float dif = temperatura - puntoRocio;
  float compRocio = 30.0 * limitar01((DIF_ROCIO_MAX - dif) / (DIF_ROCIO_MAX - DIF_ROCIO_MIN));

  // Tendencia de presión (cambio llevado a 3 horas): 0 a 35 puntos
  float compPresion = 0;
  bool hayTendencia = false;
  if (historialCount >= 2) {
    float minutos = (historialTiempo[historialCount - 1] - historialTiempo[0]) / 60000.0;
    if (minutos >= MIN_MINUTOS_TENDENCIA) {
      float cambio = historialPresion[historialCount - 1] - historialPresion[0];
      tendenciaPresion3h = cambio * (180.0 / minutos);
      compPresion = 35.0 * limitar01((-tendenciaPresion3h - CAIDA_MIN_HPA) / (CAIDA_MAX_HPA - CAIDA_MIN_HPA));
      hayTendencia = true;
    }
  }

  float riesgo;
  if (hayTendencia) {
    riesgo = compHum + compRocio + compPresion;
  } else {
    riesgo = (compHum + compRocio) * 100.0 / 65.0; // Sin tendencia: se reescala a 0-100
  }

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

// ---- Sensor de vibraciones SW-420 ----
// Interrupción: cuenta un evento cada vez que la salida DO cambia
// desde su nivel de reposo hacia el nivel de vibración.
void IRAM_ATTR isrVibracion() {
  unsigned long t = millis();
  if (t - ultimoEventoMs > REBOTE_MS) {
    ultimoEventoMs = t;
    portENTER_CRITICAL_ISR(&muxVibracion);
    contadorVibraciones++;
    portEXIT_CRITICAL_ISR(&muxVibracion);
    ledPendiente = true;
  }
}

void configurarVibracion() {
  pinMode(PIN_VIBRACION, INPUT);

  // Detecta el nivel de reposo (mantén el sensor quieto al encender).
  // Según el fabricante del módulo, DO en reposo puede ser HIGH o LOW.
  int altos = 0;
  for (int i = 0; i < 20; i++) {
    altos += digitalRead(PIN_VIBRACION);
    delay(10);
  }
  nivelReposo = (altos >= 10) ? HIGH : LOW;

  int flanco = (nivelReposo == HIGH) ? FALLING : RISING;
  attachInterrupt(digitalPinToInterrupt(PIN_VIBRACION), isrVibracion, flanco);

  Serial.print("[SW-420] Nivel en reposo: ");
  Serial.println(nivelReposo == HIGH ? "HIGH" : "LOW");
}

// Enciende el LED de la placa al detectar vibración y lo apaga a los 300 ms
void actualizarLedVibracion() {
  unsigned long ahora = millis();
  if (ledPendiente) {
    ledPendiente = false;
    ledEncendido = true;
    ledEncendidoDesde = ahora;
    digitalWrite(PIN_LED_VIBRACION, HIGH);
  }
  if (ledEncendido && (ahora - ledEncendidoDesde >= LED_DURACION_MS)) {
    ledEncendido = false;
    digitalWrite(PIN_LED_VIBRACION, LOW);
  }
}

// ---- Sensor de presión BME280 / BMP280 ----
// Lee el ID del chip por I2C: 0x58 = BMP280, 0x60 = BME280.
uint8_t leerIdChip(uint8_t direccion) {
  Wire.beginTransmission(direccion);
  Wire.write(0xD0); // Registro del ID
  if (Wire.endTransmission() != 0) return 0;
  if (Wire.requestFrom(direccion, (uint8_t) 1) != 1) return 0;
  return Wire.read();
}

void iniciarSensorPresion() {
  const uint8_t direcciones[] = {0x76, 0x77}; // SDO a GND = 0x76, SDO a 3V3 = 0x77
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t id = leerIdChip(direcciones[i]);
    if (id == 0x56 || id == 0x57 || id == 0x58 || id == 0x60) {
      // La librería BMP280 también sirve para presión en un BME280
      // si se le indica su ID (0x60).
      if (bmp.begin(direcciones[i], id)) {
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                        Adafruit_BMP280::SAMPLING_X2,   // temperatura
                        Adafruit_BMP280::SAMPLING_X16,  // presión
                        Adafruit_BMP280::FILTER_X16,
                        Adafruit_BMP280::STANDBY_MS_500);
        presionDisponible = true;
        Serial.print("[BMx280] Encontrado: ");
        Serial.print(id == 0x60 ? "BME280" : "BMP280");
        Serial.print(" en 0x");
        Serial.println(direcciones[i], HEX);
        return;
      }
    }
  }
  presionDisponible = false;
  Serial.println("[BMx280] No se encontró el sensor de presión. Revisa SDA, SCL, VCC, GND, CSB y SDO.");
}

// Función psicrométrica: Punto de Rocío
float calcularPuntoRocio(float tC, float hr) {
  float a = ((17.27 * tC) / (237.7 + tC)) + log(hr / 100.0);
  return (237.7 * a) / (17.27 - a);
}

// Función psicrométrica: Sensación Térmica
float calcularSensacionTermica(float tC, float hr) {
  if (tC < 27.0) return tC;
  float tF = tC * 9.0 / 5.0 + 32.0;
  float hi = -42.379 + 2.04901523 * tF + 10.14333127 * hr
             - 0.22475541 * tF * hr - 0.00683783 * tF * tF
             - 0.05481717 * hr * hr + 0.00122874 * tF * tF * hr
             + 0.00085282 * tF * hr * hr - 0.00000199 * tF * tF * hr * hr;
  return (hi - 32.0) * 5.0 / 9.0;
}
