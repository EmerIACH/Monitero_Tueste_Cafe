#include <Arduino.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

// Configuración de pines de I2S
#define SCK_PIN 33
#define WS_PIN 25
#define SD_PIN 32

// Pines de salida para las señales de estado/detección
#define PIN_ACTIVO      23  // Indica que el sistema está funcionando
#define PIN_DETECCION2  22  // Se activa cuando en la ventana se registran 2 o menos detecciones válidas
#define PIN_DETECCION3  21  // Se activa cuando en la ventana se registran 3 o más detecciones válidas

// Configuración de audio
#define BUFFER_SIZE 512
#define SAMPLE_RATE 48000
#define N 512
#define THRESHOLD 50000000 //Umbral estatico se ajusta relizando pruebas
#define DEBOUNCE_TIME 500

// Rangos de frecuencia válidos
#define FREQ1_MIN 8000.0
#define FREQ1_MAX 10000.0
#define FREQ2_MIN 14000.0
#define FREQ2_MAX 15000.0

const i2s_port_t I2S_PORT = I2S_NUM_0;

// Buffers para FFT
double vReal[N];
double vImag[N];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, N, SAMPLE_RATE);

// Coeficientes del filtro IIR
double b[] = {0.5658, 0, -0.5658};
double a[] = {1, 0, -0.1317};

// Variables para el procesamiento de audio
double x_buf[3] = {0};
double y_buf[3] = {0};
double sampleBuffer[N];
int bufferIndex = 0;
unsigned long lastPeakTime = 0;

// Variables para la ventana de detección
unsigned long detectionWindowStart = 0;   // Se activa con la primera detección válida
int validDetectionCount = 0;
const unsigned long WINDOW_DURATION = 3000; // Ventana de 3 segundos
bool decisionCommitted = false;             // Indica si ya se tomó la decisión de activar un pin

// Función del filtro IIR
double iir_filter(double input, double *b, double *a, double *x_buf, double *y_buf) {
  // Actualiza los buffers de entrada y salida
  x_buf[2] = x_buf[1];
  x_buf[1] = x_buf[0];
  x_buf[0] = input;

  y_buf[2] = y_buf[1];
  y_buf[1] = y_buf[0];

  y_buf[0] = b[0] * x_buf[0] + b[1] * x_buf[1] + b[2] * x_buf[2]
             - a[1] * y_buf[1] - a[2] * y_buf[2];

  return y_buf[0];
}

// Función para registrar una detección válida
// La ventana se activa con la primera detección válida y se cuenta durante 3 segundos.
void recordDetection() {
  if (decisionCommitted)
    return; // Una vez tomada la decisión, no se cuentan más detecciones

  unsigned long now = millis();
  // Si no se ha iniciado la ventana o si la ventana anterior ya expiró, se reinicia
  if (detectionWindowStart == 0 || (now - detectionWindowStart > WINDOW_DURATION)) {
    detectionWindowStart = now;
    validDetectionCount = 1;
  } else {
    validDetectionCount++;
  }
}

void setup() {
  Serial.begin(115200);
  
  // Configuración de pines de salida
  pinMode(PIN_ACTIVO, OUTPUT);
  pinMode(PIN_DETECCION2, OUTPUT);
  pinMode(PIN_DETECCION3, OUTPUT);
  
  // Indicamos que el sistema está activo
  digitalWrite(PIN_ACTIVO, HIGH);
  // Inicialmente, se apagan los pines de detección
  digitalWrite(PIN_DETECCION2, LOW);
  digitalWrite(PIN_DETECCION3, LOW);
  
  // Inicialmente, no hay ventana activa
  detectionWindowStart = 0;
  validDetectionCount = 0;
  decisionCommitted = false;

  // Configuración de I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = SCK_PIN,
    .ws_io_num = WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = SD_PIN
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

void loop() {
  static int32_t rawSamples[BUFFER_SIZE];
  size_t bytesRead;
  bool bufferTrigger = false;
  double maxAmplitude = 0;  // Amplitud máxima observada en este lote

  // Lectura de datos vía I2S
  i2s_read(I2S_PORT, (char*)rawSamples, sizeof(rawSamples), &bytesRead, portMAX_DELAY);

  if (bytesRead > 0) {
    int numSamples = bytesRead / sizeof(int32_t);
    
    for (int i = 0; i < numSamples; i++) {
      double filteredSample = iir_filter((double)rawSamples[i], b, a, x_buf, y_buf);
      sampleBuffer[bufferIndex] = filteredSample;
      
      if (abs(filteredSample) > maxAmplitude) {
        maxAmplitude = abs(filteredSample);
      }
      
      // Si algún sample supera el umbral, se activa el procesamiento FFT
      if (abs(filteredSample) > THRESHOLD) {
        bufferTrigger = true;
      }
      
      bufferIndex = (bufferIndex + 1) % N;
    }

    if (bufferTrigger) {
      // Se captura una ventana de N muestras para la FFT
      for (int j = 0; j < N; j++) {
        int index = (bufferIndex + j) % N;
        vReal[j] = sampleBuffer[index];
        vImag[j] = 0;
      }
      
      FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
      FFT.compute(FFT_FORWARD);
      FFT.complexToMagnitude();
      
      double peakFreq = FFT.majorPeak();

      // Se utiliza un tiempo de debounce para evitar múltiples eventos seguidos
      if (millis() - lastPeakTime > DEBOUNCE_TIME) {
        if (peakFreq >= FREQ1_MIN && peakFreq <= FREQ1_MAX) {
          Serial.print("Detección válida en Rango 1 (8-10 kHz)! Frecuencia dominante: ");
          Serial.print(peakFreq);
          lastPeakTime = millis();
          recordDetection(); // Se cuenta la detección
        } else if (peakFreq >= FREQ2_MIN && peakFreq <= FREQ2_MAX) {
          Serial.print("Detección válida en Rango 2 (14-15 kHz)! Frecuencia dominante: ");
          Serial.print(peakFreq);
          lastPeakTime = millis();
          recordDetection(); // Se cuenta la detección
        }
      }
    }
  }
  
 // Si ya se inició la ventana y ésta expiró, se toman las decisiones
  if (detectionWindowStart != 0 && (millis() - detectionWindowStart >= WINDOW_DURATION)) {
    if (validDetectionCount >= 3) {
      // Si se cuentan 3 o más detecciones, se activa el PIN 21
      Serial.println("Pin 21 activado (Tueste Oscuro - Segundo Crack).");
      delay(15000);
      digitalWrite(PIN_DETECCION3, HIGH);
      
    } else {
      // Si se cuentan 2 o menos detecciones, se activa el PIN 22
      digitalWrite(PIN_DETECCION2, HIGH);
      Serial.println("Pin 22 activado (Tueste Claro - Primer Crack).");
    }
    
    // Reiniciamos la ventana de detección
    detectionWindowStart = 0;
    validDetectionCount = 0;
  }
}
