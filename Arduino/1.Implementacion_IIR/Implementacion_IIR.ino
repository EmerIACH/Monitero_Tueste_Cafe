#include <Arduino.h>
#include <driver/i2s.h>

#define SCK_PIN 33  // Pin del reloj
#define WS_PIN 25   // Pin de selección de palabra
#define SD_PIN 32   // Pin de datos
#define BUFFER_SIZE 128  // Tamaño del buffer
#define SAMPLE_RATE 48000  // Frecuencia de muestreo

const i2s_port_t I2S_PORT = I2S_NUM_0;

// Coeficientes del filtro IIR (diseñados para 48 kHz)
float b[] = {0.5658, 0, -0.5658};  // Coeficientes del numerador
float a[] = {1, 0, -0.1317};       // Coeficientes del denominador

// Buffers para el filtro IIR
float x_buf[3] = {0, 0, 0};  // Buffer para las entradas anteriores
float y_buf[3] = {0, 0, 0};  // Buffer para las salidas anteriores

// Función del filtro IIR
float iir_filter(float input, float *b, float *a, float *x_buf, float *y_buf) {
    // Desplazar los valores anteriores en los buffers
    x_buf[2] = x_buf[1];
    x_buf[1] = x_buf[0];
    x_buf[0] = input;

    y_buf[2] = y_buf[1];
    y_buf[1] = y_buf[0];

    // Calcular la salida del filtro
    y_buf[0] = b[0] * x_buf[0] + b[1] * x_buf[1] + b[2] * x_buf[2]
               - a[1] * y_buf[1] - a[2] * y_buf[2];

    return y_buf[0];
}

void setup() {
    Serial.begin(921600);
    Serial.println("Configurando I2S...");

    // Configuración del I2S
    const i2s_config_t i2s_config = {
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

    // Configuración de los pines I2S
    const i2s_pin_config_t pin_config = {
        .bck_io_num = SCK_PIN,
        .ws_io_num = WS_PIN,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = SD_PIN
    };

    // Instalar el driver I2S
    esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("Error instalando el driver: %d\n", err);
        while (true);
    }

    // Configurar los pines I2S
    err = i2s_set_pin(I2S_PORT, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("Error configurando los pines: %d\n", err);
        while (true);
    }

    Serial.println("I2S inicializado");
}

void loop() {
    static int32_t samples[BUFFER_SIZE];
    size_t bytesRead;

    // Leer datos del I2S
    esp_err_t err = i2s_read(I2S_PORT, (char*)samples, sizeof(samples), &bytesRead, portMAX_DELAY);

    if (err == ESP_OK && bytesRead > 0) {
        int numSamples = bytesRead / sizeof(int32_t);

        for (int i = 0; i < numSamples; i++) {
            // Convertir la muestra a un valor flotante
            float input_sample = (float)samples[i];

            // Aplicar el filtro IIR
            float output_sample = iir_filter(input_sample, b, a, x_buf, y_buf);

            // Enviar la señal sin filtrar y filtrada al Serial Plotter
            Serial.print(input_sample);  // Señal sin filtrar
            Serial.print(" ");
            Serial.println(output_sample);  // Señal filtrada
        }
    }
}