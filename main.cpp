#include <driver/i2s.h>
#include <math.h>

// Визначення пінів для мікрофонів
#define I2S_WS     15  // LRCLK
#define I2S_SCK    14  // BCLK

// Піни даних для 8 мікрофонів
#define I2S_SD1    32  // DATA для першого мікрофона
#define I2S_SD2    33  // DATA для другого мікрофона
#define I2S_SD3    25  // DATA для третього мікрофона
#define I2S_SD4    26  // DATA для четвертого мікрофона
#define I2S_SD5    27  // DATA для п'ятого мікрофона
#define I2S_SD6    13  // DATA для шостого мікрофона
#define I2S_SD7    2   // DATA для сьомого мікрофона
#define I2S_SD8    4   // DATA для восьмого мікрофона

#define I2S_PORT   I2S_NUM_0

// Налаштування буфера
#define BUFFER_SIZE 1024
#define SAMPLE_RATE 16000
#define NUM_MICS 8

// Константа для згладжування
const float alpha = 0.1;

// Структура для зберігання даних мікрофона
typedef struct {
    int32_t samples[BUFFER_SIZE];
    float rms;
    float decibels;
    float smoothedValue;
    int dataPin;
    TaskHandle_t taskHandle;
} MicData;

// Масив структур для 8 мікрофонів
MicData mics[NUM_MICS];

// Мютекс для синхронізації доступу до I2S
SemaphoreHandle_t i2s_mutex = NULL;

// Функція для обробки даних мікрофона
void mic_task(void *pvParameters) {
    int micIndex = *((int*)pvParameters);
    size_t bytes_read;
    
    while(true) {
        // Встановлюємо пін для мікрофона
        i2s_pin_config_t pin_config = {
            .bck_io_num = I2S_SCK,
            .ws_io_num = I2S_WS,
            .data_out_num = I2S_PIN_NO_CHANGE,
            .data_in_num = mics[micIndex].dataPin
        };
        
        if (xSemaphoreTake(i2s_mutex, portMAX_DELAY)) {
            // Змінюємо пін для зчитування з цього мікрофона
            i2s_set_pin(I2S_PORT, &pin_config);
            
            // Скидаємо буфер
            i2s_zero_dma_buffer(I2S_PORT);
            
            // Затримка для стабілізації
            vTaskDelay(10 / portTICK_PERIOD_MS);
            
            // Зчитуємо дані
            i2s_read(I2S_PORT, mics[micIndex].samples, sizeof(mics[micIndex].samples), &bytes_read, 100 / portTICK_PERIOD_MS);
            
            // Звільняємо мютекс
            xSemaphoreGive(i2s_mutex);
            
            // Обчислення RMS для мікрофона
            int16_t value;
            mics[micIndex].rms = 0;
            
            for (int i = 0; i < BUFFER_SIZE; i++) {
                value = mics[micIndex].samples[i] >> 14;
                mics[micIndex].rms += value * value;
            }
            
            mics[micIndex].rms = sqrt(mics[micIndex].rms / BUFFER_SIZE);
            
            // Згладжування значень
            mics[micIndex].smoothedValue = (alpha * mics[micIndex].rms) + 
                                          ((1 - alpha) * mics[micIndex].smoothedValue);
            
            // Конвертація в децибели
            if (mics[micIndex].smoothedValue > 0) {
                mics[micIndex].decibels = 20 * log10(mics[micIndex].smoothedValue);
            }
        }
        
        // Даємо час для інших задач
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Запуск програми з вісьмома мікрофонами...");
    
    // Ініціалізація структур даних для мікрофонів
    mics[0].dataPin = I2S_SD1;
    mics[1].dataPin = I2S_SD2;
    mics[2].dataPin = I2S_SD3;
    mics[3].dataPin = I2S_SD4;
    mics[4].dataPin = I2S_SD5;
    mics[5].dataPin = I2S_SD6;
    mics[6].dataPin = I2S_SD7;
    mics[7].dataPin = I2S_SD8;
    
    for (int i = 0; i < NUM_MICS; i++) {
        mics[i].rms = 0;
        mics[i].decibels = 0;
        mics[i].smoothedValue = 0;
    }
    
    // Створюємо мютекс для синхронізації доступу до I2S
    i2s_mutex = xSemaphoreCreateMutex();
    
    // Конфігурація I2S
    i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };
    
    // Ініціалізуємо I2S для першого мікрофона
    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = mics[0].dataPin
    };
    
    // Встановлюємо I2S драйвер
    if (i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL) != ESP_OK) {
        Serial.println("Помилка при інсталяції I2S драйвера");
        return;
    }
    
    if (i2s_set_pin(I2S_PORT, &pin_config) != ESP_OK) {
        Serial.println("Помилка при конфігурації пінів I2S");
        return;
    }
    
    i2s_zero_dma_buffer(I2S_PORT);
    
    // Масив для індексів мікрофонів
    static int micIndices[NUM_MICS];
    
    // Створюємо задачі для обробки мікрофонів
    for (int i = 0; i < NUM_MICS; i++) {
        micIndices[i] = i;
        char taskName[16];
        sprintf(taskName, "mic%d_task", i+1);
        
        xTaskCreatePinnedToCore(
            mic_task,                 // Функція задачі
            taskName,                 // Назва задачі
            4096,                     // Розмір стеку (байт)
            &micIndices[i],           // Параметри (індекс мікрофона)
            1,                        // Пріоритет
            &mics[i].taskHandle,      // Хендл задачі
            i % 2                     // Чергуємо ядра (0 або 1)
        );
        
        // Невелика затримка між створенням задач
        delay(50);
    }
    
    Serial.println("Задачі запущено");
}

void loop() {
    // Виведення значень для Serial Plotter
    for (int i = 0; i < NUM_MICS; i++) {
        Serial.print("Mic");
        Serial.print(i+1);
        Serial.print("_RMS:");
        Serial.print(mics[i].rms);
        Serial.print(" Mic");
        Serial.print(i+1);
        Serial.print("_dB:");
        Serial.print(mics[i].decibels);
        Serial.print(" ");
    }
    Serial.println();
    
    delay(100);
}