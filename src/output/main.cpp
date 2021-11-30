#include <Arduino.h>
#include "BluetoothA2DPSource.h"
#include <math.h>
#include <vector>
#include "driver/i2s.h"
#include "config.h"

#define SLAVE_BCK_IO 33
#define SLAVE_WS_IO 32
#define DATA_IN_IO 35

#define c3_frequency 130.81

BluetoothA2DPSource a2dp_source;

i2s_port_t i2s_port_1;
i2s_port_t i2s_port_2;

i2s_config_t slave_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 10,
    .dma_buf_len = 1024,
    .use_apll = 0
};

i2s_pin_config_t slave_pin_config_1 = {
    .bck_io_num = OUTPUT_BCK1_PIN,
    .ws_io_num = OUTPUT_WS1_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = OUTPUT_DATA1_PIN,
};

i2s_pin_config_t slave_pin_config_2 = {
    .bck_io_num = OUTPUT_BCK2_PIN,
    .ws_io_num = OUTPUT_WS2_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = OUTPUT_DATA2_PIN,
};

std::vector<const char *> device_names = {"TOZO-T10", "Bose On-Ear Wireless", "Taylor's AirPods"};

// Closer to zero: more input 1
// Closer to one: more input 2
float balance;

int8_t data_1[4096];
int8_t data_2[4096];
int8_t data_3[4096];
size_t bytes_read;
uint8_t frame_count = 0;

// The supported audio codec in ESP32 A2DP is SBC. SBC audio stream is encoded
// from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data
int32_t get_data_channels(Frame *frame, int32_t channel_len)
{
  memcpy(frame, &data_3[frame_count * 512], channel_len * 4);
  frame_count = (frame_count + 1) % 8;
  return channel_len;
}

QueueHandle_t i2s_event_queue;
TaskHandle_t i2sReaderTaskHandle;
void i2sReaderTask(void *param)
{
  while (true)
  {
    balance = analogRead(13) / 4095.0f;
    i2s_read(I2S_NUM_0, data_1, 4096, &bytes_read, 10 / portTICK_PERIOD_MS);
    i2s_read(I2S_NUM_1, data_2, 4096, &bytes_read, 10 / portTICK_PERIOD_MS);
    for(int i=0; i<4096; i++) {
      data_3[i] = (1-balance) * 0.7f * data_1[i] + balance * 0.7f * data_2[i];
    }
    // ESP_LOGI(BT_AV_TAG, "i2s_read read %d bytes", bytes_read);
  }
}

TaskHandle_t statusReporterTaskHandle;
void statusReporterTask(void *param)
{
  while (true)
  {
    ESP_LOGI(BT_AV_TAG, "Periodic Status Report:");
    ESP_LOGI(BT_AV_TAG, "\t - balance: %d%% %c", abs(0.5f-balance)*200, balance < 0.5f ? 'L' : 'R');
    ESP_LOGI(BT_AV_TAG, "\t - frame_count: %d/8", frame_count);
    ESP_LOGI(BT_AV_TAG, "\t - bytes in buffer: %d", bytes_read);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(921600);

  i2s_driver_install(I2S_NUM_0, &slave_i2s_config, 4, &i2s_event_queue);
  i2s_set_pin(I2S_NUM_0, &slave_pin_config_1);
  // i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);

  i2s_driver_install(I2S_NUM_1, &slave_i2s_config, 4, &i2s_event_queue);
  i2s_set_pin(I2S_NUM_1, &slave_pin_config_2);

  disableCore0WDT();

  a2dp_source.start(device_names, get_data_channels);
  a2dp_source.disconnect();
  // a2dp_source.set_auto_reconnect("00:28:ab:cd:d4:9d");

  a2dp_source.set_reset_ble(true);

  xTaskCreate(i2sReaderTask, "i2s Reader", 1024, &i2s_event_queue, 3, &i2sReaderTaskHandle);
  ESP_LOGI(BT_AV_TAG, "Created i2sReaderTask");

  xTaskCreate(statusReporterTask, "Status Reporter", 2048, NULL, 4, &statusReporterTaskHandle);
  ESP_LOGI(BT_AV_TAG, "Created statusReporterTask");
}

i2s_event_t evt;
void loop()
{
}