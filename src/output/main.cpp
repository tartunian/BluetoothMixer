#include <Arduino.h>
#include "BluetoothA2DPSource.h"
#include <math.h>
#include <vector>
#include "driver/i2s.h"

#define MASTER_BCK_IO 15
#define MASTER_WS_IO 25
#define SLAVE_BCK_IO 19
#define SLAVE_WS_IO 26
#define DATA_IN_IO 21
#define DATA_OUT_IO 22
#define ADC1_CHANNEL_4_IO 32
#define I2S0_DATA_OUT_IDX I2S0O_DATA_OUT23_IDX
#define I2S0_DATA_IN_IDX I2S0I_DATA_IN15_IDX
#define I2S1_DATA_OUT_IDX I2S1O_DATA_OUT23_IDX
#define I2S1_DATA_IN_IDX I2S1I_DATA_IN15_IDX

#define SAMPLE_RATE 44100
#define SAMPLE_BITS 16

#define c3_frequency 130.81

BluetoothA2DPSource a2dp_source;

i2s_port_t i2s_port;

i2s_config_t slave_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 10,
    .dma_buf_len = 1024,
    .use_apll = 0};
i2s_pin_config_t slave_pin_config = {
    .bck_io_num = SLAVE_BCK_IO,
    .ws_io_num = SLAVE_WS_IO,
    .data_out_num = -1,
    .data_in_num = DATA_IN_IO,
};

std::vector<const char *> device_names = {"TOZO-T10", "Bose On-Ear Wireless", "Taylor's AirPods"};

int8_t samples[4096];
size_t bytes_read;
uint8_t frame_count = 0;

// The supported audio codec in ESP32 A2DP is SBC. SBC audio stream is encoded
// from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data
int32_t get_data_channels(Frame *frame, int32_t channel_len)
{

  // should write minimum of bytes_read and 4096

  memcpy(frame, &samples[frame_count * 512], channel_len * 4);
  ESP_LOGI(BT_AV_TAG, "Wrote %d samples", channel_len * 4);

  // for(int i=0; i<channel_len; i++) {
  //   dacWrite(25, (frame[i].channel1 >> 8));
  // }

  frame_count = (frame_count + 1) % 8;
  return channel_len;
}

QueueHandle_t i2s_event_queue;
TaskHandle_t i2sReaderTaskHandle;
void i2sReaderTask(void *param)
{
  while (true)
  {
    i2s_read(I2S_NUM_1, samples, 4096, &bytes_read, 2000);
    // ESP_LOGI(BT_AV_TAG, "i2s_read read %d bytes", bytes_read);
  }
}

TaskHandle_t statusReporterTaskHandle;
void statusReporterTask(void *param)
{
  while (true)
  {
    ESP_LOGI(BT_AV_TAG, "Periodic Status Report:");
    ESP_LOGI(BT_AV_TAG, "\t - frame_count: %d/8", frame_count);
    ESP_LOGI(BT_AV_TAG, "\t - bytes in buffer: %d", bytes_read);
    vTaskDelay(2000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  Serial.begin(921600);

  i2s_driver_install(I2S_NUM_1, &slave_i2s_config, 4, &i2s_event_queue);
  i2s_set_pin(I2S_NUM_1, &slave_pin_config);

  disableCore0WDT();

  a2dp_source.start(device_names, get_data_channels);
  a2dp_source.disconnect();
  // a2dp_source.set_auto_reconnect("b0:05:94:eb:52:70");
  a2dp_source.set_auto_reconnect("28:11:a5:1a:74:b5");
  a2dp_source.set_reset_ble(true);

  xTaskCreate(i2sReaderTask, "i2s Reader", 1024, &i2s_event_queue, 3, &i2sReaderTaskHandle);
  ESP_LOGI(BT_AV_TAG, "Created i2sReaderTask");

  xTaskCreate(statusReporterTask, "Status Reporter", 2048, NULL, 4, &statusReporterTaskHandle);
  ESP_LOGI(BT_AV_TAG, "Created statusReporterTask");
}

i2s_event_t evt;
void loop()
{
  // i2s_read(I2S_NUM_1, samples, 4096, &bytes_read, 2000);
  // ESP_LOGI(BT_AV_TAG, "i2s_read read %d bytes", bytes_read);
  // if (xQueueReceive(i2s_event_queue, &evt, 200) == pdPASS)
  // {

  //   if (evt.type == I2S_EVENT_RX_DONE)
  //   {
  //     // samplesReady = 1;
  //     ESP_LOGI(BT_AV_TAG, "I2S_EVENT_RX_DONE");
  //   }
  //   else if (evt.type == I2S_EVENT_DMA_ERROR)
  //   {
  //     ESP_LOGI(BT_AV_TAG, "I2S_EVENT_DMA_ERROR");
  //   }
  //   else if (evt.type == I2S_EVENT_MAX)
  //   {
  //     ESP_LOGI(BT_AV_TAG, "I2S_EVENT_MAX");
  //   }
  //   else if (evt.type == I2S_EVENT_TX_DONE)
  //   {
  //     ESP_LOGI(BT_AV_TAG, "I2S_EVENT_TX_DONE");
  //   }
  // }
}