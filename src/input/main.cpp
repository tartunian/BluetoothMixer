#include <Arduino.h>
#include "driver/dac.h"
#include <BluetoothA2DPSink.h>

#define MASTER_BCK_IO 15
#define MASTER_WS_IO 25
#define DATA_OUT_IO 22
#define ADC1_CHANNEL_4_IO 32
#define I2S0_DATA_OUT_IDX I2S0O_DATA_OUT23_IDX
#define I2S0_DATA_IN_IDX I2S0I_DATA_IN15_IDX
#define I2S1_DATA_OUT_IDX I2S1O_DATA_OUT23_IDX
#define I2S1_DATA_IN_IDX I2S1I_DATA_IN15_IDX

#define SAMPLE_RATE 44100
#define SAMPLE_BITS 16

BluetoothA2DPSink sink;

// master driver installed and send data
i2s_config_t master_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = (i2s_bits_per_sample_t)SAMPLE_BITS,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 10,
    .dma_buf_len = 1024,
    .use_apll = 0};
i2s_pin_config_t master_pin_config = {
    .bck_io_num = MASTER_BCK_IO,
    .ws_io_num = MASTER_WS_IO,
    .data_out_num = DATA_OUT_IO,
    .data_in_num = -1};

size_t i2sBytesWritten;
size_t frame_count = 0;

void audioDataReceived(const uint8_t *samples, uint32_t length)
{

  // Here is where the audio samples come in from the Bluetooth buffer
  // These need to be sent to another board to be mixed with the samples from
  // the other 'Mixer Input 2'

  if (i2s_write(I2S_NUM_0, (void *)samples, length, &i2sBytesWritten, portMAX_DELAY) != ESP_OK)
  {
    ESP_LOGI(BT_AV_TAG, "i2s_write: FAIL");
  }
  else
  {
    ESP_LOGI(BT_AV_TAG, "i2s_write: OK");
    for(int i=0; i<i2sBytesWritten; i+=4) {
      // int16_t i16sampleLeft = (samples[i] << 8) + samples[i+1];
      // double ratio = 1.0 * i16sampleLeft / 65536;
      // int8_t i8sampleLeft = ratio * INT8_MAX;
      // ESP_LOGI(BT_AV_TAG, "%4d %6d %4d %9f", i, i16sampleLeft, i8sampleLeft, ratio);
      dacWrite(26, samples[i]);
    }
  }
}

void setup()
{
  Serial.begin(921600);

  sink.set_i2s_config(master_i2s_config);
  sink.set_pin_config(master_pin_config);

  sink.set_stream_reader(audioDataReceived);
  sink.start("Mixer Input 1");

  Serial.println("Mixer Input 1 Started....");
}

void loop()
{
  // Serial.println("Mixer Input 1 Loop....");
}