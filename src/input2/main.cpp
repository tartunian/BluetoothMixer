#include <Arduino.h>
#include "driver/dac.h"
#include <BluetoothA2DPSink.h>
#include "config.h"

#define MASTER_BCK_IO 18
#define MASTER_WS_IO 19
#define DATA_OUT_IO 21

BluetoothA2DPSink sink;

// master driver installed and send data
i2s_config_t master_i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = 44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = (i2s_comm_format_t)I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 10,
    .dma_buf_len = 1024,
    .use_apll = true,
    .tx_desc_auto_clear = true
};
i2s_pin_config_t master_pin_config = {
    .bck_io_num = INPUT2_BCK_PIN,
    .ws_io_num = INPUT2_WS_PIN,
    .data_out_num = INPUT2_DATA_PIN,
    .data_in_num = -1
};

size_t i2sBytesWritten;
size_t frame_count = 0;
int8_t data[4096];

void audioDataReceived(const uint8_t *samples, uint32_t length)
{
  memcpy(data, samples, length);
  // for(int i=0; i<length; i+=4) {
  //   dacWrite(26, samples[i]);
  //   dacWrite(25, samples[i+2]);
  // }
}

void setup()
{
  Serial.begin(921600);

  sink.set_i2s_port(I2S_NUM_0);
  sink.set_i2s_config(master_i2s_config);
  sink.set_pin_config(master_pin_config);
  // i2s_set_dac_mode(I2S_DAC_CHANNEL_BOTH_EN);

  sink.set_stream_reader(audioDataReceived);
  sink.start("Mixer Input 2");

  Serial.println("Mixer Input 2 Started....");

  // pinMode(32, OUTPUT);
  // digitalWrite(32, LOW);
}

void loop()
{
  // Serial.println("Mixer Input 1 Loop....");
}