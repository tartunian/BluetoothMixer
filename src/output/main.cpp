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
    .dma_buf_count = 6,
    .dma_buf_len = 100,
    .use_apll = 0};
i2s_pin_config_t slave_pin_config = {
    .bck_io_num = SLAVE_BCK_IO,
    .ws_io_num = SLAVE_WS_IO,
    .data_out_num = -1,
    .data_in_num = DATA_IN_IO,
};

std::vector<const char *> device_names = {"TOZO-T10", "DEVICE2", "DEVICE3"};

// int32_t sample;
char sample[128];
size_t bytes_read;

// The supported audio codec in ESP32 A2DP is SBC. SBC audio stream is encoded
// from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data
int32_t get_data_channels(Frame *frame, int32_t channel_len)
{

  Serial.println("get_data");
  for(int i=0; i<channel_len; i++) {
    frame[i].channel1 = frame[i].channel2 = sample[i];
  Serial.println(sample[i]);
  }

  // static double m_time = 0.0;
  // double m_amplitude = 10000.0; // -32,768 to 32,767
  // double m_deltaTime = 1.0 / 44100.0;
  // double m_phase = 0.0;
  // double double_Pi = M_PI * 2.0;
  // // fill the channel data
  // for (int sample = 0; sample < channel_len; ++sample)
  // {
  //   double angle = double_Pi * c3_frequency * m_time + m_phase;
  //   frame[sample].channel1 = m_amplitude * sin(angle);
  //   frame[sample].channel2 = frame[sample].channel1;
  //   m_time += m_deltaTime;
  // }

  return channel_len;
}

void setup()
{
  Serial.begin(115200);

  i2s_driver_install(I2S_NUM_1, &slave_i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &slave_pin_config);

  a2dp_source.start(device_names, get_data_channels);
  a2dp_source.disconnect();
  //a2dp_source.set_auto_reconnect("b0:05:94:eb:52:70");
  a2dp_source.set_reset_ble(true);

}

void loop()
{
  if (i2s_read(I2S_NUM_1, &sample, 128, &bytes_read, portMAX_DELAY) != ESP_OK)
  {
    ESP_LOGE(BT_AV_TAG, "i2s_read failed");
  }
  else
  {
    ESP_LOGI(BT_AV_TAG, "i2s_read value: %d", sample);
  }
}