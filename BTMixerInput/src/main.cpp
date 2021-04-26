#include <Arduino.h>
#include <BluetoothA2DPSink.h>

BluetoothA2DPSink sink;

void audioDataReceived(const uint8_t* data, uint32_t length) {
  
}

void setup() {
  Serial.begin(9600);
  
  sink.set_stream_reader(audioDataReceived);
  sink.start("Mixer Input 1");

}

void loop() {
}