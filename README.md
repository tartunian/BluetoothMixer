# Bluetooth Mixer
The purpose of this project is to create a multi-input, multi-output Bluetooth audio mixing device. The project utilizes multiple ESP32 microcontrollers for their built-in Bluetooth capabilities. 

## Libraries & Code
The project is built using the Arduino platform and is coded mostly in C++.

To handle the Bluetooth functionality of the ESP32, we make use of the [ESP32-A2DP Arduino library](https://github.com/pschatzmann/ESP32-A2DP) developed by pschatzmann.

## Organization
This project is organized as a Platform.io project in VSCode.

Inside the 'src' folder are subfolders for the separate modules of the device which each have their own 'main.cpp' files. Currently, there are subfolders for 'input' and 'output' modules which handle the Bluetooth input and output. In the near future, we will add code for a mixer/controller module.

## Compilation & Flashing
The compilation and upload processes are configured in the 'platformio.ini' file. Variables that apply to the whole project are listed under the '[env]' tag.

By using platform.io we are able to compile the separate 'main.cpp' files to the respective module/boards at the same time. Each of these modules has their own tag in the 'platformio.ini' file (i.e. btinput, btoutput) which are configured to be flashed on a particular USB port ('upload_port'). The 'src_filter' variable determines which files are included in the compilation and flashing process for each module.

## Wiring
### I2S

| Input Board | Type | Pin | Output Board Pin |
|    :---:     |    :---:     |     :---:     |     :---:       |
| Input 1	| Bit Clock (BCK)	| 18	| 33	|
| Input 1	| Word Select (WS)	| 19	| 32	|
| Input 1	| Data (DATA) 		| 21	| 35	|
| Input 2	| Bit Clock (BCK)	| 13	| 18	|
| Input 2	| Word Select (WS)	| 12	| 19	|
| Input 2	| Data (DATA)		| 14	| 21	|

### Other
Additionally, there is a potentiometer connected to pin 13 of the output board.