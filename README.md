
# Ebyte E22 Serial T serie 📝  
This library has been created to faciliate the communication between an MCU and the Ebyte E22 
serial module.
The code has been developped and tested with the Ebyte E22-900-T22 serie and an ESP32.

## Change Log
Version 1.0:
- The communication with the module is fully functional and tested.
- The wake up over the air mode (WOR) has not been tested.
- The wireless configuration is not complete yet.


## Get Started 🚀  
Install the library 📘 from PlatformIO and connect the Ebyte module to your MCU.

You need to have two hardware serial ports, one for the communiation with the Ebyte module 
and one for debugging, optional.
- one interrupt capable GPIO pin to read the Aux from the Ebyte module.
- two output GPIO pin to control M0 & M1.

The source code also use FreeRTOS.

## Usage
Include the library and declare the class by providing the reference to the hardware serial and 
the pin connected to Aux, M0 and M1.
```cpp
#include <LoRaEbyteSerial.h>

LoRa_Ebyte_Serial lora(&Serial1, 27, 33, 34);
```
Initialize the module:
```cpp
lora.Init();
```
Use SendMessage and ReceiveMessage to send and receive the payload over LoRa.

To apply any configuration change to the module, you have to use the function CommitChange() such like:
```cpp
lora.SetTransmittingPower(E22_TransmissionPower::POWER_10);
lora.SetChannel(9);
lora.CommitChanges();
```

## Comments ✏
I do not recommend to change the communication baud rate as it seems to be badly managed 
by the module.
After such a change, it is often not possible to switch to the configuration mode which has 
a fixed 9600 bps communication speed.
