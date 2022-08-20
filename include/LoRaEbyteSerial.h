/*
 Name:		EBYTE UART 1262 - LoRaEbyteSerial.h
 Created:	03 October 2021
 Author:	Didier Coyman
 MIT License
 Copyright (c) 2021 Didier  Coyman
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

#ifndef Arduino_h
#include "Arduino.h"
#endif
#include "FunctionalInterrupt.h"

#define LoRa_Ebyte_Serial_Debug
#define SERIAL_BAUD_RATE 115200

// Ebyte E22 delay switching mode of >2ms after the AUX output HIGH as per the manufacturer recommendation
constexpr uint8_t delaySwitch = 3;

// Ebyte E22 module readyness timeout
constexpr uint16_t moduleTimeout = 10000;

// Tranmission time (ms) for 240 bytes
enum E22_Transmission_AirDataRate
{
    TRANSMISSION_ADR_300 = 8800,
    TRANSMISSION_ADR_1200 = 2550,
    TRANSMISSION_ADR_2400 = 1330, // Default
    TRANSMISSION_ADR_4800 = 500,
    TRANSMISSION_ADR_9600 = 305,
    TRANSMISSION_ADR_19200 = 190,
    TRANSMISSION_ADR_38400 = 110,
    TRANSMISSION_ADR_62500 = 77
};

// Uart Baud Rate
enum E22_UartBaudRate
{
    UART_BAUD_RATE_NA = 0,
    UART_BAUD_RATE_1200 = 1200,
    UART_BAUD_RATE_2400 = 2400,
    UART_BAUD_RATE_4800 = 4800,
    UART_BAUD_RATE_9600 = 9600, // Default
    UART_BAUD_RATE_19200 = 19200,
    UART_BAUD_RATE_38400 = 38400,
    UART_BAUD_RATE_57600 = 57600,
    UART_BAUD_RATE_115200 = 115200
};

// Binary Uart baud rate
enum E22_UartBaudRate_Binary
{
    UART_BAUD_RATE_1200_BINARY = 0b000,
    UART_BAUD_RATE_2400_BINARY = 0b001,
    UART_BAUD_RATE_4800_BINARY = 0b010,
    UART_BAUD_RATE_9600_BINARY = 0b011, // Default
    UART_BAUD_RATE_19200_BINARY = 0b100,
    UART_BAUD_RATE_38400_BINARY = 0b101,
    UART_BAUD_RATE_57600_BINARY = 0b110,
    UART_BAUD_RATE_115200_BINARY = 0b111
};

// Air Data Rate
enum E22_AirDataRate
{
    AIR_DATA_RATE_NA = 0,
    AIR_DATA_RATE_300 = 300,
    AIR_DATA_RATE_1200 = 1200,
    AIR_DATA_RATE_2400 = 2400, // Default
    AIR_DATA_RATE_4800 = 4800,
    AIR_DATA_RATE_9600 = 9600,
    AIR_DATA_RATE_19200 = 19200,
    AIR_DATA_RATE_38400 = 38400,
    AIR_DATA_RATE_62500 = 62500
};

// Air Data Rate - registry 0 bit representation
enum E22_AirDataRate_Binary
{
    AIR_DATA_RATE_300_BINARY = 0b000,
    AIR_DATA_RATE_1200_BINARY = 0b001,
    AIR_DATA_RATE_2400_BINARY = 0b010, // Default
    AIR_DATA_RATE_4800_BINARY = 0b011,
    AIR_DATA_RATE_9600_BINARY = 0b100,
    AIR_DATA_RATE_19200_BINARY = 0b101,
    AIR_DATA_RATE_38400_BINARY = 0b110,
    AIR_DATA_RATE_62500_BINARY = 0b111
};

// Sub Packet size
enum E22_PacketSize
{
    PACKET_SIZE_240 = 240, // Default
    PACKET_SIZE_128 = 128,
    PACKET_SIZE_64 = 64,
    PACKET_SIZE_32 = 32,
    PACKET_SIZE_NA = 0
};

// Sub Packet size - registry 1 bit representation
enum E22_PacketSize_Binary
{
    PACKET_SIZE_240_BINARY = 0b00, // Default
    PACKET_SIZE_128_BINARY = 0b01,
    PACKET_SIZE_64_BINARY = 0b10,
    PACKET_SIZE_32_BINARY = 0b11
};

// Transmitting power
enum E22_TransmissionPower
{
    POWER_NA = 0,
    POWER_22 = 22,
    POWER_17 = 17,
    POWER_13 = 13,
    POWER_10 = 10
};

// Transmitting power - registry 2 bit representation
enum E22_TransmissionPower_Binary
{
    POWER_22_BINARY = 0b00,
    POWER_17_BINARY = 0b01,
    POWER_13_BINARY = 0b10,
    POWER_10_BINARY = 0b11
};

// Transmitting power - registry 2 bit representation
enum E22_WorCycle
{
    WOR_NA = 0,
    WOR_0500 = 500,
    WOR_1000 = 1000,
    WOR_1500 = 1500,
    WOR_2000 = 2000, // Default
    WOR_2500 = 2500,
    WOR_3000 = 3000,
    WOR_3500 = 3500,
    WOR_4000 = 4000
};

// Transmitting power - registry 2 bit representation
enum E22_WorCycle_Binary
{
    WOR_0500_BINARY = 0b000,
    WOR_1000_BINARY = 0b001,
    WOR_1500_BINARY = 0b010,
    WOR_2000_BINARY = 0b011, // Default
    WOR_2500_BINARY = 0b100,
    WOR_3000_BINARY = 0b101,
    WOR_3500_BINARY = 0b110,
    WOR_4000_BINARY = 0b111
};

// Operating Mode
enum E22T_Mode_t
{
    Normal = 0b00,
    WorkOnRadio = 0b01,
    Configuration = 0b10,
    DeepSleep = 0b11
};

// Device communication type
enum E22_Device_ComType
{
    UART,
    Wireless,
};

// Commit registry type
enum E22_Device_Storage
{
    Temporary = 1,
    Permanent = 2,
};

// Command code
enum E22_Command_t
{
    Set_Permanent_Register = 0xC0,
    Read_Register = 0xC1,
    Set_Temporary_Register = 0xC2,
    Response = 0xC1,
    Over_Wireless = 0xCF,
    Wrong = 0xFF,
};

// Registry mapping
enum E22_Register_Addr
{
    Address = 0x00,
    Crypto = 0x07,
    ProductInfo = 0x80
};

// Registry length
enum E22_Register_Length
{
    Address_length = 2,
    Registry_length = 7,
    Crypto_length = 2,
    ProductInfo_length = 7
};

/*
    Structure for the product information registry
    Starting from 80H, they are NAME_H, NAME_L, VER, MAX_POWER, FREQ, TYPE, RESERVE.
*/
struct ProductInfoStructure
{
    byte Name_H = 0;
    byte Name_L = 0;
    byte Version = 0;
    byte MaxPower = 0;
    byte Freq = 0;
    byte Type = 0;
    byte Reserve = 0;
};

/*
    Structure for the general module registry
    Starting from 00H, they are ADD_H, ADD_L, NETID, REG0, REG1, REG2, REG3.
*/
struct RegistryStructure
{
    byte Add_H = 0;
    byte Add_L = 0;
    byte NetID = 0;
    byte Reg0 = 0;
    byte Reg1 = 0;
    byte Reg2 = 0;
    byte Reg3 = 0;
};

/*
    Structure for the crypto registry
*/
struct CryptoStructure
{
    byte Crupto_H = 0;
    byte Crypto_L = 0;
};

/*
    Structure for sending message in fixed point transmission
*/
struct FixedTransmissionMessage
{
    byte Add_H = 0;
    byte Add_L = 0;
    byte Channel = 0;
    char *Message;
};

/* 
    Structure bit fields
*/
struct Registry0
{
    uint8_t E22_UartBaudRate : 3;
    uint8_t E22_UartParityBit : 2;
    uint8_t E22_AirDataRate : 3;
};

// Class definition for Ebyte Serial LoRa devices
class LoRa_Ebyte_Serial
{
public:
    LoRa_Ebyte_Serial(HardwareSerial *hs, int8_t m0Pin = -1, int8_t m1Pin = -1, int8_t auxPin = -1, int8_t rxPin = -1, int8_t txPin = -1);
    ~LoRa_Ebyte_Serial();

    // init return true if communication could be established with device, otherwise false
    bool Init();

    // variable that indicate if the initialization was succesful or not
    bool Initialized();

    // set or get the module mode
    bool SetModuleMode(E22T_Mode_t mode);
    E22T_Mode_t GetModuleMode();

    // discover the actual uart baud rate speed
    uint32_t ScanUartSpeed();

    // reset to factory default
    bool ResetToFactoryDefault();

    // select the device communication type
    void SelectDeviceComType(E22_Device_ComType selectDeviceComType);

    // select the registry to store the modification
    void SelectStorage(E22_Device_Storage selectStorage);

    // registry read/write
    uint16_t GetModuleName();
    uint8_t GetModuleVersion();
    uint8_t GetModuleMaxPower();
    bool GetRSSI(uint8_t &ambiantRSSI, uint8_t &lastRSSI);

    uint16_t GetModuleAddress();
    void SetModuleAddress(uint16_t moduleaddr);

    uint8_t GetNetID();
    void SetNetID(uint8_t netID);

    E22_UartBaudRate GetUartBaudRate();
    void SetUartBaudRate(E22_UartBaudRate baudrate);

    E22_AirDataRate GetAirDataRate();
    void SetAirDataRate(E22_AirDataRate airdatarate);

    E22_PacketSize GetPacketSize();

    bool GetRssiNoiseMode();
    void SetRssiNoiseMode(bool enabled);

    E22_TransmissionPower GetTransmittingPower();
    void SetTransmittingPower(E22_TransmissionPower transPower);

    uint8_t GetChannel();
    void SetChannel(uint8_t channel);

    bool GetRssiMode();
    void SetRssiMode(bool enabled);

    bool GetFixPointTransmissionMode();
    void SetFixPointTransmissionMode(bool enable);

    bool GetEnableReply();
    void SetEnableReply(bool enable);

    bool GetLookBeforeTransmit();
    void SetLookBeforeTransmit(bool enable);

    bool GetWakeOnRadioMode();
    void SetWakeOnRadioMode(bool wrMode);

    E22_WorCycle GetWakeOnRadioCycle();
    void SetWakeOnRadioCycle(E22_WorCycle worCycle);

    void SetEncryption(uint16_t encryption);

    bool CompareRegistry(RegistryStructure *regStruct1, RegistryStructure *regStruct2);

    bool CommitChanges();

    // send messages over lora in fixed or broadcast transmission
    bool SendMessage(const char *msgbuffer);
    bool SendMessage(FixedTransmissionMessage *fmsgbuffer);

    // receive messages over lora in fixed or broadcast transmission
    bool ReceiveMessage(char *msgbuffer, size_t bufsize, size_t &msglenght, uint8_t &rssi, bool wait);

protected:
    // interrupt handler for the Aux line
    void IRAM_ATTR InterruptRoutine();

private:
    // external reference to the selected hardware serial port
    HardwareSerial *hs = nullptr;

    // convert binary baud rate to decimal baud rate
    E22_UartBaudRate ConvertUartBaudRateBin2Dec(uint8_t binBaudRate);
    // convert decimal baud rate to binary
    uint8_t ConvertUartBaudRateDec2Bin(uint32_t decBaudRate);
    // convert binary baud rate to decimal baud rate
    E22_AirDataRate ConvertAirDataRateBin2Dec(uint8_t binBaudRate);
    // convert decimal baud rate to binary
    uint8_t ConvertAirDataRateDec2Bin(uint32_t decBaudRate);
    // convert binary packet size to decimal
    E22_PacketSize ConvertPacketSizeBin2Dec(uint8_t binPacketSize);
    // convert decimal packet size to binary
    uint8_t ConvertPacketSizeDec2Bin(uint8_t decPacketSize);
    // convert binary transmission power to decimal
    E22_TransmissionPower ConvertTransPowerBin2Dec(uint8_t binTransPower);
    // convert decimal transmission power to binary
    uint8_t ConvertTransPowerDec2Bin(uint8_t decTransPower);
    // convert WOR cycle to decimal
    E22_WorCycle ConvertWorCycleBin2Dec(uint8_t binWorCycle);
    // convert decimal WOR cycle to binary
    uint8_t ConvertWorCycleDec2Bin(uint8_t decWorCycle);

    // flag to confirm initialization
    bool initialized = false;

    // rssi enable after transmission
    bool rssiTrans = false;

    // crypto active
    bool cryptoOn = false;

    // selectable MCU pin that will be hooked to the Ebyte device
    int8_t m0_Pin;
    int8_t m1_Pin;
    int8_t aux_Pin;
    int8_t rx_Pin;
    int8_t tx_Pin;

    // last module mode, needed to manage the module transition
    E22T_Mode_t lastModuleMode = E22T_Mode_t::DeepSleep;

    // select device for communication
    E22_Device_ComType deviceComType = E22_Device_ComType::UART;

    // selected register for modification
    E22_Device_Storage selectedRegister = E22_Device_Storage::Permanent;

    // handle for interrupt notification on Aux
    TaskHandle_t taskToNotify;

    // handle for mutex to access the class only one at the time
    SemaphoreHandle_t semaphoreHandle = NULL;
    StaticSemaphore_t staticMutex;

    // verify the module readyness
    bool ModuleReady(bool checkAux, uint16_t timeout);

    // registry function access
    void SetPacketSize(E22_PacketSize packetSize);

    // wait for transmission complete
    void WaitForTransmission(size_t msglen);

    // Structure for the product information, registry, crypto
    ProductInfoStructure productSerialInfo, productWirelessInfo;

    RegistryStructure registrySerialRead, registryWirelessRead, registrySerialTemp, registryWirelessTemp, registrySerialPerm, registryWirelessPerm;
    RegistryStructure *currentStorageRegistry, *currentReadRegistry;

    CryptoStructure cryptoData;

    bool GetAllParameters();
};