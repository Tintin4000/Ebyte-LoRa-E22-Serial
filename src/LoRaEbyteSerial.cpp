/*
 Name:		EBYTE UART 1262 - LoRaEbyteSerial.cpp
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

/*
    This library use various resources to operate correctly, a list of must have resources are listed here under:
    - two hardware serial port, one for the communiation with the Ebyte module and one for debugging, optional.
    - one interrupt capable GPIO pin
    - two output GPIO pin
*/

#include "LoRaEbyteSerial.h"

/*
    Contructor for the class.
*/
LoRa_Ebyte_Serial::LoRa_Ebyte_Serial(HardwareSerial *hws, int8_t m0Pin, int8_t m1Pin, int8_t auxPin, int8_t rxPin, int8_t txPin)
{
    // allow only one instance of the library at the time
    semaphoreHandle = xSemaphoreCreateMutexStatic(&staticMutex);
    if (semaphoreHandle != NULL)
    {
        xSemaphoreTake(this->semaphoreHandle, portMAX_DELAY);
    }

#ifdef LoRa_Ebyte_Serial_Debug
    Serial.begin(SERIAL_BAUD_RATE);
    Serial.println("Debug Started");
#endif

    // assign the selected pin to the class variables
    m0_Pin = m0Pin;
    m1_Pin = m1Pin;
    aux_Pin = auxPin;
    rx_Pin = rxPin;
    tx_Pin = txPin;

    hs = hws;

    // initialize the GPIO mode
    if ((m0_Pin != -1) && (m1_Pin != -1) && (aux_Pin != -1))
    {
        // set the aux pin to input;
        pinMode(aux_Pin, INPUT);

        // set the mode pin to output and default to deep sleep *******
        digitalWrite(m0_Pin, HIGH);
        digitalWrite(m1_Pin, HIGH);
        pinMode(m0_Pin, OUTPUT);
        pinMode(m1_Pin, OUTPUT);
        digitalWrite(m0_Pin, HIGH);
        digitalWrite(m1_Pin, HIGH);
    }
}

/*
    Destructor for the class
*/
LoRa_Ebyte_Serial::~LoRa_Ebyte_Serial()
{
    this->hs = nullptr;
    xSemaphoreGive(this->semaphoreHandle);
}

/*
    Convert binary baud rate to actual baud rate
*/
E22_UartBaudRate LoRa_Ebyte_Serial::ConvertUartBaudRateBin2Dec(uint8_t binBaudRate)
{
    E22_UartBaudRate baudrate = E22_UartBaudRate::UART_BAUD_RATE_NA;
    switch (binBaudRate)
    {
    case 0:
        baudrate = E22_UartBaudRate::UART_BAUD_RATE_1200;
        break;
    case 1:
        baudrate = E22_UartBaudRate::UART_BAUD_RATE_2400;
        break;
    case 2:
        baudrate = E22_UartBaudRate::UART_BAUD_RATE_4800;
        break;
    case 3:
        baudrate = E22_UartBaudRate::UART_BAUD_RATE_9600;
        break;
    case 4:
        baudrate = E22_UartBaudRate::UART_BAUD_RATE_19200;
        break;
    case 5:
        baudrate = E22_UartBaudRate::UART_BAUD_RATE_38400;
        break;
    case 6:
        baudrate = E22_UartBaudRate::UART_BAUD_RATE_57600;
        break;
    case 7:
        baudrate = E22_UartBaudRate::UART_BAUD_RATE_115200;
        break;
    }
    return baudrate;
}

/*
    Convert decimal baud rate to binary baud rate
*/
uint8_t LoRa_Ebyte_Serial::ConvertUartBaudRateDec2Bin(uint32_t decBaudRate)
{
    uint8_t baudrate = 0;
    switch (decBaudRate)
    {
    case E22_UartBaudRate::UART_BAUD_RATE_1200:
        baudrate = E22_UartBaudRate_Binary::UART_BAUD_RATE_1200_BINARY;
        break;
    case E22_UartBaudRate::UART_BAUD_RATE_2400:
        baudrate = E22_UartBaudRate_Binary::UART_BAUD_RATE_2400_BINARY;
        break;
    case E22_UartBaudRate::UART_BAUD_RATE_4800:
        baudrate = E22_UartBaudRate_Binary::UART_BAUD_RATE_4800_BINARY;
        break;
    case E22_UartBaudRate::UART_BAUD_RATE_9600:
        baudrate = E22_UartBaudRate_Binary::UART_BAUD_RATE_9600_BINARY;
        break;
    case E22_UartBaudRate::UART_BAUD_RATE_19200:
        baudrate = E22_UartBaudRate_Binary::UART_BAUD_RATE_19200_BINARY;
        break;
    case E22_UartBaudRate::UART_BAUD_RATE_38400:
        baudrate = E22_UartBaudRate_Binary::UART_BAUD_RATE_38400_BINARY;
        break;
    case E22_UartBaudRate::UART_BAUD_RATE_57600:
        baudrate = E22_UartBaudRate_Binary::UART_BAUD_RATE_57600_BINARY;
        break;
    case E22_UartBaudRate::UART_BAUD_RATE_115200:
        baudrate = E22_UartBaudRate_Binary::UART_BAUD_RATE_115200_BINARY;
        break;
    default:
        baudrate = 0;
        break;
    }
    return baudrate;
}

/*
    Convert binary air data rate to decimal baud rate
*/
E22_AirDataRate LoRa_Ebyte_Serial::ConvertAirDataRateBin2Dec(uint8_t binBaudRate)
{
    E22_AirDataRate baudrate = E22_AirDataRate::AIR_DATA_RATE_NA;
    switch (binBaudRate)
    {
    case E22_AirDataRate_Binary::AIR_DATA_RATE_300_BINARY:
        baudrate = E22_AirDataRate::AIR_DATA_RATE_300;
        break;
    case E22_AirDataRate_Binary::AIR_DATA_RATE_1200_BINARY:
        baudrate = E22_AirDataRate::AIR_DATA_RATE_1200;
        break;
    case E22_AirDataRate_Binary::AIR_DATA_RATE_2400_BINARY:
        baudrate = E22_AirDataRate::AIR_DATA_RATE_2400;
        break;
    case E22_AirDataRate_Binary::AIR_DATA_RATE_4800_BINARY:
        baudrate = E22_AirDataRate::AIR_DATA_RATE_4800;
        break;
    case E22_AirDataRate_Binary::AIR_DATA_RATE_9600_BINARY:
        baudrate = E22_AirDataRate::AIR_DATA_RATE_9600;
        break;
    case E22_AirDataRate_Binary::AIR_DATA_RATE_19200_BINARY:
        baudrate = E22_AirDataRate::AIR_DATA_RATE_19200;
        break;
    case E22_AirDataRate_Binary::AIR_DATA_RATE_38400_BINARY:
        baudrate = E22_AirDataRate::AIR_DATA_RATE_38400;
        break;
    case E22_AirDataRate_Binary::AIR_DATA_RATE_62500_BINARY:
        baudrate = E22_AirDataRate::AIR_DATA_RATE_62500;
        break;
    }
    return baudrate;
}

/*
    Convert decimal air data rate to binary baud rate
*/
uint8_t LoRa_Ebyte_Serial::ConvertAirDataRateDec2Bin(uint32_t decBaudRate)
{
    uint8_t baudrate = 0;
    switch (decBaudRate)
    {
    case E22_AirDataRate::AIR_DATA_RATE_300:
        baudrate = E22_AirDataRate_Binary::AIR_DATA_RATE_300_BINARY;
        break;
    case E22_AirDataRate::AIR_DATA_RATE_1200:
        baudrate = E22_AirDataRate_Binary::AIR_DATA_RATE_1200_BINARY;
        break;
    case E22_AirDataRate::AIR_DATA_RATE_2400:
        baudrate = E22_AirDataRate_Binary::AIR_DATA_RATE_2400_BINARY;
        break;
    case E22_AirDataRate::AIR_DATA_RATE_4800:
        baudrate = E22_AirDataRate_Binary::AIR_DATA_RATE_4800_BINARY;
        break;
    case E22_AirDataRate::AIR_DATA_RATE_9600:
        baudrate = E22_AirDataRate_Binary::AIR_DATA_RATE_9600_BINARY;
        break;
    case E22_AirDataRate::AIR_DATA_RATE_19200:
        baudrate = E22_AirDataRate_Binary::AIR_DATA_RATE_19200_BINARY;
        break;
    case E22_AirDataRate::AIR_DATA_RATE_38400:
        baudrate = E22_AirDataRate_Binary::AIR_DATA_RATE_38400_BINARY;
        break;
    case E22_AirDataRate::AIR_DATA_RATE_62500:
        baudrate = E22_AirDataRate_Binary::AIR_DATA_RATE_62500_BINARY;
        break;
    default:
        baudrate = 0;
        break;
    }
    return baudrate;
}

/*
    Convert binary packet size to decimal
*/
E22_PacketSize LoRa_Ebyte_Serial::ConvertPacketSizeBin2Dec(uint8_t binPacketSize)
{
    E22_PacketSize packetsize = E22_PacketSize::PACKET_SIZE_NA;
    switch (binPacketSize)
    {
    case E22_PacketSize_Binary::PACKET_SIZE_240_BINARY:
        packetsize = E22_PacketSize::PACKET_SIZE_240;
        break;
    case E22_PacketSize_Binary::PACKET_SIZE_128_BINARY:
        packetsize = E22_PacketSize::PACKET_SIZE_128;
        break;
    case E22_PacketSize_Binary::PACKET_SIZE_64_BINARY:
        packetsize = E22_PacketSize::PACKET_SIZE_64;
        break;
    case E22_PacketSize_Binary::PACKET_SIZE_32_BINARY:
        packetsize = E22_PacketSize::PACKET_SIZE_32;
        break;
    }
    return packetsize;
}

/*
    Convert decimal packet size to binary
*/
uint8_t LoRa_Ebyte_Serial::ConvertPacketSizeDec2Bin(uint8_t decPacketSize)
{
    uint8_t binPacketSize = 0;
    switch (decPacketSize)
    {
    case E22_PacketSize::PACKET_SIZE_240:
        binPacketSize = E22_PacketSize_Binary::PACKET_SIZE_240_BINARY;
        break;
    case E22_PacketSize::PACKET_SIZE_128:
        binPacketSize = E22_PacketSize_Binary::PACKET_SIZE_128_BINARY;
        break;
    case E22_PacketSize::PACKET_SIZE_64:
        binPacketSize = E22_PacketSize_Binary::PACKET_SIZE_64_BINARY;
        break;
    case E22_PacketSize::PACKET_SIZE_32:
        binPacketSize = E22_PacketSize_Binary::PACKET_SIZE_32_BINARY;
        break;
    default:
        binPacketSize = 0;
        break;
    }
    return binPacketSize;
}

/*
    Convert binary transmitting power to decimal
*/
E22_TransmissionPower LoRa_Ebyte_Serial::ConvertTransPowerBin2Dec(uint8_t binTransPower)
{
    E22_TransmissionPower transPower = E22_TransmissionPower::POWER_NA;
    switch (binTransPower)
    {
    case E22_TransmissionPower_Binary::POWER_10_BINARY:
        transPower = E22_TransmissionPower::POWER_10;
        break;
    case E22_TransmissionPower_Binary::POWER_13_BINARY:
        transPower = E22_TransmissionPower::POWER_13;
        break;
    case E22_TransmissionPower_Binary::POWER_17_BINARY:
        transPower = E22_TransmissionPower::POWER_17;
        break;
    case E22_TransmissionPower_Binary::POWER_22_BINARY:
        transPower = E22_TransmissionPower::POWER_22;
        break;
    }
    return transPower;
}

/*
    Convert decimal transmission power to binary
*/
uint8_t LoRa_Ebyte_Serial::ConvertTransPowerDec2Bin(uint8_t decTransPower)
{
    uint8_t binTransPower = 0;
    switch (decTransPower)
    {
    case E22_TransmissionPower::POWER_10:
        binTransPower = E22_TransmissionPower_Binary::POWER_10_BINARY;
        break;
    case E22_TransmissionPower::POWER_13:
        binTransPower = E22_TransmissionPower_Binary::POWER_13_BINARY;
        break;
    case E22_TransmissionPower::POWER_17:
        binTransPower = E22_TransmissionPower_Binary::POWER_17_BINARY;
        break;
    case E22_TransmissionPower::POWER_22:
        binTransPower = E22_TransmissionPower_Binary::POWER_22_BINARY;
        break;
    default:
        binTransPower = 0;
        break;
    }
    return binTransPower;
}

/*
    Get method of the initialized variable
*/
bool LoRa_Ebyte_Serial::Initialized()
{
    return this->initialized;
}

/*
    Interrupt handler for the Aux line
*/
void IRAM_ATTR LoRa_Ebyte_Serial::InterruptRoutine()
{
    vTaskNotifyGiveFromISR(this->taskToNotify, NULL);
}

/*
    Check module readyness state.  The checkAux can be turn on or off depending if you need to check if Aux is High or not.
    Default is check for Aux High during a short period and return true if succesful.
    The timeout is set by default for most module activities and should be set to 0 for infinite waiting such as waiting for a received message.
    The task will be suspended during the waiting time.
*/
bool LoRa_Ebyte_Serial::ModuleReady(bool checkAux = true, uint16_t timeout = moduleTimeout)
{
    // check that the AUX line is HIGH
    if (checkAux && digitalRead(aux_Pin))
    {
        // the aux should stay high for at least 2ms after rising to be consider done with the operation
        vTaskDelay(pdMS_TO_TICKS(delaySwitch));
        if (digitalRead(aux_Pin))
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.printf("Aux line is high (%lu)\n", millis());
#endif
            return true;
        }
    }

#ifdef LoRa_Ebyte_Serial_Debug
    Serial.printf("Getting ready for interrupt (%lu)\n", millis());
#endif

    // assign the task handle
    taskToNotify = xTaskGetCurrentTaskHandle();

    // attach the interrupt to the return to level high of the Aux line after changing mode
    attachInterrupt(digitalPinToInterrupt(aux_Pin), std::bind(&LoRa_Ebyte_Serial::InterruptRoutine, this), RISING);

    bool ready = false;
    TimeOut_t timeOut;
    vTaskSetTimeOutState(&timeOut);

    // select the timeout for either finite waiting or indefinite waiting such as end of receiving transmission
    TickType_t ticksToWait;
    if (timeout)
    {
        ticksToWait = timeout;
    }
    else
    {
        ticksToWait = portMAX_DELAY;
    }

    do
    {
        // wait for the interrupt to release the task
#ifdef LoRa_Ebyte_Serial_Debug
        uint32_t token = ulTaskNotifyTake(pdTRUE, ticksToWait);
#else
        ulTaskNotifyTake(pdTRUE, ticksToWait); // portMAX_DELAY
#endif
        // check if an interrupt occured or a timeout
        if (xTaskCheckForTimeOut(&timeOut, &ticksToWait) == pdTRUE)
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.printf("Ebyte module timeout (%lu)\n", millis());
#endif
            break;
        }

        // according to user manual p.14 the aux should stay high for at least 2ms after rising to be consider done with the operation
        vTaskDelay(pdMS_TO_TICKS(delaySwitch));

        if (digitalRead(aux_Pin))
        {
            ready = true;
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.printf("Ebyte module ready, token = %d (%lu)\n", token, millis());
#endif
        }
        else
        {
            ready = false;
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.printf("Ebyte module not ready after interrupt, token = %d (%lu)\n", token, millis());
#endif
        }

    } while (!ready);

    // detach the interrupt
    detachInterrupt(digitalPinToInterrupt(aux_Pin));

    if (ready)
        return true;
    else
        return false;
}

/*
    Get the current module mode.
*/
E22T_Mode_t LoRa_Ebyte_Serial::GetModuleMode()
{
    return this->lastModuleMode;
}

/*
    Change the mode of the Ebyte device.
    Return false is module is not ready.
*/
bool LoRa_Ebyte_Serial::SetModuleMode(E22T_Mode_t mode)
{
// only switch if the mode is different then current
#ifdef LoRa_Ebyte_Serial_Debug
    Serial.printf("Module previous mode was %d (%lu)\n", lastModuleMode, millis());
#endif
    if (mode != lastModuleMode)
    {
        // check that the module is ready to switch mode
        if (!ModuleReady(true))
            return false;
        switch (mode)
        {
        case Normal:
            digitalWrite(m0_Pin, LOW);
            digitalWrite(m1_Pin, LOW);
            if (initialized)
                hs->updateBaudRate(GetUartBaudRate());
            break;
        case WorkOnRadio:
            digitalWrite(m0_Pin, HIGH);
            digitalWrite(m1_Pin, LOW);
            break;
        case Configuration:
            digitalWrite(m0_Pin, LOW);
            digitalWrite(m1_Pin, HIGH);
            hs->updateBaudRate(E22_UartBaudRate::UART_BAUD_RATE_9600);
            break;
        case DeepSleep:
            digitalWrite(m0_Pin, HIGH);
            digitalWrite(m1_Pin, HIGH);
            break;
        }

        // from DeepSleep or from Configuration to Normal, wait for trigger.
        if ((lastModuleMode == E22T_Mode_t::DeepSleep) ||
            (lastModuleMode == E22T_Mode_t::Configuration && (mode == E22T_Mode_t::Normal)))
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.printf("Wait for Aux raise (%lu)\n", millis());
#endif
            ModuleReady(false);
        }
        // from Normal to Configuration, wait for 75ms extra for the module to respond after the mode change
        if (lastModuleMode == E22T_Mode_t::Normal && (mode == E22T_Mode_t::Configuration))
        {
            vTaskDelay(pdMS_TO_TICKS(75));
        }
    }
#ifdef LoRa_Ebyte_Serial_Debug
    Serial.printf("Module ready in mode %d (%lu)\n", mode, millis());
#endif
    lastModuleMode = mode;

    return true;
}

/*
    Get the product information
*/
bool LoRa_Ebyte_Serial::GetAllParameters()
{
    if (!initialized)
    {
        // set the module to config mode for the first time without waiting for the readyness Aux line
        if (!SetModuleMode(E22T_Mode_t::Configuration))
            return false;

#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Used uart baud rate is %d\n", hs->baudRate());
#endif
        // returning the product information using the command format C1+Starting Address+length
        hs->write(E22_Command_t::Read_Register);
        hs->write(E22_Register_Addr::ProductInfo);
        hs->write(E22_Register_Length::ProductInfo_length);

        // wait for triggered response
        if (ModuleReady(false))
        {
            // get the response, formated as C1+Starting Address+length+parameters
            uint8_t response;
            bool correct = true;
            // command
            hs->readBytes(&response, 1);
            if (response != E22_Command_t::Response)
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Response is not as expected %x\n", response);
#endif
                correct = false;
            }
            hs->readBytes(&response, 1);
            if (response != E22_Register_Addr::ProductInfo)
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Address is not as expected %x\n", response);
#endif
                correct = false;
            }
            hs->readBytes(&response, 1);
            if (response != E22_Register_Length::ProductInfo_length)
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Length is not as expected %x\n", response);
#endif
                correct = false;
            }
            if (!correct)
            {
                return false;
            }
            // parameters
            hs->readBytes((uint8_t *)&this->productSerialInfo, (uint8_t)sizeof(this->productSerialInfo));
        }
        else
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.println("Module not ready");
#endif
            return false;
        }

#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Product Name = %#x\n", (productSerialInfo.Name_H << 8) | productSerialInfo.Name_L);
        Serial.printf("Product Version = %d\n", productSerialInfo.Version);
        Serial.printf("Product Max Power = %d\n", productSerialInfo.MaxPower);
        Serial.printf("Product Freq = %d\n", productSerialInfo.Freq);
        Serial.printf("Product Type = %d\n", productSerialInfo.Type);
#endif

        // returning the product registry using the command format C1+Starting Address+length
        hs->write(E22_Command_t::Read_Register);
        hs->write(E22_Register_Addr::Address);
        hs->write(E22_Register_Length::Registry_length);

        // wait for triggered response
        if (ModuleReady(false))
        {
            // get the response, formated as C1+Starting Address+length+parameters
            uint8_t response;
            bool correct = true;
            // command
            hs->readBytes(&response, 1);
            if (response != E22_Command_t::Response)
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Response is not as expected %x\n", response);
#endif
                correct = false;
            }
            hs->readBytes(&response, 1);
            if (response != E22_Register_Addr::Address)
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Address is not as expected %x\n", response);
#endif
                correct = false;
            }
            hs->readBytes(&response, 1);
            if (response != E22_Register_Length::Registry_length)
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Length is not as expected %x\n", response);
#endif
                correct = false;
            }
            if (!correct)
            {
                return false;
            }
            // parameters
            hs->readBytes((uint8_t *)&registrySerialRead, (uint8_t)sizeof(registrySerialRead));
        }
        else
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.println("Module not ready");
#endif
            return false;
        }

        // copy the registry values to the registry stack
        this->registrySerialTemp = registrySerialRead;
        this->registrySerialPerm = registrySerialRead;

#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Address = %#x\n", (registrySerialRead.Add_H << 8) | registrySerialRead.Add_L);
        Serial.printf("NetID = %d\n", registrySerialRead.NetID);
        Serial.printf("Registry 0 = %#x\n", registrySerialRead.Reg0);
        Serial.printf("Registry 1 = %#x\n", registrySerialRead.Reg1);
        Serial.printf("Registry 2 = %#x\n", registrySerialRead.Reg2);
        Serial.printf("Registry 3 = %#x\n", registrySerialRead.Reg3);
#endif
    }
    return true;
}

/*
    Select the device communication mode.  Default to UART.
*/
void LoRa_Ebyte_Serial::SelectDeviceComType(E22_Device_ComType selectDeviceComType)
{
    // select the comunication mode either local or remote
    deviceComType = selectDeviceComType;
    // assign the current registry to the corresponding mode
    if (deviceComType == E22_Device_ComType::UART)
    {
        currentReadRegistry = &registrySerialRead;
    }
    if (deviceComType == E22_Device_ComType::Wireless)
    {
        currentReadRegistry = &registryWirelessRead;
    }

    // select the storage registry
    SelectStorage(selectedRegister);
}

/*
    Select the registry to store the update.  Default to permanent.
*/
void LoRa_Ebyte_Serial::SelectStorage(E22_Device_Storage selectStorage)
{
    selectedRegister = selectStorage;
    if (deviceComType == E22_Device_ComType::UART)
    {
        if (selectStorage == E22_Device_Storage::Permanent)
        {
            currentStorageRegistry = &registrySerialPerm;
        }
        if (selectStorage == E22_Device_Storage::Temporary)
        {
            currentStorageRegistry = &registrySerialTemp;
        }
    }
    if (this->deviceComType == E22_Device_ComType::Wireless)
    {
        if (selectStorage == E22_Device_Storage::Permanent)
        {
            currentStorageRegistry = &registryWirelessPerm;
        }
        if (selectStorage == E22_Device_Storage::Temporary)
        {
            currentStorageRegistry = &registryWirelessTemp;
        }
    }
}

/*
    Block the module during the transmission.
    The waiting time is related to the size of the payload and the transmission speed.
*/
void LoRa_Ebyte_Serial::WaitForTransmission(size_t msglen)
{
    uint16_t waitDuration = msglen;
    uint32_t speed = GetAirDataRate();

    if (speed < 19200)
    {
        if (speed == 9600)
            waitDuration += 65 + msglen;
        if (speed == 2400)
            waitDuration += 215 + (5 * msglen);
        if (speed == 1200)
            waitDuration += 335 + (10 * msglen);
        if (speed == 300)
            waitDuration += 995 + (33 * msglen);
    }
    else
        waitDuration = +200;

#ifdef LoRa_Ebyte_Serial_Debug
    Serial.printf("The waiting time is %d\n", waitDuration);
#endif
    vTaskDelay(pdMS_TO_TICKS(waitDuration));
}

/*
 */
bool LoRa_Ebyte_Serial::GetRSSI(uint8_t &ambiantRSSI, uint8_t &lastRSSI)
{
    if (initialized)
    {
        // check that the RSSI mode is activated
        if (!GetRssiNoiseMode())
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.println("RSSI noise function not activated");
#endif
            return false;
        }

        // returning the current ambient channel noise using the command format C0+C1+C2+C3+Starting Address+length
        hs->write(0xC0);
        hs->write(0xC1);
        hs->write(0xC2);
        hs->write(0xC3);
        hs->write(E22_Register_Addr::Address);
        hs->write(E22_Register_Length::Address_length);

        // wait for triggered response
        if (ModuleReady(false))
        {
            // get the response, formated as C1+Starting Address+length+parameters
            uint8_t response;
            bool correct = true;
            // command
            hs->readBytes(&response, 1);
            if (response != E22_Command_t::Response)
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Response is not as expected %x\n", response);
#endif
                correct = false;
            }
            hs->readBytes(&response, 1);
            if (response != E22_Register_Addr::Address)
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Address is not as expected %x\n", response);
#endif
                correct = false;
            }
            hs->readBytes(&response, 1);
            if (response != E22_Register_Length::Address_length)
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Length is not as expected %x\n", response);
#endif
                correct = false;
            }
            if (!correct)
            {
                return false;
            }

            // parameters
            byte rssiReg[2];
            hs->readBytes((uint8_t *)&rssiReg, (uint8_t)sizeof(rssiReg));
            ambiantRSSI = rssiReg[0];
            lastRSSI = rssiReg[1];
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.printf("Ambient RSSI = %d\n", ambiantRSSI);
            Serial.printf("Last transmittion RSSI = %d\n", lastRSSI);
#endif
        }
        // }
        return true;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return false;
}

/*
    Get the Ebyte module name
*/
uint16_t LoRa_Ebyte_Serial::GetModuleName()
{
    return ((productSerialInfo.Name_H << 8) | productSerialInfo.Name_L);
}

/*
    Get the Ebyte module version
*/
uint8_t LoRa_Ebyte_Serial::GetModuleVersion()
{
    return productSerialInfo.Version;
}

/*
    Get the Ebyte module max power
*/
uint8_t LoRa_Ebyte_Serial::GetModuleMaxPower()
{
    return productSerialInfo.MaxPower;
}

/*
    Get the module address.
    Return a word containing the module address.  0 is not initialized.
*/
uint16_t LoRa_Ebyte_Serial::GetModuleAddress()
{
    uint16_t address = 0;

    if (initialized)
    {
        address = currentReadRegistry->Add_H;
        address = ((currentReadRegistry->Add_H << 8) | (currentReadRegistry->Add_L));

#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("The module address is currently %d\n", address);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return address;
}

/*
    Set the module address.
*/
void LoRa_Ebyte_Serial::SetModuleAddress(uint16_t moduleaddr)
{
    uint8_t addrLow = 0, addrHigh = 0;

    addrHigh = (moduleaddr & 0XFF00) >> 8;
    addrLow = (moduleaddr & 0X00FF);

    currentStorageRegistry->Add_H = addrHigh;
    currentStorageRegistry->Add_L = addrLow;
}

/*
    Get the Network ID.
*/
uint8_t LoRa_Ebyte_Serial::GetNetID()
{
    uint8_t netID = 0;
    if (this->initialized)
    {
        netID = currentReadRegistry->NetID;
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Net ID is currently %d\n", netID);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return netID;
}

/*
    Set the Network ID.
 */
void LoRa_Ebyte_Serial::SetNetID(uint8_t netID)
{
    currentStorageRegistry->NetID = netID;
}

/*
    Get UART baud rate.  The default baud rate is 9600bps.
*/
E22_UartBaudRate LoRa_Ebyte_Serial::GetUartBaudRate()
{
    E22_UartBaudRate uartBaudrate = E22_UartBaudRate::UART_BAUD_RATE_NA;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg0;
        regbyte &= 0b11100000;
        regbyte >>= 5;

        uartBaudrate = ConvertUartBaudRateBin2Dec(regbyte);
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Uart baud rate is currently %dbps\n", uartBaudrate);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return uartBaudrate;
}

/*
    Set Uart baud rate. This function will commit automatically to be able to control the communication between the controller and the device.
*/
void LoRa_Ebyte_Serial::SetUartBaudRate(E22_UartBaudRate baudrate)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg0;
        regbyte &= 0b00011111;
        regbyte |= (ConvertUartBaudRateDec2Bin(baudrate) << 5);
        currentStorageRegistry->Reg0 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Get the air data transmission rate.  Default is set to 2400 bytes per sec.
 */
E22_AirDataRate LoRa_Ebyte_Serial::GetAirDataRate()
{
    E22_AirDataRate airdatarate = E22_AirDataRate::AIR_DATA_RATE_NA;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg0;
        regbyte &= 0b00000111;
        airdatarate = ConvertAirDataRateBin2Dec(regbyte);

#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Air data Baud Rate is set to %d\n", airdatarate);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return airdatarate;
}

/*
    Set the air date transmission rate.
 */
void LoRa_Ebyte_Serial::SetAirDataRate(E22_AirDataRate airdatarate)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg0;
        regbyte &= 0b11111000;
        regbyte |= ConvertAirDataRateDec2Bin(airdatarate);
        currentStorageRegistry->Reg0 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Get transmission packet size.
 */
E22_PacketSize LoRa_Ebyte_Serial::GetPacketSize()
{
    E22_PacketSize packetSize = E22_PacketSize::PACKET_SIZE_NA;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg1;
        regbyte &= 0b11000000;
        regbyte >>= 6;
        packetSize = ConvertPacketSizeBin2Dec(regbyte);

#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Packet size is set to %d\n", packetSize);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return packetSize;
}

/*
    Set transmission packet size.
 */
void LoRa_Ebyte_Serial::SetPacketSize(E22_PacketSize packetSize)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg1;
        regbyte &= 0b00111111;
        regbyte |= (ConvertPacketSizeDec2Bin(packetSize) << 6);
        currentStorageRegistry->Reg1 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Get Rssi Noise Enable bit.
 */
bool LoRa_Ebyte_Serial::GetRssiNoiseMode()
{
    bool rssiNoiseEnable = false;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg1;
        regbyte &= 0b00100000;
        regbyte >>= 5;
        rssiNoiseEnable = regbyte;
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Rssi Noise Enable bit is set to %d\n", rssiNoiseEnable);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return rssiNoiseEnable;
}

/*
    Set Rssi Noise Enable bit.
 */
void LoRa_Ebyte_Serial::SetRssiNoiseMode(bool rssiNoiseEnabled)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg1;
        regbyte &= 0b11011111;
        regbyte |= (rssiNoiseEnabled << 5);
        currentStorageRegistry->Reg1 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Get the transmission power in dbm.
 */
E22_TransmissionPower LoRa_Ebyte_Serial::GetTransmittingPower()
{
    E22_TransmissionPower transPower = E22_TransmissionPower::POWER_NA;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg1;
        regbyte &= 0b00000011;
        transPower = ConvertTransPowerBin2Dec(regbyte);
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Transmitting power is %ddbm\n", transPower);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return transPower;
}

/*
    Set the transmission power in dbm.
*/
void LoRa_Ebyte_Serial::SetTransmittingPower(E22_TransmissionPower transPower)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg1;
        regbyte &= 0b11111100;
        regbyte |= ConvertTransPowerDec2Bin(transPower);
        currentStorageRegistry->Reg1 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Get the selected communication channel.
 */
uint8_t LoRa_Ebyte_Serial::GetChannel()
{
    uint8_t channel = 0;
    if (initialized)
    {
        channel = currentReadRegistry->Reg2;
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("The channel is currently %d\n", channel);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return channel;
}

/*
    Set the communication channel.  Channen must be between 0 and 80.
 */
void LoRa_Ebyte_Serial::SetChannel(uint8_t channel)
{
    if (channel > 80)
        channel = 80;

    currentStorageRegistry->Reg2 = channel;
}

/*
    Return the RSSI enable bit state.
    When enabled, after receiving wireless data an RSSI strength byte will be added.
 */
bool LoRa_Ebyte_Serial::GetRssiMode()
{
    bool rssiEnable = false;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg3;
        regbyte &= 0b10000000;
        regbyte >>= 7;
        rssiEnable = regbyte;
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Rssi Enable bit is set to %d\n", rssiEnable);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return rssiEnable;
}

/*
    Set the RSSI enable mode.
    When enabled, after receiving wireless data a RSSI strength byte will be added.
*/
void LoRa_Ebyte_Serial::SetRssiMode(bool enabled)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg3;
        regbyte &= 0b01111111;
        regbyte |= (enabled << 7);
        currentStorageRegistry->Reg3 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Return the fixed point transmission enable bit state.
    When enabled, the module recognizes the first three bytes of the serial data as the module address (high and low), channel and interprets it as the wireless transmitting targets.
*/
bool LoRa_Ebyte_Serial::GetFixPointTransmissionMode()
{
    bool fpt = false;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg3;
        regbyte &= 0b01000000;
        regbyte >>= 6;
        fpt = regbyte;
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Fix Point Transmission is set to %d\n", fpt);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return fpt;
}

/*
    Set the fixed point transmission enable bit state.
    When enabled, the module recognizes the first three bytes of the serial data as the module address (high and low), channel and interprets it as the wireless transmitting targets.
*/
void LoRa_Ebyte_Serial::SetFixPointTransmissionMode(bool enable)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg3;
        regbyte &= 0b10111111;
        regbyte |= (enable << 6);
        currentStorageRegistry->Reg3 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Return the enable reply bit state.
    When enabled, if not the destination address, the message is relayed.
*/
bool LoRa_Ebyte_Serial::GetEnableReply()
{
    bool er = false;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg3;
        regbyte &= 0b00100000;
        regbyte >>= 5;
        er = regbyte;
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Enable reply is set to %d\n", er);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return er;
}

/*
    Set the enable reply bit state.
    When enabled, if not the destination address, the message is relayed.
*/
void LoRa_Ebyte_Serial::SetEnableReply(bool enable)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg3;
        regbyte &= 0b11011111;
        regbyte |= (enable << 5);
        currentStorageRegistry->Reg3 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Return the monitor before transmit function bit.
    When enabled, wireless data will be monitored before transmission, which can avoid interference , but may cause data delay.
*/
bool LoRa_Ebyte_Serial::GetLookBeforeTransmit()
{
    bool lbt = false;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg3;
        regbyte &= 0b00010000;
        regbyte >>= 4;
        lbt = regbyte;
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Enable reply is set to %d\n", lbt);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return lbt;
}

/*
    Set the monitor before transmit function bit.
    When enabled, wireless data will be monitored before transmission, which can avoid interference , but may cause data delay.
*/
void LoRa_Ebyte_Serial::SetLookBeforeTransmit(bool enable)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg3;
        regbyte &= 0b11101111;
        regbyte |= (enable << 4);
        currentStorageRegistry->Reg3 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Return the wake on radio (WOR) mode.  Only valid when module mode is set to WOR.
    True/1 when WOR transmitter mode, false/0 when WOR receiver mode.  The monitoring period is define by the WOR cycle.
    The WOR cycle must be the same on the transmitter and receiver module.
*/
bool LoRa_Ebyte_Serial::GetWakeOnRadioMode()
{
    bool wormode = false;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg3;
        regbyte &= 0b00001000;
        regbyte >>= 3;
        wormode = regbyte;
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("Enable reply is set to %d\n", wormode);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return wormode;
}

/*
    Set the wake on radio (WOR) mode.  Only valid when module mode is set to WOR.
    True/1 when WOR transmitter mode, false/0 when WOR receiver mode.  The monitoring period is define by the WOR cycle.
    The WOR cycle must be the same on the transmitter and receiver module.
*/
void LoRa_Ebyte_Serial::SetWakeOnRadioMode(bool worMode)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg3;
        regbyte &= 0b11110111;
        regbyte |= (worMode << 3);
        currentStorageRegistry->Reg3 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Return the wake on radio (WOR) cycle.  Only valid when module mode is set to WOR.
    The WOR cycle must be the same on the transmitter and receiver module.
*/
E22_WorCycle LoRa_Ebyte_Serial::GetWakeOnRadioCycle()
{
    E22_WorCycle worCycle = E22_WorCycle::WOR_NA;

    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentReadRegistry->Reg3;
        regbyte &= 0b00000111;
        worCycle = ConvertWorCycleBin2Dec(regbyte);
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("WOR cycle is %dms\n", worCycle);
#endif
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return worCycle;
}

/*
    Set the wake on radio (WOR) cycle.  Only valid when module mode is set to WOR.
    The WOR cycle must be the same on the transmitter and receiver module.
*/
void LoRa_Ebyte_Serial::SetWakeOnRadioCycle(E22_WorCycle worCycle)
{
    if (initialized)
    {
        uint8_t regbyte = 0;
        regbyte = currentStorageRegistry->Reg3;
        regbyte &= 0b11111000;
        regbyte |= this->ConvertWorCycleDec2Bin(worCycle);
        currentStorageRegistry->Reg3 = regbyte;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
}

/*
    Used for user encryption to avoid interceptiong airborne wireless data by similar module.
*/
void LoRa_Ebyte_Serial::SetEncryption(uint16_t encryption)
{
    uint8_t cryptLow = 0, cryptHigh = 0;

    cryptHigh = (encryption & 0XFF00) >> 8;
    cryptLow = (encryption & 0X00FF);

    cryptoData.Crupto_H = cryptHigh;
    cryptoData.Crypto_L = cryptLow;
    cryptoOn = true;
}

/*
    Send messages over lora in broadcast transmission. The maximum message length is 240 bytes.
    Return true if transmission appeared to be successful, false otherwise.
*/
bool LoRa_Ebyte_Serial::SendMessage(const char *msgbuffer)
{
    if (initialized)
    {
        size_t size = strlen(msgbuffer);
        // no more than 240 bytes can be received
        if (size > 240)
            return false;

        // check the sub packet size
        if (GetPacketSize() != 240)
        {
            SetPacketSize(E22_PacketSize::PACKET_SIZE_240);
            CommitChanges();
        }

        SetModuleMode(E22T_Mode_t::Normal);
        size_t len = hs->write((uint8_t *)msgbuffer, size);
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("The message is :%s, length = %d, written bytes = %d\n", msgbuffer, size, len);
#endif
        if (len == size)
            WaitForTransmission(len);
        return true;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return false;
}

/*
    Send messages over lora in fixed transmission.  The maximum message length is 237 bytes.
    Return true if transmission appeared to be successful, false otherwise.
*/
bool LoRa_Ebyte_Serial::SendMessage(FixedTransmissionMessage *fmsgbuffer)
{
    if (initialized)
    {
        size_t size = strlen((char *)fmsgbuffer->Message);
        // no more than 237 bytes can be received
        if (size > 237)
            return false;

        // check the sub packet size
        if (GetPacketSize() != 240)
        {
            SetPacketSize(E22_PacketSize::PACKET_SIZE_240);
            CommitChanges();
        }

        SetModuleMode(E22T_Mode_t::Normal);
        size_t len = 0;
        len += hs->write((uint8_t *)fmsgbuffer, 3);
        len += hs->write((uint8_t *)fmsgbuffer->Message, size);
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("The address is %d %d, the channel is %d, the message is :%s, length = %d, written bytes = %d\n", fmsgbuffer->Add_H, fmsgbuffer->Add_L, fmsgbuffer->Channel, fmsgbuffer->Message, size, len);
#endif
        if (len == size + 3)
            WaitForTransmission(len);
        return true;
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return false;
}

/*
    Get the message received over LoRa transmission as a character array.
*/
bool LoRa_Ebyte_Serial::ReceiveMessage(char *msgbuffer, size_t bufsize, size_t &msglenght, uint8_t &rssi, bool wait = false)
{
    if (msgbuffer == nullptr)
        return false;

    if (initialized)
    {
        // set the module to normal, receiving mode
        bool ready = SetModuleMode(E22T_Mode_t::Normal);
        if (!ready)
            return false;

        // check that the module is in ready state and wait for reception or go through
        if (wait)
        {
            ModuleReady(false, 0);
            vTaskDelay(50);
        }
        else
            ModuleReady(true);

        // receive the serial transmission
        int charbuffer = hs->available();
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.printf("There is %d char in the uart buffer\n", charbuffer);
#endif
        if (charbuffer)
        {
            if (charbuffer > bufsize)
                charbuffer = bufsize;
            msglenght = hs->read(msgbuffer, charbuffer);

            // check for RSSI last byte if enabled
            if (GetRssiMode())
            {
                rssi = msgbuffer[msglenght - 1];
                // remove the rssi value from the message
                msglenght--;
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("The message is :%s, buffer = %d, length = %d, rssi = %d\n", msgbuffer, bufsize, msglenght, rssi);
#endif
            }
            else
            {
                rssi = 0;
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("The message is :%s, buffer = %d, length = %d\n", msgbuffer, bufsize, msglenght);
#endif
            }
            return true;
        }
        else
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.printf("No message received\n");
#endif
        }
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return false;
}

/*
    Compare two registry structure and return true if identical.
    Note: if the structure change this function will need to be adjusted.
*/
bool LoRa_Ebyte_Serial::CompareRegistry(RegistryStructure *regStruct1, RegistryStructure *regStruct2)
{
    if (regStruct1->Add_H != regStruct2->Add_H)
        return false;
    if (regStruct1->Add_L != regStruct2->Add_L)
        return false;
    if (regStruct1->NetID != regStruct2->NetID)
        return false;
    if (regStruct1->Reg0 != regStruct2->Reg0)
        return false;
    if (regStruct1->Reg1 != regStruct2->Reg1)
        return false;
    if (regStruct1->Reg2 != regStruct2->Reg2)
        return false;
    if (regStruct1->Reg3 != regStruct2->Reg3)
        return false;
    return true;
}

/*
    Store the permanent or temporary registry structure to the module registry.
 */
bool LoRa_Ebyte_Serial::CommitChanges()
{
    if (initialized)
    {
        if (SetModuleMode(E22T_Mode_t::Configuration))
        {
            uint8_t dataLength;

            // check the aux level and wait for next trigger if low
            ModuleReady(true);

            // write the registry status to the module permanently or temporarily using the command format C0 or C2+Starting Address+length
            if (selectedRegister == E22_Device_Storage::Permanent)
            {
                hs->write(E22_Command_t::Set_Permanent_Register);
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Commit to permanent registry\n");
#endif
            }
            else
            {
                hs->write(E22_Command_t::Set_Temporary_Register);
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Commit to temporary registry\n");
#endif
            }
            hs->write(E22_Register_Addr::Address);

            if (!cryptoOn)
            {
                dataLength = sizeof(RegistryStructure);
                hs->write(dataLength);
            }
            else
            {
                dataLength = sizeof(RegistryStructure) + sizeof(CryptoStructure);
                hs->write(dataLength);
            }

            hs->write((uint8_t *)currentStorageRegistry, sizeof(RegistryStructure));

            if (cryptoOn)
            {
                hs->write((uint8_t *)&cryptoData, (uint8_t)sizeof(CryptoStructure));
            }

            // wait for triggered response
            if (ModuleReady(false))
            {
                // get the response, formated as C1+Starting Address+length+parameters
                uint8_t response;
                bool correct = true;
                // command
                hs->readBytes(&response, 1);
                if (response != E22_Command_t::Response)
                {
#ifdef LoRa_Ebyte_Serial_Debug
                    Serial.printf("Response is not as expected %x\n", response);
#endif
                    correct = false;
                }
                hs->readBytes(&response, 1);
                if (response != E22_Register_Addr::Address)
                {
#ifdef LoRa_Ebyte_Serial_Debug
                    Serial.printf("Address is not as expected %x\n", response);
#endif
                    correct = false;
                }
                hs->readBytes(&response, 1);
                if (response != dataLength)
                {
#ifdef LoRa_Ebyte_Serial_Debug
                    Serial.printf("Length is not as expected %x\n", response);
#endif
                    correct = false;
                }

                // return if any incorrect module feedback
                if (!correct)
                {
                    return false;
                }

                // parameters
                hs->readBytes((uint8_t *)currentReadRegistry, (uint8_t)sizeof(RegistryStructure));

                // read the crypto return if enabled
                if (cryptoOn)
                {
                    hs->readBytes((uint8_t *)&cryptoData, (uint8_t)sizeof(CryptoStructure));
                    // reset the flag for the crypto registry
                    cryptoOn = false;
#ifdef LoRa_Ebyte_Serial_Debug
                    Serial.printf("Crypto H & L = %#x,%#x\n", cryptoData.Crupto_H, cryptoData.Crypto_L);
#endif
                }
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.printf("Address = %#x\n", (currentStorageRegistry->Add_H << 8) | currentStorageRegistry->Add_L);
                Serial.printf("NetID = %d\n", currentStorageRegistry->NetID);
                Serial.printf("Registry 0 = %#x\n", currentStorageRegistry->Reg0);
                Serial.printf("Registry 1 = %#x\n", currentStorageRegistry->Reg1);
                Serial.printf("Registry 2 = %#x\n", currentStorageRegistry->Reg2);
                Serial.printf("Registry 3 = %#x\n", currentStorageRegistry->Reg3);
#endif
                return CompareRegistry(currentReadRegistry, currentStorageRegistry);
            }
            else
            {
#ifdef LoRa_Ebyte_Serial_Debug
                Serial.println("Module not ready after send command");
#endif
            }
        }
        else
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.println("Module not ready after mode change");
#endif
        }
    }
    else
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Successful initialization is required");
#endif
    }
    return false;
}

/*
    Scan the UART to define its current baud rate configuration
*/
uint32_t LoRa_Ebyte_Serial::ScanUartSpeed()
{
    // timeout for the baud rate test
    uint32_t currentTime = 0;

    if (!SetModuleMode(E22T_Mode_t::Configuration))
        return 0;

    // loop through all supporteed uart baud rate
    uint8_t baudrateCount = 0;
    do
    {
        currentTime = millis();
        // change the baud rate to scan the correct one
        hs->updateBaudRate(ConvertUartBaudRateBin2Dec(baudrateCount));

        // returning the product information using the command format C1+Starting Address+length
        hs->write(E22_Command_t::Read_Register);
        hs->write(E22_Register_Addr::ProductInfo);
        hs->write(E22_Register_Length::ProductInfo_length);

        // wait for triggered response
        uint8_t found = false;
        if (ModuleReady(false))
        {
            do
            {
                while (hs->available())
                {
                    // get the response code
                    uint8_t response = 0;

                    hs->readBytes(&response, 1);
#ifdef LoRa_Ebyte_Serial_Debug
                    Serial.printf("Character received is %d\n", response);
#endif
                    if (response == E22_Command_t::Response)
                        found = true;
                }
            } while (((millis() - currentTime) < 1000) && !found); // timeout is 1000ms
        }
        if (found)
            break;
        baudrateCount++;
    } while (baudrateCount < 8);

    return ConvertUartBaudRateBin2Dec(baudrateCount);
}

/*
    Reset the local module to its factory default value.
*/
bool LoRa_Ebyte_Serial::ResetToFactoryDefault()
{
    // select the local permanent registry
    SelectDeviceComType(E22_Device_ComType::UART);
    SelectStorage(E22_Device_Storage::Permanent);

    // define the default value structure
    currentStorageRegistry->Add_H = 0;
    currentStorageRegistry->Add_L = 0;
    currentStorageRegistry->NetID = 0;
    currentStorageRegistry->Reg0 = 98; // x62
    currentStorageRegistry->Reg1 = 00;
    currentStorageRegistry->Reg2 = 18; // x12
    currentStorageRegistry->Reg3 = 3;

    // reset the encryption
    SetEncryption(0);

    // commit the changes
    return CommitChanges();
}

/*
    Initialize the E22 module
*/
bool LoRa_Ebyte_Serial::Init()
{
    if (!initialized)
    {
#ifdef LoRa_Ebyte_Serial_Debug
        Serial.println("Start initialization");
        hs->setDebugOutput(true);
#endif

        if ((m0_Pin == -1) || (m1_Pin == -1) || (aux_Pin == -1))
            return false;

        uint32_t CfgBaudRate = 0;

        // initialize the serial port
        if (!hs)
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.println("Initialization failed due to no serial port provided");
#endif
            return false;
        }
        else
        {
            CfgBaudRate = E22_UartBaudRate::UART_BAUD_RATE_9600;

            if ((rx_Pin == -1) || (tx_Pin == -1))
            {
                hs->begin(CfgBaudRate);
            }
            else
            {
                hs->begin(CfgBaudRate, SERIAL_8N1, rx_Pin, tx_Pin);
            }
        }

        // select the device and storage
        SelectDeviceComType(E22_Device_ComType::UART);
        SelectStorage(E22_Device_Storage::Permanent);

        // get the product information to verify that the module initialized correctly
        if (!GetAllParameters())
        {
#ifdef LoRa_Ebyte_Serial_Debug
            Serial.printf("Initialization failed, no communication with device at %dbps\n", CfgBaudRate);
#endif
            return false;
        }

        // set the initialized flag to true
        initialized = true;
    }

#ifdef LoRa_Ebyte_Serial_Debug
    Serial.println("Initialization succeeded");
#endif
    return true;
}