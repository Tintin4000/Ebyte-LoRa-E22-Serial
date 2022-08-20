#include <Arduino.h>
#include <LoRaEbyteSerial.h>
#include <unity.h>

LoRa_Ebyte_Serial lora(&Serial1, 27, 33, 34);

FixedTransmissionMessage ftMessage;

// char msg[] = "a";
// char msg[] = "abcdefghi";
// char msg[] = "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz";
// char msg[] = "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz";
char msg[] = "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabc";
// char msg[] = "abcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyzabcdefghijklmnopqrstuvwxyz";
void test_Initializing(void)
{
  TEST_ASSERT_EQUAL_UINT8(1, lora.Init());
}

void test_ScanUartSpeed(void)
{
  TEST_ASSERT_GREATER_THAN_UINT8(0, lora.ScanUartSpeed());
}

void test_ResetToFactoryDefault(void)
{
  TEST_ASSERT_EQUAL_UINT8(1, lora.ResetToFactoryDefault());
}

void test_SetUartBaudRate(void)
{
  lora.SetUartBaudRate(E22_UartBaudRate::UART_BAUD_RATE_19200);
  TEST_ASSERT_EQUAL_UINT8(1, lora.CommitChanges()); // Reg0 from 62h to 82h
}

void test_GetUartBaudRate(void)
{
  TEST_ASSERT_EQUAL_UINT32(19200, lora.GetUartBaudRate());
}

void test_SetModuleAddress(void)
{
  lora.SetModuleAddress(16448);
  lora.CommitChanges();
  TEST_ASSERT_EQUAL_UINT8(16448, lora.GetModuleAddress()); // module address = 40h and 40h
}

void test_SetChannelTo9(void)
{
  lora.SetChannel(9);
  lora.CommitChanges();
  TEST_ASSERT_EQUAL_UINT8(9, lora.GetChannel()); // Reg2 to 50h
}

void test_SetAirDataRate(void)
{
  lora.SetAirDataRate(E22_AirDataRate::AIR_DATA_RATE_2400);
  lora.CommitChanges();
  TEST_ASSERT_EQUAL_UINT8(E22_AirDataRate::AIR_DATA_RATE_2400, lora.GetAirDataRate()); // Reg1 from 62h to 64h
}

void test_SetRSSiEnable(void)
{
  lora.SetRssiNoiseMode(true);
  lora.SetRssiMode(true);
  lora.CommitChanges();
  TEST_ASSERT_EQUAL_UINT8(1, lora.GetRssiNoiseMode());
}

void test_SetFixedTransMode(void)
{
  lora.SetFixPointTransmissionMode(true);
  lora.CommitChanges();
  TEST_ASSERT_EQUAL_UINT8(1, lora.GetFixPointTransmissionMode());
}

void test_GetRRSI(void)
{
  uint8_t ambiantRSSI = 0, lastRSSI = 0;
  TEST_ASSERT_EQUAL_UINT8(1, lora.GetRSSI(ambiantRSSI, lastRSSI));
}

void test_SetTransPower10(void)
{
  lora.SetTransmittingPower(E22_TransmissionPower::POWER_10);
  lora.CommitChanges();
  TEST_ASSERT_EQUAL_UINT8(E22_TransmissionPower::POWER_10, lora.GetTransmittingPower());
}

void test_SetEncryption(void)
{
  lora.SetEncryption(4437);
  lora.CommitChanges();
}

void test_SendStructuredMessage(void)
{
  bool retstat = lora.SendMessage(&ftMessage);

  TEST_ASSERT_EQUAL_UINT8(true, retstat);
}

void test_SendMessage(void)
{
  lora.SendMessage(ftMessage.Message);
}

void test_ReceiveMessage(void)
{
  char buffer1[241] = {0};
  size_t len = 0;
  uint8_t rssi = 0;

  lora.ReceiveMessage(buffer1, sizeof(buffer1), len, rssi, true);
  TEST_ASSERT_EQUAL_UINT8(237, len);
}

// void test_x(void)
// {
//   TEST_ASSERT_EQUAL_UINT8(1, 1);
// }

void process()
{
  delay(3000);

  ftMessage.Add_H = 00;
  ftMessage.Add_L = 01;
  ftMessage.Channel = 18;
  ftMessage.Message = msg;

  UNITY_BEGIN(); // IMPORTANT LINE!

  pinMode(A0, OUTPUT);
  // pinMode(A1, OUTPUT);
  pinMode(13, OUTPUT);

  digitalWrite(A0, LOW);
  // digitalWrite(A1, LOW);
  digitalWrite(A0, HIGH);
  delay(1);
  digitalWrite(A0, LOW);

  // lora.SetModuleMode(Normal);
  // delay(200);

  RUN_TEST(test_Initializing);
  RUN_TEST(test_ResetToFactoryDefault);
  // RUN_TEST(test_ScanUartSpeed);
  // RUN_TEST(test_SetUartBaudRate);
  // RUN_TEST(test_GetUartBaudRate);
  RUN_TEST(test_SetModuleAddress); // to 40h 40h
  // RUN_TEST(test_SetChannelTo9);
  // RUN_TEST(test_SetAirDataRate);
  RUN_TEST(test_SetTransPower10);
  // RUN_TEST(test_SetRSSiEnable);
  // RUN_TEST(test_SetFixedTransMode);
  // RUN_TEST(test_SetEncryption);
  RUN_TEST(test_SendStructuredMessage);
  // RUN_TEST(test_GetRRSI);
  // lora.SetModuleMode(E22T_Mode_t::DeepSleep);

  // delay(3000);
  // RUN_TEST(test_ReceiveMessage);
  // RUN_TEST(test_GetRRSI);

  // RUN_TEST(test_ResetToFactoryDefault);

  UNITY_END(); // stop unit testing
}

void setup()
{
  process();
}

void loop()
{
  // put your main code here, to run repeatedly:
}