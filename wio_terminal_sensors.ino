#define LGFX_WIO_TERMINAL

#include <LovyanGFX.hpp>
#include <LGFX_AUTODETECT.hpp>

#include <Seeed_FS.h>
#include "SD/Seeed_SD.h"

#include <rpcBLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLE2904.h>

#include <LIS3DHTR.h>

#include <ArduinoJson.h>

namespace {
  LGFX lcd;
  LGFX_Sprite sprite_meter(&lcd);
  LIS3DHTR<TwoWire> imu;

  StaticJsonDocument<1024> json_document;

  BLECharacteristic* pCharacteristicX = nullptr;
  BLECharacteristic* pCharacteristicY = nullptr;
  BLECharacteristic* pCharacteristicZ = nullptr;
  BLECharacteristic* pCharacteristicLight = nullptr;

  BLE2902 cccdX;
  BLE2902 cccdY;
  BLE2902 cccdZ;
  BLE2902 cccdLight;

  BLE2904 ble_format;

  BLEUUID service_uuid("d1e60e98-06ee-455a-ba1f-1fa773e903fd");
  BLEUUID characteristic_x_uuid("3b34b8bd-d4a1-41f4-8874-688cef7df2c4");
  BLEUUID characteristic_y_uuid("b8640971-802e-41aa-8cc0-626fe0f740dd");
  BLEUUID characteristic_z_uuid("ba4b6397-f53b-4623-a70e-0d4535374524");
  BLEUUID characteristic_light_uuid("c5096544-4725-46ab-af9d-b0a669e65f70");
}

void setup() {
  Serial.begin(115200);

  lcd.init();
  lcd.setFont(&fonts::lgfxJapanGothic_20);

  sprite_meter.createSprite(160, 100);
  sprite_meter.setFont(&fonts::lgfxJapanGothic_20);

  pinMode(WIO_LIGHT, INPUT);
  pinMode(WIO_BUZZER, OUTPUT);

  pinMode(WIO_KEY_A, INPUT_PULLUP);
  pinMode(WIO_KEY_B, INPUT_PULLUP);
  pinMode(WIO_KEY_C, INPUT_PULLUP);

  pinMode(WIO_5S_UP, INPUT_PULLUP);
  pinMode(WIO_5S_DOWN, INPUT_PULLUP);
  pinMode(WIO_5S_LEFT, INPUT_PULLUP);
  pinMode(WIO_5S_RIGHT, INPUT_PULLUP);
  pinMode(WIO_5S_PRESS, INPUT_PULLUP);

  imu.begin(Wire1);
  imu.setOutputDataRate(LIS3DHTR_DATARATE_25HZ);
  imu.setFullScaleRange(LIS3DHTR_RANGE_2G);

  if (!SD.begin(SDCARD_SS_PIN, SDCARD_SPI))
  {
    lcd.drawString("SD open failed.", 0, 220);
    return;
  }
  File file = SD.open("wio_settings.json", FILE_READ);
  DeserializationError err = deserializeJson(json_document, file);
  if (err != DeserializationError::Ok)
  {
    lcd.drawString("Deserialize failed.", 0, 220);
    return;
  }

  const char* ssid = json_document["ssid"];
  const char* password = json_document["password"];

  BLEDevice::init("Wio Terminal");
  BLEServer* pServer = BLEDevice::createServer();
  BLEService* pService = pServer->createService(service_uuid);

  ble_format.setFormat(BLE2904::FORMAT_FLOAT32);

  pCharacteristicX = pService->createCharacteristic(characteristic_x_uuid, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicX->setAccessPermissions(GATT_PERM_READ);
  pCharacteristicX->addDescriptor(&cccdX);
  pCharacteristicX->addDescriptor(&ble_format);

  pCharacteristicY = pService->createCharacteristic(characteristic_y_uuid, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicY->setAccessPermissions(GATT_PERM_READ);
  pCharacteristicY->addDescriptor(&cccdY);
  pCharacteristicY->addDescriptor(&ble_format);

  pCharacteristicZ = pService->createCharacteristic(characteristic_z_uuid, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicZ->setAccessPermissions(GATT_PERM_READ);
  pCharacteristicZ->addDescriptor(&cccdZ);
  pCharacteristicZ->addDescriptor(&ble_format);

  pCharacteristicLight = pService->createCharacteristic(characteristic_light_uuid, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pCharacteristicLight->setAccessPermissions(GATT_PERM_READ);
  pCharacteristicLight->addDescriptor(&cccdLight);
  pCharacteristicLight->addDescriptor(&ble_format);

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(service_uuid);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void draw_meter(int32_t value, int32_t min, int32_t max, const char* name, int32_t x, int32_t y);
void draw_stick();

void loop() {
  float x = imu.getAccelerationX();
  float y = imu.getAccelerationY();
  float z = imu.getAccelerationZ();

  pCharacteristicX->setValue(x);
  pCharacteristicX->notify();
  delay(3);
  pCharacteristicY->setValue(y);
  pCharacteristicY->notify();
  delay(3);
  pCharacteristicZ->setValue(z);
  pCharacteristicZ->notify();
  delay(3);

  draw_meter(x, -1, 1, "X", 0, 20);
  draw_meter(y, -1, 1, "Y", 160, 20);
  draw_meter(z, -1, 1, "Z", 00, 120);

  float light = analogRead(WIO_LIGHT);

  pCharacteristicLight->setValue(light);
  pCharacteristicLight->notify();
  delay(3);

  draw_meter(light, 0, 1023, "Light", 160, 120);

  draw_stick();

  delay(21);
}

void draw_meter_scale(int32_t x, int32_t y, int32_t radius, int32_t length, float scale)
{
  const int32_t scale_x_s = x - radius * sin(scale * M_PI / 2 + M_PI * 3 / 4);
  const int32_t scale_x_e = x - (radius + length) * sin(scale * M_PI / 2 + M_PI * 3 / 4);
  const int32_t scale_y_s = y + radius * cos(scale * M_PI / 2 + M_PI * 3 / 4);
  const int32_t scale_y_e = y + (radius + length) * cos(scale * M_PI / 2 + M_PI * 3 / 4);
  sprite_meter.drawLine(scale_x_s, scale_y_s, scale_x_e, scale_y_e, 0x000000U);
}

void draw_meter(float value, float min, float max, const char* name, int32_t x, int32_t y)
{
  // 背景
  sprite_meter.fillRect(0, 0, 160, 100, 0xFFFFFFU);

  // 色目盛
  sprite_meter.fillArc(80, 100, 70, 80, 225, 255, 0xFF0000U);
  sprite_meter.fillArc(80, 100, 70, 80, 255, 285, 0xFFFF00U);
  sprite_meter.fillArc(80, 100, 70, 80, 285, 315, 0x00FF00U);
  
  // 目盛
  sprite_meter.drawArc(80, 100, 85, 85, 225, 315, 0x000000U);
  draw_meter_scale(80, 100, 85, 5, 0.0f);
  draw_meter_scale(80, 100, 85, 5, 0.2f);
  draw_meter_scale(80, 100, 85, 5, 0.4f);
  draw_meter_scale(80, 100, 85, 5, 0.6f);
  draw_meter_scale(80, 100, 85, 5, 0.8f);
  draw_meter_scale(80, 100, 85, 5, 1.0f);

  // 文字
  sprite_meter.setTextDatum(MC_DATUM);
  sprite_meter.setTextColor(0x000000U, 0xFFFFFFU);
  sprite_meter.drawString(name, 80, 60);

  // 針
  int32_t line_x = 80 - 90 * sin((value - min) / (max - min) * M_PI / 2 + M_PI * 3 / 4);
  int32_t line_y = 100 + 90 * cos((value - min) / (max - min) * M_PI / 2 + M_PI * 3 / 4);
  sprite_meter.drawLine(80, 100, line_x, line_y, 0x000000U);
  
  // 針の根元
  sprite_meter.fillArc(80, 100, 0, 20, 180, 360, 0x999999U);

  // 外周
  sprite_meter.drawRect(0, 0, 160, 100, 0x000000U);

  // 描画
  sprite_meter.pushSprite(x, y);
}

void draw_stick()
{
  lcd.drawString("Switch=", 200, 220);
  lcd.fillRect(270, 220, 150, 20, 0x000000U);
  if (digitalRead(WIO_5S_UP) == LOW)
  {
    lcd.drawString("UP", 270, 220);
  }
  else if (digitalRead(WIO_5S_DOWN) == LOW)
  {
    lcd.drawString("DOWN", 270, 220);
  }
  else if (digitalRead(WIO_5S_LEFT) == LOW)
  {
    lcd.drawString("LEFT", 270, 220);
  }
  else if (digitalRead(WIO_5S_RIGHT) == LOW)
  {
    lcd.drawString("RIGHT", 270, 220);
  }
  else if (digitalRead(WIO_5S_PRESS) == LOW)
  {
    lcd.drawString("PRESS", 270, 220);
  }
}