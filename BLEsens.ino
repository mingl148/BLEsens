

/*kjkj
*/

#include <Arduino_LPS22HB.h>
#include <BARO.h>
#include <Arduino_HTS221.h>
#include <HTS.h>
#include <ArduinoBLE.h>
#include <mbed.h>
#include <mbed_memory_status.h>
#include <mbed_wait_api.h>
#include <platform/CircularBuffer.h>

using namespace mbed;

#define ledPinRed 22
#define ledPinBlue 23
#define ledPinGreen 24

// BLE Battery Service
BLEService batteryService("180F");

// BLE Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",   // standard 16-bit characteristic UUID
    BLERead); // remote clients will be able to get notifications if this characteristic changes
BLEService Sensor("181A");
BLEShortCharacteristic temp("2A6E",   // standard 16-bit characteristic UUID
                            BLERead); // remote clients will be able to get notifications if this characteristic changes

BLEUnsignedShortCharacteristic humidity("2A6F",   // standard 16-bit characteristic UUID
                                        BLERead); // remote clients will be able to get notifications if this characteristic changes

BLEIntCharacteristic pressure("2A6D",   // standard 16-bit characteristic UUID
                              BLERead); // remote clients will be able to get notifications if this characteristic changes

BLEIntCharacteristic sampleNum("2A58",               // standard 16-bit characteristic UUID
                               BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

CircularBuffer<short, 32> SampleNumBuf;
CircularBuffer<int, 32> PressureBuf;
CircularBuffer<short, 32> TemperatureBuf;
CircularBuffer<unsigned short, 32> HumidityBuf;

short NumSample = 0;
int oldBatteryLevel = 0; // last battery level reading from analog input
int battery = 0;

long pressureLastMillis = 0;
long batteryLastMillis = 0;
long tempsensLastMillis = 0;
long sendLastMillis = 0;
long currentMillis = 0;

int pressure_sens = 0;
short temperature_sens = 0;
short humidity_sens = 0;
double timetosend = 0;

int pressMem = 0;
unsigned short humiMem = 0;
short tempMem = 0;
short NumsampleMem = 0;

const int intervalSend = 200;
const int intervalBattery = 20000;
const int intervalPressure = 200;
const int intervalTemp = 20000;

bool update = false;

#define RED 0, 1, 1
#define BLUE 1, 1, 0
#define GREEN 1, 0, 1
#define BLACK 1, 1, 1
#define CYAN 1, 0, 0
#define YELLOW 0, 1, 0
#define PURPLE 0, 1, 0
#define WHITE 0, 0, 0

#define color_ERROR RED
#define color_BLE BLUE
#define color_SERIAL GREEN
#define color_PRESS PURPLE
#define color_TEMPHUMI YELLOW
#define color_BATTERY CYAN
#define color_OFF BLACK

#define pressure_calibration_k0 1
#define pressure_calibration_k1 0
#define pressure_conversion 10000

#define temperature_calibration_k0 0.0337
#define temperature_calibration_k1 1.6151
#define temperature_conversion 100

#define humidity_calibration_k0 1
#define humidity_calibration_k1 0
#define humidity_conversion 100

void setup()
{
  Serial.begin(115200);
  ledOutput_set();
  senzor_int();
  BLE_init();
}

void loop()
{
  BLE.poll();
}



void ledOutput_set()
{
  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinBlue, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  digitalWrite(ledPinRed, HIGH);
  digitalWrite(ledPinGreen, HIGH);
  digitalWrite(ledPinBlue, HIGH);
  digitalWrite(LED_BUILTIN, HIGH);
}

void senzor_int()
{
  if (BARO.begin())
  {
    for (int i = 0; i < 5; i++)
    {
      Led_color(color_PRESS);
      delay(250);
      Led_color(color_OFF);
      delay(250);
    }
    Serial.println("tlak meri");
  }
  else
  {
    while (!BARO.begin())
    {
      Led_color(color_ERROR);
      delay(250);
      Led_color(color_PRESS);
      delay(250);
      Led_color(color_OFF);
      delay(250);
    }
  }

  if (HTS.begin())
  {
    for (int i = 0; i < 5; i++)
    {
      Led_color(color_PRESS);
      delay(250);
      Led_color(color_OFF);
      delay(250);
    }
    Serial.println("teplota meri");
  }
  else
  {
    while (!HTS.begin())
    {
      Led_color(color_ERROR);
      delay(250);
      Led_color(color_PRESS);
      delay(250);
      Led_color(color_OFF);
      delay(250);
    }
  }
}

void BLE_init()
{
  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    while (!BLE.begin())
    {
      Led_color(color_ERROR);
      delay(250);
      Led_color(color_BLE);
      delay(250);
      Led_color(color_OFF);
      delay(250);
    }
  }
  else
  {
    Serial.println("BLE jede");
    for (int i = 0; i < 5; i++)
    {
      Led_color(color_BLE);
      delay(250);
      Led_color(color_OFF);
      delay(250);
    }
  }

  BLE.setLocalName("BLEsens");

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  BLE.setAdvertisedService(batteryService);
  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);
  batteryLevelChar.writeValue(oldBatteryLevel);
  BLE.setAdvertisedService(Sensor);

  Sensor.addCharacteristic(temp);
  Sensor.addCharacteristic(pressure);
  Sensor.addCharacteristic(humidity);
  Sensor.addCharacteristic(sampleNum);
  BLE.addService(Sensor);

  pressure.writeValue(pressure_sens);
  temp.writeValue(temperature_sens);
  humidity.writeValue(humidity_sens);
  sampleNum.writeValue(NumSample);

  pressure.setEventHandler(BLERead, read_sensPressure);
  humidity.setEventHandler(BLERead, read_sensHumidity);
  temp.setEventHandler(BLERead, read_sensTemp);
  sampleNum.setEventHandler(BLERead, read_Numsample);

  BLE.setAdvertisingInterval(50);
  BLE.advertise();
  Serial.println("Bluetooth device active, waiting for connections...");
}

void read_sensPressure(BLEDevice central, BLECharacteristic characteristic)
{ double current_millis = millis();
  float pressure_sensf = BARO.readPressure() * 10000;
  pressure_sens = pressure_sensf;
  // pressure_sens = ((pressure_sens * pressure_calibration_k0) + pressure_calibration_k1) * pressure_conversion;
  pressure.writeValue(pressure_sens);
  current_millis = millis() - current_millis;
  Serial.println(current_millis );
}

void read_sensHumidity(BLEDevice central, BLECharacteristic characteristic)
{
  double current_millis = millis();
  float humidity_sensf = HTS.readHumidity() * 100;
  humidity_sens = humidity_sensf;
  //  humidity_sens = ((humidity_sens * humidity_calibration_k0) + humidity_calibration_k1) * humidity_conversion;
  humidity.writeValue(humidity_sens);
  current_millis = millis() - current_millis;
  Serial.println(current_millis );
}

void read_sensTemp(BLEDevice central, BLECharacteristic characteristic)
{
  double current_millis = millis();
  float temperaturefloat = HTS.readTemperature();
 // temperaturefloat = temperaturefloat - ((temperaturefloat * temperature_calibration_k0) + temperature_calibration_k1);
  temperature_sens = temperaturefloat * 100;

  temp.writeValue(temperature_sens);
}

void read_Numsample(BLEDevice central, BLECharacteristic characteristic)
{

}

void blePeripheralConnectHandler(BLEDevice central)
{
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());

  while (central.connected())
  {
    Led_color(color_OFF);
  }
}

void blePeripheralDisconnectHandler(BLEDevice central)
{
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
}

void Led_color(bool Red, bool Blue, bool Green)
{
  digitalWrite(ledPinRed, Red);
  digitalWrite(ledPinBlue, Blue);
  digitalWrite(ledPinGreen, Green);
}
