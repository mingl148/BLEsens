

/*kjkj
*/

#include <Arduino_LPS22HB.h>
#include <BARO.h>
#include <Arduino_HTS221.h>
#include <HTS.h>
#include <ArduinoBLE.h>

#define ledPinRed 22
#define ledPinBlue 23
#define ledPinGreen 24

// BLE Battery Service
BLEService batteryService("180F");

// BLE Battery Level Characteristic
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",               // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
BLEService Sensor("181C");
BLEShortCharacteristic temp("2A6E",               // standard 16-bit characteristic UUID
                            BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEUnsignedShortCharacteristic humidity("2A6F",               // standard 16-bit characteristic UUID
                                        BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEIntCharacteristic pressure("2A6D",               // standard 16-bit characteristic UUID
                              BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

int oldBatteryLevel = 0; // last battery level reading from analog input
int battery = 0;
long pressureLastMillis = 0;
long batteryLastMillis = 0;
long tempsensLastMillis = 0;
long currentMillis = 0;

float pressure_sens = 0;
float temperature_sens = 0;
float humidity_sens = 0;

const int intervalSend = 1000;
const int intervalBattery = 5000;
const int intervalPressure = 200;
const int intervalTemp = 1000;

short humidity_sensint = 500;
short temperature_sensint = 320;
short pressure_sensint = 10040;

void setup()
{

  Serial.begin(9600);
  while (!Serial);
  Serial.println("komunikace jede");
  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinBlue, OUTPUT);
  Serial.println("LED nastaveny");


  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(ledPinRed, LOW);
  digitalWrite(ledPinGreen, LOW);
  digitalWrite(ledPinBlue, LOW);
  Serial.println("sviti");
  delay(1000);
  digitalWrite(ledPinRed, HIGH);
  digitalWrite(ledPinGreen, HIGH);
  digitalWrite(ledPinBlue, HIGH);
  Serial.println("nesviti");
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(ledPinRed, LOW);
    delay(500);
    digitalWrite(ledPinRed, HIGH);
    delay(500);
    Serial.println("blik");
  }

  while (!Serial)
  {
    digitalWrite(ledPinBlue, LOW);
    delay(250);
    digitalWrite(ledPinBlue, HIGH);
    delay(250);
  }
  Serial.println("komunikuje");
  if (!BARO.begin())
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1)
    {
      digitalWrite(ledPinRed, !digitalRead(ledPinRed));
      delay(500);
    }
  }
  Serial.println("tlak meri");
  if (!HTS.begin())
  {
    Serial.println("Failed to initialize pressure sensor!");
    while (1)
    {
      digitalWrite(ledPinRed, !digitalRead(ledPinRed));
      delay(500);
    }
  }
  Serial.println("teplota meri");

  // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");

    while (1)
    {
      digitalWrite(ledPinRed, !digitalRead(ledPinRed));
      delay(500);
    }
  }
  Serial.println("BLE jede");


  BLE.setLocalName("BLEsens");

  BLE.setAdvertisedService(batteryService);
  batteryService.addCharacteristic(batteryLevelChar);
  BLE.addService(batteryService);
  batteryLevelChar.writeValue(oldBatteryLevel);


  BLE.setAdvertisedService(Sensor);
  Sensor.addCharacteristic(temp);
  Sensor.addCharacteristic(pressure);
  Sensor.addCharacteristic(humidity);
  BLE.addService(Sensor);
  temp.writeValue(temperature_sens);
  humidity.writeValue(humidity_sens);
  pressure.writeValue(pressure_sens);

  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
}

void loop()
{
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central)
  {
    Serial.print("Connected to central: ");
    // print the central's BT address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected())
    {
      currentMillis = millis();
      updateBatteryLevel();
      update_sens();
      //   sendBySerial();
    }
  }
  // when the central disconnects, turn off the LED:
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());

}

void updateBatteryLevel()
{
  /* Read the current voltage level on the A0 analog input pin.
    This is used here to simulate the charge level of a battery.
  */
  if (currentMillis - batteryLastMillis >= intervalBattery)
  {
    battery = analogRead(A0);
    int batteryLevel = map(battery, 0, 1023, 0, 100);
    if (batteryLevel != oldBatteryLevel)
    {
      // if the battery level has changed
      Serial.print("Battery Level % is now: ");         // print it
      Serial.println(batteryLevel);
      batteryLevelChar.writeValue(batteryLevel); // and update the battery level characteristic
      oldBatteryLevel = batteryLevel;
      // save the level for next comparison
    }
    batteryLastMillis = currentMillis;
  }
}

void update_sens()
{
  if (currentMillis - pressureLastMillis >= intervalPressure)
  {
    pressure_sens = BARO.readPressure() * 10000;
    //pressure_sensint = pressure_sens;
    pressure.writeValue(pressure_sens);
    Serial.print(" tlak: ");
    Serial.print(pressure_sens);
    pressureLastMillis = currentMillis;
  }

  if (currentMillis - tempsensLastMillis >= intervalTemp)
  {
    temperature_sens = HTS.readTemperature() * 100;
    // temperature_sensint = temperature_sens * 10;
    humidity_sens = HTS.readHumidity() * 100;
    // humidity_sensint = humidity_sens * 10;
    temp.writeValue(temperature_sens);
    humidity.writeValue(humidity_sens);
    //  Serial.print(" teplota: ");
    //  Serial.print(temperature_sens);
    //  Serial.print(" vlhkost: ");
    //  Serial.println(humidity_sens);
    tempsensLastMillis = currentMillis;
  }
}

void sendBySerial()
{
  if (currentMillis - tempsensLastMillis >= intervalSend)
  {

    Serial.print(" vlhkost: ");
    Serial.print(humidity_sensint);
    Serial.print(" tlak: ");
    Serial.print(humidity_sens);
    tempsensLastMillis = currentMillis;
  }
}
