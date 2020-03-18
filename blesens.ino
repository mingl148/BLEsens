

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
long sendLastMillis = 0;
long currentMillis = 0;

float pressure_sens = 0;
short temperature_sens = 0;
short humidity_sens = 0;

const int intervalSend = 200;
const int intervalBattery = 20000;
const int intervalPressure = 200;
const int intervalTemp = 20000;





#define RED 0,1,1
#define BLUE 1,1,0
#define GREEN 1,0,1
#define BLACK 1,1,1
#define CYAN 1,0,0
#define YELLOW 0,1,0
#define PURPLE 0,1,0
#define WHITE 0,0,0

void setup()
{

  Serial.begin(9600);


  pinMode(ledPinRed, OUTPUT);
  pinMode(ledPinGreen, OUTPUT);
  pinMode(ledPinBlue, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Led_color(RED);
  delay(1000);
  Led_color(BLUE);
  delay(1000);
  Led_color(GREEN);
  delay(1000);
  Led_color(YELLOW);
  delay(1000);
  Led_color(CYAN);
  delay(1000);
  Led_color(PURPLE);
  delay(1000);
  Led_color(WHITE);
  delay(1000);
  Led_color(BLACK);
  delay(1000);

  if (Serial)
  { for (int i = 0; i < 5; i++)
    {
      Led_color(GREEN);
      delay(250);
      Led_color(0, 0, 0);
      delay(250);
    }
  }

  if (BARO.begin())
  { for (int i = 0; i < 5; i++)
    {
      Led_color(PURPLE);
      delay(500);
      Led_color(BLACK);
      delay(500);
    }
    Serial.println("tlak meri");
  }
  else {
    while (1)
    {
      Led_color(RED);
      delay(250);
      Led_color(PURPLE);
      delay(250);
      Led_color(0, 0, 0);
      delay(250);
    }
  }

  if (HTS.begin())
  {
    for (int i = 0; i < 5; i++)
    {
      Led_color(CYAN);
      delay(500);
      Led_color(BLACK);
      delay(500);
    }
    Serial.println("teplota meri");
  }
  else
  {
    while (1)
    {
      Led_color(RED);
      delay(250);
      Led_color(CYAN);
      delay(250);
      Led_color(BLACK);
      delay(250);
    }
  }


  // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin())
  {
    Serial.println("starting BLE failed!");
    while (1)
    {
      Led_color(RED);
      delay(250);
      Led_color(YELLOW);
      delay(250);
      Led_color(0, 0, 0);
      delay(250);
    }
  }
  else
  {
    Serial.println("BLE jede");
    for (int i = 0; i < 5; i++)
    {
      Led_color(YELLOW);
      delay(250);
      Led_color(0, 0, 0);
      delay(250);
    }
  }

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

    while (central.connected())
    {
      currentMillis = millis();
      updateBatteryLevel();
      update_sens();
      if (Serial)
      {
        sendBySerial();
      }
      delay(250);
      Led_color(BLACK);
    }
  }

  Led_color(BLACK);
  digitalWrite(LED_BUILTIN, LOW);
  Serial.print("Disconnected from central: ");
  Serial.println(central.address());
}

void updateBatteryLevel()
{
  if (currentMillis - batteryLastMillis >= intervalBattery)
  {
    Led_color(YELLOW);
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
#define pressure_calibration_k0   1
#define pressure_calibration_k1   0
  if (currentMillis - pressureLastMillis >= intervalPressure)
  {


    Led_color(PURPLE);
    pressure_sens = BARO.readPressure() * 10000;
    pressure_sens =  pressure_sens * pressure_calibration_k0 +  pressure_calibration_k1;
    //pressure_sensint = pressure_sens;
    pressure.writeValue(pressure_sens);
    pressureLastMillis = currentMillis;
  }

  if (currentMillis - tempsensLastMillis >= intervalTemp)
  {
#define   temperature_calibration_k0   1
#define   temperature_calibration_k1   -1000
#define   humidity_calibration_k0   1
#define   humidity_calibration_k1  0
    Led_color(CYAN);
    temperature_sens = HTS.readTemperature() * 100;
    temperature_sens = (temperature_sens * temperature_calibration_k0) + temperature_calibration_k1;

    humidity_sens = HTS.readHumidity() * 100;
    humidity_sens =  humidity_sens * humidity_calibration_k0 + humidity_calibration_k1;

    temp.writeValue(temperature_sens);
    humidity.writeValue(humidity_sens);

    tempsensLastMillis = currentMillis;
  }
}

void sendBySerial()
{
  Led_color(GREEN);
  if (currentMillis - sendLastMillis >= intervalSend)
  {
    Serial.print(" teplota: ");
    Serial.print(temperature_sens);
    Serial.print(" vlhkost: ");
    Serial.print(humidity_sens);
    Serial.print(" tlak: ");
    Serial.println(pressure_sens);
    sendLastMillis = currentMillis;
  }
}

void Led_color( bool Red, bool Blue , bool Green) {

  digitalWrite(ledPinRed, Red);
  digitalWrite(ledPinBlue, Blue);
  digitalWrite(ledPinGreen, Green);
}
