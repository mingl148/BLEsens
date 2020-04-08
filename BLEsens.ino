

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
BLEUnsignedCharCharacteristic batteryLevelChar("2A19",               // standard 16-bit characteristic UUID
                                               BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes
BLEService Sensor("181C");
BLEShortCharacteristic temp("2A6E",               // standard 16-bit characteristic UUID
                            BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEUnsignedShortCharacteristic humidity("2A6F",               // standard 16-bit characteristic UUID
                                        BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEIntCharacteristic pressure("2A6D",               // standard 16-bit characteristic UUID
                              BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEIntCharacteristic temphumi_interval("19B10001-E8F2-537E-4F6C-D104768A1214", // standard 16-bit characteristic UUID
                                       BLERead | BLEWrite);
BLEIntCharacteristic pressure_interval("19B10001-E8F2-537E-4F6C-D104768A1215", // standard 16-bit characteristic UUID
                                       BLERead | BLEWrite);

CircularBuffer<int, 32> PressureBuf;
CircularBuffer<short, 32> TemperatureBuf;
CircularBuffer<short, 32> HumidityBuf;

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

#define temperature_calibration_k0 1
#define temperature_calibration_k1 0
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

void BLE_valueUpdate()
{
    if (pressure.subscribed() && temp.subscribed() && humidity.subscribed() && update)
    {
        Led_color(color_BLE);
        batteryLevelChar.writeValue(oldBatteryLevel);
        pressure.writeValue(pressure_sens);
        temp.writeValue(temperature_sens);
        humidity.writeValue(humidity_sens);
        update = false;
    }
}

void update_sens()
{
    if (currentMillis - pressureLastMillis >= intervalPressure)
    {
        // Led_color(color_PRESS);
        pressure_sens = (BARO.readPressure() * pressure_conversion);
        pressure_sens = pressure_sens * pressure_calibration_k0 + pressure_calibration_k1;
        update = true;
        pressureLastMillis = currentMillis;
        TemperatureBuf.push(int(temperature_sens * 100));
        HumidityBuf.push(int(humidity_sens * 100));
        PressureBuf.push(int(pressure_sens * 10));
    }

    if (currentMillis - tempsensLastMillis >= intervalTemp)
    {
        // Led_color(color_TEMPHUMI);
        temperature_sens = (HTS.readTemperature() * temperature_conversion);
        temperature_sens = (temperature_sens * temperature_calibration_k0) + temperature_calibration_k1;

        humidity_sens = (HTS.readHumidity() * humidity_conversion);
        humidity_sens = humidity_sens * humidity_calibration_k0 + humidity_calibration_k1;
        update = true;
        tempsensLastMillis = currentMillis;
    }

    if (currentMillis - batteryLastMillis >= intervalBattery)
    {
        // Led_color(color_BATTERY);
        battery = analogRead(A0);
        int batteryLevel = map(battery, 0, 1023, 0, 100);
        if (batteryLevel != oldBatteryLevel)
        {
            // if the battery level has changed

            // and update the battery level characteristic
            oldBatteryLevel = batteryLevel;
            // save the level for next comparison
            update = true;
        }
        batteryLastMillis = currentMillis;
    }
}

void sendBySerial()
{
    if (Serial.available > 0)
    {
        String receiver = Serial.readString();
        if (receiver[0] == 's')
        {
            //  Led_color(color_SERIAL);
            Serial.print("time to send");
            Serial.print(timetosend);
            Serial.print("Battery Level ");
            Serial.print(oldBatteryLevel);
            Serial.print(" % ");
            Serial.print(" teplota: ");
            Serial.print(temperature_sens);
            Serial.print(" C ");
            Serial.print(" vlhkost: ");
            Serial.print(humidity_sens);
            Serial.print(" % ");
            Serial.print(" tlak: ");
            Serial.print(pressure_sens);
            Serial.print(" Pa ");
            Serial.println("");
        }
    }
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
    digitalWrite(LED_BUILTIN, LOW);
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
        while (1)
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
            Led_color(color_TEMPHUMI);
            delay(250);
            Led_color(color_OFF);
            delay(250);
        }
        Serial.println("teplota meri");
    }
    else
    {
        while (1)
        {
            Led_color(color_ERROR);
            delay(250);
            Led_color(color_TEMPHUMI);
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
    Sensor.addCharacteristic(temphumi_interval);
    Sensor.addCharacteristic(pressure_interval);
    BLE.addService(Sensor);

    pressure.writeValue(pressure_sens);
    temp.writeValue(temperature_sens);
    temphumi_interval.writeValue(intervalTemp);

    humidity.writeValue(humidity_sens);
    pressure_interval.writeValue(intervalPressure);

    pressure.setEventHandler(BLERead, read_sensPressure);
    humidity.setEventHandler(BLERead, read_sensHumidity);
    temp.setEventHandler(BLERead, read_sensTemp);

    BLE.advertise();
    Serial.println("Bluetooth device active, waiting for connections...");
}

void read_sensPressure(BLEDevice central, BLECharacteristic characteristic)
{
    
    pressure.writeValue(pressure_sens);
}

void read_sensHumidity(BLEDevice central, BLECharacteristic characteristic)
{
 humidity.writeValue(humidity_sens);
}

void read_sensTemp(BLEDevice central, BLECharacteristic characteristic)
{
    temp.writeValue(temperature_sens);
}

void blePeripheralConnectHandler(BLEDevice central)
{
    // central connected event handler
    Serial.print("Connected event, central: ");
    Serial.println(central.address());

    while (central.connected())
    {
        Led_color(color_OFF);
        currentMillis = millis();
        timetosend = micros();
        update_sens();
        BLE_valueUpdate();
        timetosend = micros() - timetosend;
        sendBySerial();

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