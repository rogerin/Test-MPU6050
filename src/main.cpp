#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

#define LED_DIREITA 19
#define LED_ESQUERDA 18

void setup() {
    Serial.begin(9600);
    Wire.begin(21, 22);  // Configura I2C com GPIO 21 (SDA) e GPIO 22 (SCL)

    Serial.println("Inicializando MPU6050...");

    if (!mpu.begin()) {
        Serial.println("Falha ao encontrar o MPU6050. Verifique as conexões!");
        while (1);
    }

    Serial.println("MPU6050 conectado com sucesso!");

    // Configuração do sensor
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

       pinMode(LED_DIREITA, OUTPUT);
    pinMode(LED_ESQUERDA, OUTPUT);

    // Garante que os LEDs começam desligados
    digitalWrite(LED_DIREITA, LOW);
    digitalWrite(LED_ESQUERDA, LOW);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float giroZ = g.gyro.z;  // Captura o valor do giroscópio no eixo Z

    // Serial.print("Giroscópio (°/s): X=");
    // Serial.print(g.gyro.x);
    // Serial.print(" Y=");
    // Serial.print(g.gyro.y);
    // Serial.print(" Z=");
    // Serial.println(giroZ);

    // Define um limiar para evitar ruído (valores pequenos)
    float threshold = 1; // Ajuste conforme necessário

    if (giroZ > threshold) {
        // Serial.println("Girando para a ESQUERDA (Anti-horário)");
        digitalWrite(LED_DIREITA,LOW);
        digitalWrite(LED_ESQUERDA, HIGH);
    } 
    else if (giroZ < -threshold) {
        // Serial.println("Girando para a DIREITA (Horário)");
         digitalWrite(LED_DIREITA, HIGH);
        digitalWrite(LED_ESQUERDA, LOW);
    } 
    else {
        // Serial.println("Sem rotação significativa");
        digitalWrite(LED_DIREITA, LOW);
        digitalWrite(LED_ESQUERDA, LOW);
    }

    //Serial.println("------------------------------------");
    delay(5);
}