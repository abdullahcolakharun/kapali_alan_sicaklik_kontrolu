// =====================================================
// ARDUINO MODBUS SLAVE - PELTİER KONTROL (BASİT)
// =====================================================
// 3 Bölüm:
// 1. Sensör ölçümü (x100 int)
// 2. Modbus haberleşme
// 3. Röle kontrolü
// =====================================================

#include <ModbusRTUSlave.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ==================== PIN TANIMLARI ====================
#define DE_RE_PIN 2          // MAX485 DE/RE

#define SENSOR1_PIN 8        // Dışarıda
#define SENSOR2_PIN 11       // İçeride 1
#define SENSOR3_PIN 12       // İçeride 2

#define SSR_PIN 5            // SSR PWM
#define RELAY1_PIN 6         // Röle 1
#define RELAY2_PIN 7         // Röle 2

// ==================== MODBUS ====================
#define SLAVE_ID 1
#define BAUD 9600

ModbusRTUSlave modbus(Serial, DE_RE_PIN);
uint16_t holdingRegisters[8];

// Register adresleri:
// 0: İç sıcaklık 1 (x100)
// 1: İç sıcaklık 2 (x100)
// 2: Dış sıcaklık (x100)
// 3: Kontrol çıkışı (-255...+255) - PLC'den gelir

// ==================== SENSÖRLER ====================
OneWire oneWire1(SENSOR1_PIN);
OneWire oneWire2(SENSOR2_PIN);
OneWire oneWire3(SENSOR3_PIN);

DallasTemperature sensor1(&oneWire1);
DallasTemperature sensor2(&oneWire2);
DallasTemperature sensor3(&oneWire3);

int16_t temp_inside1 = 0;   // x100
int16_t temp_inside2 = 0;   // x100
int16_t temp_outside = 0;   // x100

// ==================== ZAMANLAMA ====================
unsigned long lastSensorRead = 0;

// ==================== SETUP ====================
void setup() {
    // Pinler
    pinMode(SSR_PIN, OUTPUT);
    pinMode(RELAY1_PIN, OUTPUT);
    pinMode(RELAY2_PIN, OUTPUT);
    pinMode(LED_BUILTIN, OUTPUT);
    
    // Başlangıçta kapalı
    analogWrite(SSR_PIN, 0);
    digitalWrite(RELAY1_PIN, LOW);
    digitalWrite(RELAY2_PIN, LOW);
    
    // Serial ve Modbus
    Serial.begin(BAUD);
    modbus.begin(SLAVE_ID, BAUD);
    modbus.configureHoldingRegisters(holdingRegisters, 8);
    
    // Sensörler
    sensor1.begin();
    sensor2.begin();
    sensor3.begin();
    
    sensor1.setResolution(11);
    sensor2.setResolution(11);
    sensor3.setResolution(11);
    
    // Başlangıç blink
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(200);
        digitalWrite(LED_BUILTIN, LOW);
        delay(200);
    }
}

// ==================== LOOP ====================
void loop() {
    
    // ========== BÖLÜM 1: SENSÖR ÖLÇÜMÜ ==========
    if (millis() - lastSensorRead >= 1000) {
        lastSensorRead = millis();
        
        float raw;
        
        // Sensör 1 - Dışarıda
        sensor1.requestTemperatures();
        raw = sensor1.getTempCByIndex(0);
        if (raw > -50 && raw < 125) {
            temp_outside = (int16_t)(raw * 100);
        }
        
        // Sensör 2 - İçeride 1
        sensor2.requestTemperatures();
        raw = sensor2.getTempCByIndex(0);
        if (raw > -50 && raw < 125) {
            temp_inside1 = (int16_t)(raw * 100);
        }
        
        // Sensör 3 - İçeride 2
        sensor3.requestTemperatures();
        raw = sensor3.getTempCByIndex(0);
        if (raw > -50 && raw < 125) {
            temp_inside2 = (int16_t)(raw * 100);
        }
        
        // Registerlere yaz
        holdingRegisters[0] = (uint16_t)temp_inside1;
        holdingRegisters[1] = (uint16_t)temp_inside2;
        holdingRegisters[2] = (uint16_t)temp_outside;
    }
    
    // ========== BÖLÜM 2: MODBUS HABERLEŞME ==========
    modbus.poll();
    
    // ========== BÖLÜM 3: RÖLE KONTROLÜ ==========
    int16_t controlOutput = (int16_t)holdingRegisters[3];
    
    // Sınırla
    if (controlOutput > 255) controlOutput = 255;
    if (controlOutput < -255) controlOutput = -255;
    
    // Önceki durumu hatırla
    static bool wasHeating = false;
    static bool wasCooling = false;
    
    if (controlOutput > 0) {
        // ISITMA: Röle1=HIGH, Röle2=HIGH
        
        // Soğutmadan ısıtmaya geçiş - SSR kapat, bekle
        if (wasCooling) {
            analogWrite(SSR_PIN, 0);
            delay(100);  // 100ms güvenlik
        }
        
        digitalWrite(RELAY1_PIN, HIGH);
        digitalWrite(RELAY2_PIN, HIGH);
        analogWrite(SSR_PIN, abs(controlOutput));
        
        wasHeating = true;
        wasCooling = false;
    }
    else if (controlOutput < 0) {
        // SOĞUTMA: Röle1=LOW, Röle2=LOW
        
        // Isıtmadan soğutmaya geçiş - SSR kapat, bekle
        if (wasHeating) {
            analogWrite(SSR_PIN, 0);
            delay(100);  // 100ms güvenlik
        }
        
        digitalWrite(RELAY1_PIN, LOW);
        digitalWrite(RELAY2_PIN, LOW);
        analogWrite(SSR_PIN, abs(controlOutput));
        
        wasHeating = false;
        wasCooling = true;
    }
    else {
        // OFF: PWM=0
        analogWrite(SSR_PIN, 0);
        wasHeating = false;
        wasCooling = false;
    }
}

// ==================== NOTLAR ====================
/*
 * MODBUS REGISTER:
 * ----------------
 * 0: İç sıcaklık 1 (x100) - Sensör 2
 * 1: İç sıcaklık 2 (x100) - Sensör 3
 * 2: Dış sıcaklık (x100)  - Sensör 1
 * 3: Kontrol çıkışı (-255...+255) - PLC yazar
 *
 * KONTROL:
 * --------
 * +255: Tam ısıtma (R1=H, R2=H, PWM=255)
 * +127: %50 ısıtma (R1=H, R2=H, PWM=127)
 *    0: OFF (PWM=0)
 * -127: %50 soğutma (R1=L, R2=L, PWM=127)
 * -255: Tam soğutma (R1=L, R2=L, PWM=255)
 *
 * PLC OKUMA:
 * ----------
 * tempReal = INT_TO_REAL(register) / 100.0
 *
 * PIN BAĞLANTILARI:
 * -----------------
 * 0 (RX)  → MAX485 RO
 * 1 (TX)  → MAX485 DI
 * 2       → MAX485 DE/RE
 * 5       → SSR PWM
 * 6       → Röle 1
 * 7       → Röle 2
 * 8       → Sensör 1 (Dış)
 * 11      → Sensör 2 (İç 1)
 * 12      → Sensör 3 (İç 2)
 */
