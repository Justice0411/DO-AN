#define BLYNK_TEMPLATE_ID "TMPL6gAP3Z5ZZ"
#define BLYNK_TEMPLATE_NAME "DO AN"
#define BLYNK_AUTH_TOKEN "HwjcFUWL4SUIeBIhZdNdOBu1DRayTTUz"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Thông tin WiFi
const char* ssid = "202 HD4";
const char* password = "68686868";

// Khởi tạo LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Định nghĩa chân UART
#define RXD2 16
#define TXD2 17

// Khởi tạo timer Blynk
BlynkTimer timer;

// Struct lưu trữ dữ liệu
struct SensorData {
    float voltage;
    float current;
    float power;
    float light[4];
};

SensorData sensorData;

// Hàm gửi dữ liệu lên Blynk
void sendDataToBlynk() {
    Blynk.virtualWrite(V0, sensorData.voltage);  // Điện áp
    Blynk.virtualWrite(V1, sensorData.current);  // Dòng điện
    Blynk.virtualWrite(V2, sensorData.power);    // Công suất
    Blynk.virtualWrite(V3, sensorData.light[0]); // Ánh sáng TL
    Blynk.virtualWrite(V4, sensorData.light[1]); // Ánh sáng TR
    Blynk.virtualWrite(V5, sensorData.light[2]); // Ánh sáng BL
    Blynk.virtualWrite(V6, sensorData.light[3]); // Ánh sáng BR
}

void parseData(String data) {
    // Tách điện áp
    int voltageStart = data.indexOf("V:") + 2;  
    int voltageEnd = data.indexOf(";", voltageStart);
    if(voltageStart != -1 && voltageEnd != -1) {
        String voltageStr = data.substring(voltageStart, voltageEnd);
        sensorData.voltage = voltageStr.toFloat();
        Serial.println("V: " + voltageStr);
    }

    // Tách dòng điện
    int currentStart = data.indexOf("I:") + 2;  
    int currentEnd = data.indexOf(";", currentStart);
    if(currentStart != -1 && currentEnd != -1) {
        String currentStr = data.substring(currentStart, currentEnd);
        sensorData.current = currentStr.toFloat();
        Serial.println("I: " + currentStr);
    }

    // Tách công suất
    int powerStart = data.indexOf("P:") + 2;  
    int powerEnd = data.indexOf("\n", powerStart);
    if(powerStart != -1 && powerEnd != -1) {
        String powerStr = data.substring(powerStart, powerEnd);
        sensorData.power = powerStr.toFloat();
        Serial.println("P: " + powerStr);
    }

    // Tách dữ liệu cảm biến ánh sáng
    String lightLabels[] = {"TL:", "TR:", "BL:", "BR:"};
    for(int i = 0; i < 4; i++) {
        int lightStart = data.indexOf(lightLabels[i]) + 3;
        int lightEnd;
        if(i < 3) {
            lightEnd = data.indexOf(" ", lightStart);
        } else {
            lightEnd = data.indexOf("\r", lightStart);
        }
        
        if(lightStart != -1 && lightEnd != -1) {
            String lightStr = data.substring(lightStart, lightEnd);
            sensorData.light[i] = lightStr.toFloat();
            Serial.println(lightLabels[i] + " " + lightStr);
        }
    }
}

void updateLCD() {
    lcd.clear();
    
    // Dòng 1: Điện áp
    lcd.setCursor(0, 0);
    lcd.printf("V:%.2fV", sensorData.voltage);
    
    lcd.setCursor(10, 0);
    lcd.printf("TL:%.1f", sensorData.light[0]);

    // Dòng 2: Dòng điện 
    lcd.setCursor(0, 1);
    lcd.printf("I:%.3fA", sensorData.current);
    
    lcd.setCursor(10, 1);
    lcd.printf("TR:%.1f",sensorData.light[1]);

    lcd.setCursor(0, 2);
    lcd.printf("P:%.3fW",sensorData.power);

    lcd.setCursor(10, 2);
    lcd.printf("BL:%.1f",sensorData.light[2]);
    
    // Dòng 4: Cảm biến ánh sáng BR
    lcd.setCursor(10, 3);
    lcd.printf("BR:%.1f", sensorData.light[3]);
}

void setup() {
    // Khởi tạo Serial và I2C
    Serial.begin(115200);
    Wire.begin();
    
    // Khởi tạo UART2
    Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
    
    // Khởi tạo LCD
    lcd.init();
    lcd.backlight();
    lcd.clear();
    
    // Kết nối WiFi và Blynk
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password);
    
    // Thiết lập timer để gửi dữ liệu mỗi 1 giây
    timer.setInterval(1000L, sendDataToBlynk);
}

void loop() {
    Blynk.run();
    timer.run();
    
    static String inputString = "";
    
    while(Serial2.available()) {
        char c = Serial2.read();
        inputString += c;
        
        if(inputString.endsWith("\n\n")) {
            Serial.println(inputString);
            parseData(inputString);
            updateLCD();
            inputString = "";
        }
    }
}
