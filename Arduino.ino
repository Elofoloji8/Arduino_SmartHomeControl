// ==============================================================================
//           TÜM BİLEŞENLER İÇİN GENEL KÜTÜPHANE İÇERİ AKTARMALARI
// ==============================================================================
#include <DHT.h> 
#include <LiquidCrystal_I2C.h> 
#include <Servo.h>
#include <SPI.h> 
#include <MFRC522.h> 
#include <SoftwareSerial.h>
#include <stdint.h>


//--------------------------UART ------------------------
// Arduino'nun RX pini 2'ye, TX pini 3'e bağlıdır. 
SoftwareSerial espSerial(2, 3); 

// ==============================================================================
//                         GENEL PIN TANIMLAMALARI (Arduino UNO için)
// ==============================================================================
#define FAN_PIN 5// Fan PWM kontrol pini (D5)
#define DHT_PIN 8// DHT11 veri pini (D8)
#define BUTON_PIN 7// Buton giriş pini (D7 - Zil)
#define BUZZER_PIN 4// Bazır (Buzzer) pini (A1)
#define YAGMUR_SENSOR_PIN A0// Yağmur sensörü dijital/analog pini (A0)
#define YAGMUR_SERVO_PIN 9// Yağmur kapısı pini (D9)
#define SS_PIN 10// RFID SDA pini (D10)
#define RST_PIN 6// RFID RST pini (D6)

// ==============================================================================
//                            NESNE TANIMLAMALARI
// ==============================================================================
LiquidCrystal_I2C lcd(0x27, 16, 2); 
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
MFRC522 rfid(SS_PIN, RST_PIN);
Servo yagmur_servo; 

// ** Fan Hız Seviyeleri ve Eşikleri **
const int FAN_DURGUN = 0;
const int FAN_YAVAS = 70;
const int FAN_ORTA = 150;
const int FAN_HIZLI = 255;

const float ESKIK_YAVAS = 20.0;
const float ESKIK_ORTA = 25.0;
const float ESKIK_HIZLI = 30.0;

// ** Servo Açıları ve Yağmur Sensörü **
const int YAGMUR_KAPALI_ACI = 0;
const int YAGMUR_ACIK_ACI = 90;
const int KURU_ESIK_DEGERI = LOW; 

// ** Yetkili RFID ID'leri **
byte yetkiliUIDs[][4] = {
{0x7B, 0xFF, 0x25, 0x03}, // Ayşegül
{0x36, 0x6A, 0x0C, 0x01}// Elif
};
const int UID_SAYISI = 2;

// ** Global Durum Değişkenleri **
int yetkiliID = -1; 
float currentTemperature = 0;
String fanYaz = "DURGUN";
int fanDurumDeger = FAN_DURGUN; 
int fanHizSeviyesi = 0;
int sonYagmurSensorDurum = -1; 
int sonYagmurServoAci = -1; 

// ** ESP32'den gelen kontrol değerleri **
int gelenFanKontrolAyar = 1; 
int gelenFanPWM = 0;
int gelenBuzzerPWM = 0;// ESP32'den gelen Buzzer PWM değeri
int gelenServoOtomatik = 1; 
int gelenServoAci = YAGMUR_KAPALI_ACI; 


// ==============================================================================
//                            VERİ PAKETİ YAPISI (SENKRONİZE EDİLDİ)
// ==============================================================================
struct __attribute__((packed)) VeriPaketi {
  uint8_t header;        // 0xAA
  int16_t sicak_x10;     // sıcaklık * 10
  uint8_t fanSeviyesi;   // 0-3
  int yagmurDurumu;
  int camasirlikDurumu; 
};

// ==============================================================================
//                          ZAMANLAYICI VE LCD DURUM
// ==============================================================================
unsigned long sonDHTOkumaZamani = 0;
const long DHT_ARALIGI = 2000; 
enum ScreenState { TEMPERATUR_FAN, RFID_MESSAGE };
ScreenState currentScreen = TEMPERATUR_FAN; 
unsigned long messageEndTime = 0;
const unsigned long RFID_MESSAGE_DURATION = 3500; 

// ==============================================================================
//                          YARDIMCI FONKSİYONLAR
// ==============================================================================

bool kartlariKarsilastir(byte* kartUID) {
    if (rfid.uid.size != 4) { 
        yetkiliID = -1;
        return false;
    }
    for (int i = 0; i < UID_SAYISI; i++) {
        bool eslesti = true;
        for (byte j = 0; j < 4; j++) {
            if (kartUID[j] != yetkiliUIDs[i][j]) {
                eslesti = false;
                break;
            }
        }
        if (eslesti) {
            yetkiliID = i + 1; 
            return true;
        }
    }
    yetkiliID = -1; 
    return false;
}

// ** FAN OTOMATİK/MANUEL KONTROL FONKSİYONU DÜZELTİLDİ **
void sicaklikVeFanKontrol() {
    float sicaklik = dht.readTemperature();
    
    if (isnan(sicaklik)) {
        fanDurumDeger = FAN_DURGUN; fanYaz = "HATA"; fanHizSeviyesi = 0; 
        currentTemperature = 0; 
        analogWrite(FAN_PIN, fanDurumDeger); 
        return;
    }
    currentTemperature = sicaklik;

    if (gelenFanKontrolAyar == 1) { // 1: OTOMATİK KONTROL (Sıcaklığa bağlı)
        if (currentTemperature >= ESKIK_HIZLI) {
            fanDurumDeger = FAN_HIZLI; fanYaz = "Oto:HIZLI"; fanHizSeviyesi = 3;
        } else if (currentTemperature >= ESKIK_ORTA) {
            fanDurumDeger = FAN_ORTA; fanYaz = "Oto:ORTA"; fanHizSeviyesi = 2;
        } else if (currentTemperature >= ESKIK_YAVAS) {
            fanDurumDeger = FAN_YAVAS; fanYaz = "Oto:YAVAS"; fanHizSeviyesi = 1;
        } else {
            fanDurumDeger = FAN_DURGUN; fanYaz = "Oto:DURGUN"; fanHizSeviyesi = 0;
        }
    } else { // 0: MANUEL KONTROL (ESP32'den gelen PWM'e bağlı)
        fanDurumDeger = gelenFanPWM; 
        fanHizSeviyesi = 4;// Manuel modu belirtmek için 4 gönder
        fanYaz = String("Man:") + gelenFanPWM;
    }
    analogWrite(FAN_PIN, fanDurumDeger);
    Serial.print("🌡️ Sicaklik: ");
    Serial.println(sicaklik);
}

int hedefAci;
int yagmurDeger;
void yagmurVeServoKontrol() {

    yagmurDeger = digitalRead(YAGMUR_SENSOR_PIN); 
  
    if(yagmurDeger != KURU_ESIK_DEGERI) {
            hedefAci = YAGMUR_ACIK_ACI; // Yağışlı ise aç
        } else { 
            hedefAci = YAGMUR_KAPALI_ACI; // Kuru ise kapat
        }
    if (yagmur_servo.read() != hedefAci) {
        yagmur_servo.write(hedefAci);
    }

  
}

void lcdGuncelle() {
    if (currentScreen == RFID_MESSAGE && millis() >= messageEndTime) {
        currentScreen = TEMPERATUR_FAN;
        lcd.clear();
    }

    if (currentScreen == TEMPERATUR_FAN) {
        lcd.setCursor(0, 0);
        lcd.print("Sicaklik: ");
        lcd.print(currentTemperature, 1);
        lcd.print("C "); 

        lcd.setCursor(0, 1);
        lcd.print("Fan: ");
        lcd.print(fanYaz);
        if (gelenFanKontrolAyar == 0) lcd.print(" (M)");
        lcd.print(" "); 
    }
}

// ------------------------------------ SENSÖR VERİLERİNİ GÖNDERME (ARDUINO -> ESP32) ------------------------------------
#include <stdint.h>

void uartVeriGonder() {
  VeriPaketi paket;

  paket.header = 0xAA;
  paket.sicak_x10 = (int16_t)(currentTemperature * 10.0);
  paket.fanSeviyesi = (uint8_t)fanHizSeviyesi;
  paket.camasirlikDurumu = hedefAci;
  paket.yagmurDurumu = yagmurDeger;

  espSerial.write((uint8_t*)&paket, sizeof(paket));

  Serial.print("📤 UART -> ");
  Serial.print(paket.sicak_x10);
  Serial.print(" | Fan:");
  Serial.println(paket.fanSeviyesi);
  Serial.print("yağmur durumu: ");
  Serial.println(paket.)
}

// ==============================================================================
//                                  SETUP
// ==============================================================================
void setup() {
    pinMode(FAN_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTON_PIN, INPUT_PULLUP); 
    pinMode(YAGMUR_SENSOR_PIN, INPUT); 

    analogWrite(BUZZER_PIN, 0); 
    analogWrite(FAN_PIN, FAN_DURGUN);

    Serial.begin(9600);
    espSerial.begin(9600); 

    dht.begin();
    lcd.init();
    lcd.backlight();
    
    SPI.begin();
    rfid.PCD_Init();

    yagmur_servo.attach(YAGMUR_SERVO_PIN);
    yagmur_servo.write(YAGMUR_KAPALI_ACI); 

    sicaklikVeFanKontrol(); 
    yagmurVeServoKontrol();
    sonDHTOkumaZamani = millis();
    uartVeriGonder();
    
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("SISTEM HAZIR"); 
}

// ==============================================================================
//                                   LOOP
// ==============================================================================
void loop() {
    unsigned long currentMillis = millis();

    // 1. SENSÖR OKUMA VE KONTROL
    if (currentMillis - sonDHTOkumaZamani >= DHT_ARALIGI) {
        sicaklikVeFanKontrol(); 
        yagmurVeServoKontrol();
        uartVeriGonder(); 
        sonDHTOkumaZamani = currentMillis;
    }

    // 3. BUTON Kontrolü (Zil olarak çalışır)
    // ESP32'den gelen alarm (gelenBuzzerPWM > 0) yoksa ve butona basıldıysa zil çal
    if (gelenBuzzerPWM == 0 && digitalRead(BUTON_PIN) == HIGH) { 
        analogWrite(BUZZER_PIN, 255); 
    } else if (gelenBuzzerPWM == 0) {
        // Buton serbest bırakıldıysa ve ESP32 alarmı yoksa kapat
        analogWrite(BUZZER_PIN, 0); 
    }

  /*  // 4. RFID ve LCD Kontrolü
    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
        currentScreen = RFID_MESSAGE;
        bool yetkili = kartlariKarsilastir(rfid.uid.uidByte);
        if (yetkili) { 
            lcd.clear();
            lcd.setCursor(0, 0); lcd.print("HOS GELDINIZ!");
            lcd.setCursor(0, 1); lcd.print(yetkiliID == 1 ? "ELIF" : "BUSE");
            // Yetkili girişte Buzzer'ı manuel olarak kontrol et
            analogWrite(BUZZER_PIN, 255); delay(200); analogWrite(BUZZER_PIN, 0); 
        } else {
            lcd.clear();
            lcd.setCursor(0, 0); lcd.print("YETKISIZ KART");
            lcd.setCursor(0, 1); lcd.print("Giris Reddedildi");
            for(int i=0; i<3; i++) { analogWrite(BUZZER_PIN, 255); delay(100); analogWrite(BUZZER_PIN, 0); delay(100); }
        }
        messageEndTime = currentMillis + RFID_MESSAGE_DURATION;
        rfid.PICC_HaltA(); 
    }*/
    
    // 4. RFID ve LCD Kontrolü
    if (rfid.PICC_IsNewCardPresent()) 
    { // Sadece kart var mı kontrolü
        Serial.println("Yeni Kart Algılandı!");
        if (rfid.PICC_ReadCardSerial())
        { // Seri numarasını okumayı dene
             Serial.println("Kart Seri Numarası Başarıyla Okundu.");
             // ...
             // KartıKarsilastir fonksiyonu çağrılır
             // ...
        } else {
             Serial.println("Kart Seri Numarası Okunamadı!");
        }
        // rfid.PICC_HaltA(); zaten okunsa da okunmasa da çağrılabilir.

    }

    // 5. LCD Durum Güncellemesi
    lcdGuncelle(); 
}
