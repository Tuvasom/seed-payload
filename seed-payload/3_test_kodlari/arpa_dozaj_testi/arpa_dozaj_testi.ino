/*
  ARPA BIRAKMA (DOZAJ) KALIBRASYON TESTI - Arduino Nano
  Bitirme Projesi: Otonom Tohum Dagitim Sistemi

  KOMUTLAR:
    2  -> 200 ms        3  -> 300 ms        4  -> 400 ms
    5  -> 500 ms        6  -> 600 ms        8  -> 800 ms
    a  -> 1000 ms       b  -> 1200 ms       c  -> 1500 ms   (uzun sureler)
    k  -> kapiyi kapat 
    t  -> kapiyi tam ac / kapat testi (aci kontrolu)
    x5 -> son secilen sureyle arka arkaya 5 acilis

*/

#include <Servo.h>   

const int SERVO_PIN  = 9;     // servo sinyal pini
const int KAPALI_ACI = 130;     // kapi tam kapali
const int ACIK_ACI   = 0;   // kapi tam acik (olcumle bulundu)
const unsigned long KAPANMA_BEKLEME_MS = 400;  // kapinin kapanmasi icin
const unsigned long TEKRAR_ARASI_MS    = 1500; // x5 modunda acilislar arasi

Servo kapi;
unsigned long sonSure = 0;    // son secilen acik kalma suresi (ms)
unsigned int acilisNo = 0;    // toplam acilis sayaci

void kapiyiAc(unsigned long sureMs) {
  acilisNo++;
  Serial.print("#");
  Serial.print(acilisNo);
  Serial.print("  kapi ");
  Serial.print(sureMs);
  Serial.println(" ms acik...");

  kapi.write(ACIK_ACI);
  delay(sureMs);
  kapi.write(KAPALI_ACI);
  delay(KAPANMA_BEKLEME_MS);

  Serial.println("   kapandi. Tohumu tart ve tabloya yaz.");
}

unsigned long rakamdanSure(char c) {
  switch (c) {
    case '2': return 200;
    case '3': return 300;
    case '4': return 400;
    case '5': return 500;
    case '6': return 600;
    case '8': return 800;
    case 'a': return 1000;
    case 'b': return 1200;
    case 'c': return 1500;
    default:  return 0;
  }
}

void setup() {
  Serial.begin(115200);
  kapi.attach(SERVO_PIN);
  kapi.write(KAPALI_ACI);   // guvenli baslangic: kapi kapali
  delay(500);
  Serial.println();
  Serial.println("=== Arpa dozaj testi hazir ===");
  Serial.println("Sure sec: 2 3 4 5 6 8 (x100 ms), a=1000 b=1200 c=1500 ms | x5: son sureyle 5 acilis | k: kapat | t: aci testi");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == '\n' || c == '\r' || c == ' ') return;

  if (c == 'k') {
    kapi.write(KAPALI_ACI);
    Serial.println("Kapi kapatildi.");
    return;
  }

  if (c == 't') {
    Serial.println("Aci testi: tam ac (130 derece), 1 s bekle, kapat.");
    kapi.write(ACIK_ACI);
    delay(1000);
    kapi.write(KAPALI_ACI);
    delay(KAPANMA_BEKLEME_MS);
    Serial.println("Aci testi bitti (sayaca eklenmedi).");
    return;
  }

  if (c == 'x') {
    // "x5" bekleniyor
    delay(20);
    char n = Serial.available() ? Serial.read() : '5';
    int tekrar = (n >= '1' && n <= '9') ? (n - '0') : 5;
    if (sonSure == 0) {
      Serial.println("Once bir sure sec (ornek: 3), sonra x5 yaz.");
      return;
    }
    Serial.print(tekrar);
    Serial.print(" acilis, her biri ");
    Serial.print(sonSure);
    Serial.println(" ms. Toplami tart, acilis sayisina bol.");
    for (int i = 0; i < tekrar; i++) {
      kapiyiAc(sonSure);
      if (i < tekrar - 1) delay(TEKRAR_ARASI_MS);
    }
    Serial.println("--- seri bitti ---");
    return;
  }

  unsigned long s = rakamdanSure(c);
  if (s == 0) {
    Serial.print("Gecersiz komut: ");
    Serial.println(c);
    return;
  }
  sonSure = s;
  kapiyiAc(s);
}
