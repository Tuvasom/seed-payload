/*
  ESP32-CAM - OTOMATIK TOHUM BIRAKMA (zemin_otomatik_tohum.ino)
  Bitirme Projesi: Otonom Tohum Dagitim Sistemi

  NE YAPAR:
  - Kamera goruntusunun orta bolgesini (%25-%75) HSV'ye cevirir ve her
    pikseli toprak / cimen-bitki / asfalt-beton olarak sayar. Esikler
    Python testinde (1_kesif_analizi.py, 2_siniflandirma_testi.py) secildi:
    S < 18 asfalt, H 10-30 toprak, H 28-90 cimen. Hicbir sinif %15'e
    ulasmazsa karar "belirsiz" olur ve tohum birakilmaz.
  - Siniflandirma, web sayfasindan bagimsiz ayri bir gorevde surekli
    calisir (saniyede yaklasik 6-7 karar).
  - OTOMATIK BIRAKMA: art arda ARDISIK_KARE (2) kez toprak gorulurse kapi,
    toprak oranina gore 600-1500 ms acik kalir; kapandiktan sonra
    BEKLEME_MS (2 s) boyunca yeniden acilmaz.
  - GUVENLIK: kamera karesi okunamazsa kapi kapali kalir. Kapi kapandiktan
    600 ms sonra servoya darbe gonderilmez (SERVO_DINLENDIR).
  - Servo donanim PWM'i (LEDC, 50 Hz, kanal 4) ile surulur.
    OPTOKUPLOR_VAR = true : sinyal PC817 uzerinden (ters darbe)
    OPTOKUPLOR_VAR = false: GPIO12 dogrudan servoya bagli

  WEB ARAYUZU (Wi-Fi "ZeminTest" / 12345678 -> http://192.168.4.1):
  - Canli goruntu, karar ve sinif oranlari, otomatik birakma ac/kapat,
    elle kapi butonlari (200-1500 ms, ACIK TUT, KAPAT), karar gecikmesi
    ve kart durumu (son yeniden baslama nedeni, calisma suresi).
  - Iki HTTP sunucusu: 80 (sayfa ve kontrol), 81 (goruntu akisi). Tek
    sunucuda MJPEG akisi sunucuyu surekli mesgul ettigi icin /classify
    istekleri cevaplanamiyordu; Espressif'in CameraWebServer ornegindeki
    yapi kullanildi.
  - WIFI_ACIK = false: Wi-Fi ve web sayfasi hic baslamaz.

  
*/

#include "esp_camera.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "img_converters.h"
#include <WiFi.h>


// Wi-Fi Access Point ayarlari (ESP32 kendi agini acar)
const char *WIFI_SSID = "ZeminTest";
const char *WIFI_SIFRE = "12345678";   // en az 8 karakter olmali

// AYNI ESIK DEGERLERI
const int S_ASFALT_ESIK = 18;
const int H_TOPRAK_MIN = 10, H_TOPRAK_MAX = 30;
const int H_CIMEN_MIN = 28, H_CIMEN_MAX = 90;
const float MIN_GUVEN = 0.15f;   // bu oranin altinda "belirsiz" denir

// Performans ayarlari
const int PIXEL_STEP = 2;       // merkez bolgede her N pikselde bir orneklem al
const bool RGB565_BYTE_SWAP = false;  // renkler tuhafsa true olmalı

// ---------------------------------------------------------------
// KAPI (SERVO) AYARLARI
// ---------------------------------------------------------------
#define SERVO_PIN 12
const bool OPTOKUPLOR_VAR = true;   // true: PC817 uzerinden (ters darbe) | false: GPIO dogrudan servoya (Test A)
const int KAPALI_ACI = 130;           // kapi tam kapali
const int ACIK_ACI   = 0;         // kapi tam acik (olcumle bulundu)
const int PULSE_MIN_US = 544;       // 0 derece darbe genisligi (Arduino Servo kutuphanesiyle ayni)
const int PULSE_MAX_US = 2400;      // 180 derece darbe genisligi
const int SERVO_KANAL = 4;          // LEDC kanali (kamera kanal 0'i kullaniyor)
const int SERVO_COZUNURLUK = 14;    // bit
const uint32_t MAKS_SURE_MS = 5000; // butonla verilebilecek en uzun sure
// true: kapi kapandiktan 600 ms sonra servoya darbe gonderilmez (servo
// dinlenir). Boylece servo kapali konumda zorlanip surekli akim cekmez.
const bool SERVO_DINLENDIR = true;

// OTOMATIK BIRAKMA VE WI-FI AYARLARI
const bool WIFI_ACIK = true;            // false: Wi-Fi ve web sayfasi kapali (dis mekan / ucus)
const bool OTOMATIK_BASLANGIC = true;   // acilista otomatik birakma
const int  ARDISIK_KARE = 2;            // kac ardisik "toprak" kararindan sonra kapi acilsin
const uint32_t OTO_MIN_SURE_MS  = 600;  // toprak orani en dusukken acik kalma suresi
const uint32_t OTO_MAKS_SURE_MS = 1500; // toprak orani 1.00 iken acik kalma suresi
const uint32_t BEKLEME_MS = 2000;       // kapi kapandiktan sonra yeniden acilmadan once bekleme
const uint32_t SINIF_ARALIK_MS = 50;    // iki siniflandirma arasi bekleme
#define FLAS_LED_PIN     4

// Karar gecikmesi web sayfasinda gormek icin
const bool SURE_OLCUMU = false;

// Sunucu portlari (stream ve control/classify artik AYRI sunucularda)
#define CONTROL_PORT 80   // sayfa (/) + /classify + /gate
#define STREAM_PORT  81   // sadece /stream

// AI-Thinker ESP32-CAM pin yerlesimi (standart)
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ---------------------------------------------------------------
// Paylasilan siniflandirma sonucu (stream gorevi yazar, web
// istekleri okur) - mutex ile korunur.
// ---------------------------------------------------------------
SemaphoreHandle_t resultMutex;
char g_label[16] = "baslatiliyor";
float g_toprak = 0, g_cimen = 0, g_asfalt = 0;

uint8_t *g_rgb565buf = NULL;
size_t g_rgb565bufLen = 0;

httpd_handle_t control_httpd = NULL;  // port 80: index + classify
httpd_handle_t stream_httpd  = NULL;  // port 81: sadece stream

// ---------------------------------------------------------------
// Kapi durumu (web istegi yazar, loop() kapatir)
// ---------------------------------------------------------------
// Karar gecikmesi olcumleri (stream gorevi yazar, /classify okur)
volatile uint32_t g_cozmeMs = 0;       // son JPEG -> RGB565 cozme suresi
volatile uint32_t g_sinifMs = 0;       // son siniflandirma suresi
volatile uint32_t g_aralikMs = 0;      // son iki karar arasi sure
volatile uint32_t g_olcumSayisi = 0;   // kac olcum yapildi
volatile uint32_t g_gecikmeToplam = 0; // (aralik + islem) toplami -> ortalama icin
volatile uint32_t g_gecikmeMaks = 0;   // en kotu (aralik + islem)

volatile bool g_otomatik = OTOMATIK_BASLANGIC;  // otomatik birakma acik mi
volatile uint32_t g_otoAcilis = 0;     // otomatik acilis sayaci
volatile bool g_kameraHatasi = false;  // kare alinamiyor

const char *g_resetNedeni = "?";       // kart en son neden yeniden basladi
volatile bool g_dinlenecek = false;    // kapanistan sonra servo dinlendirilecek
volatile uint32_t g_kapatmaAni = 0;    // kapatma komutunun verildigi an

volatile bool g_kapiAcik = false;      // kapi su an acik mi
volatile bool g_acikTut = false;       // true: KAPAT'a basilana kadar acik
volatile uint32_t g_kapanmaMs = 0;     // millis() bu degere gelince kapat
volatile uint32_t g_acilisNo = 0;      // acilis sayaci
volatile uint32_t g_sonSureMs = 0;     // son istenen sure

// Aciyi servo darbesine cevirip PWM'e yazar. Optokuplor varsa darbe
// ters cevrilir
void servoAciYaz(int aci) {
  uint32_t darbeUs = PULSE_MIN_US + (uint32_t)(PULSE_MAX_US - PULSE_MIN_US) * aci / 180;
  uint32_t tam = (1UL << SERVO_COZUNURLUK);          // 20 ms = tam
  uint32_t duty = darbeUs * tam / 20000UL;
  if (OPTOKUPLOR_VAR) duty = tam - duty;             // ters darbe
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(SERVO_KANAL, duty);
#endif
}

// Servoya darbe gondermeyi keser (servo hatti surekli LOW gorur).
void servoDinlendir() {
  uint32_t duty = OPTOKUPLOR_VAR ? (1UL << SERVO_COZUNURLUK) : 0;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(SERVO_PIN, duty);
#else
  ledcWrite(SERVO_KANAL, duty);
#endif
}

const char *resetNedeniYazi() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "guc verildi (normal)";
    case ESP_RST_BROWNOUT: return "BESLEME DUSTU (brown-out)";
    case ESP_RST_SW:       return "yazilim reseti";
    case ESP_RST_PANIC:    return "yazilim hatasi (panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "watchdog (kod takildi)";
    case ESP_RST_EXT:      return "RST butonu";
    default:               return "bilinmiyor";
  }
}

void servoBaslat() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(SERVO_PIN, 50, SERVO_COZUNURLUK, SERVO_KANAL);
#else
  ledcSetup(SERVO_KANAL, 50, SERVO_COZUNURLUK);
  ledcAttachPin(SERVO_PIN, SERVO_KANAL);
#endif
  servoAciYaz(KAPALI_ACI);   // guvenli baslangic: kapi kapali
  g_kapatmaAni = millis();
  g_dinlenecek = SERVO_DINLENDIR;
}

void kapiKapat() {
  servoAciYaz(KAPALI_ACI);
  g_kapiAcik = false;
  g_acikTut = false;
  g_kapatmaAni = millis();
  g_dinlenecek = SERVO_DINLENDIR;
}

// sureMs > 0: o kadar acik kal, sonra loop() kapatir. sureMs == 0: acik tut.
void kapiAc(uint32_t sureMs) {
  g_dinlenecek = false;
  g_acilisNo++;
  g_sonSureMs = sureMs;
  g_acikTut = (sureMs == 0);
  g_kapanmaMs = millis() + sureMs;
  g_kapiAcik = true;
  servoAciYaz(ACIK_ACI);
  if (sureMs == 0) Serial.printf("#%lu kapi ACIK TUT\n", (unsigned long)g_acilisNo);
  else Serial.printf("#%lu kapi %lu ms acik\n", (unsigned long)g_acilisNo, (unsigned long)sureMs);
}

// RGB -> HSV

void rgbToHsv(uint8_t r, uint8_t g, uint8_t b, uint8_t *H, uint8_t *S, uint8_t *V) {
  float rf = r / 255.0f, gf = g / 255.0f, bf = b / 255.0f;
  float cmax = max(rf, max(gf, bf));
  float cmin = min(rf, min(gf, bf));
  float delta = cmax - cmin;
  float h = 0;
  if (delta > 0.0001f) {
    if (cmax == rf) h = 60.0f * fmodf(((gf - bf) / delta), 6.0f);
    else if (cmax == gf) h = 60.0f * (((bf - rf) / delta) + 2.0f);
    else h = 60.0f * (((rf - gf) / delta) + 4.0f);
  }
  if (h < 0) h += 360.0f;
  float s = (cmax <= 0.0001f) ? 0 : (delta / cmax);
  *H = (uint8_t)(h / 2.0f);        // 0-179 olcegi 
  *S = (uint8_t)(s * 255.0f);
  *V = (uint8_t)(cmax * 255.0f);
}

// Merkez bolgeyi tara, siniflandir, global sonucu guncelle.

void siniflandirKareyiGuncelle(uint8_t *rgb565, int width, int height) {
  int x0 = (int)(width * 0.25f), x1 = (int)(width * 0.75f);
  int y0 = (int)(height * 0.25f), y1 = (int)(height * 0.75f);

  long toplam = 0, toprakSay = 0, cimenSay = 0, asfaltSay = 0;

  for (int y = y0; y < y1; y += PIXEL_STEP) {
    for (int x = x0; x < x1; x += PIXEL_STEP) {
      int idx = (y * width + x) * 2;
      uint8_t b0 = rgb565[idx];
      uint8_t b1 = rgb565[idx + 1];
      uint16_t pixel = RGB565_BYTE_SWAP ? ((b0 << 8) | b1) : ((b1 << 8) | b0);

      uint8_t r5 = (pixel >> 11) & 0x1F;
      uint8_t g6 = (pixel >> 5) & 0x3F;
      uint8_t b5 = pixel & 0x1F;
      uint8_t r = (r5 * 255) / 31;
      uint8_t g = (g6 * 255) / 63;
      uint8_t b = (b5 * 255) / 31;

      uint8_t H, S, V;
      rgbToHsv(r, g, b, &H, &S, &V);
      if (V <= 20 || V >= 250) continue;

      toplam++;
      if (S < S_ASFALT_ESIK) {
        asfaltSay++;
      } else if (H >= H_TOPRAK_MIN && H <= H_TOPRAK_MAX) {
        toprakSay++;
      } else if (H >= H_CIMEN_MIN && H <= H_CIMEN_MAX) {
        cimenSay++;
      }
    }
  }
  if (toplam == 0) toplam = 1;

  float toprakOran = (float)toprakSay / toplam;
  float cimenOran = (float)cimenSay / toplam;
  float asfaltOran = (float)asfaltSay / toplam;

  const char *etiket = "belirsiz";
  float enIyi = 0;
  if (toprakOran > enIyi) { enIyi = toprakOran; etiket = "toprak"; }
  if (cimenOran > enIyi) { enIyi = cimenOran; etiket = "cimen_bitki"; }
  if (asfaltOran > enIyi) { enIyi = asfaltOran; etiket = "asfalt_beton"; }
  if (enIyi < MIN_GUVEN) etiket = "belirsiz";

  xSemaphoreTake(resultMutex, portMAX_DELAY);
  strncpy(g_label, etiket, sizeof(g_label) - 1);
  g_label[sizeof(g_label) - 1] = 0;
  g_toprak = toprakOran;
  g_cimen = cimenOran;
  g_asfalt = asfaltOran;
  xSemaphoreGive(resultMutex);
}

// Web sayfasi

static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Zemin Siniflandirma - Canli</title>
<style>
  body{background:#111;color:#eee;font-family:sans-serif;text-align:center;margin:0;padding:16px;}
  #stream{display:block;max-width:100%;height:auto;margin:0 auto;}
  #info{margin-top:6px;font-size:14px;color:#aaa;}
  h2{font-size:16px;font-weight:normal;color:#ccc;}
  h3{font-size:15px;font-weight:normal;color:#ccc;margin:18px 0 8px 0;}
  #durum{
    margin:0 0 14px 0; padding:18px 10px; border-radius:10px;
    font-size:28px; font-weight:bold; letter-spacing:1px;
    background:#333; color:#999; transition:background .15s, color .15s;
  }
  #durum.aktif{background:#15803d; color:#fff;}
  #durum.pasif{background:#333; color:#999;}
  #etiket{margin-top:10px; font-size:20px; font-weight:bold;}
  button{
    font-size:17px; padding:12px 14px; margin:4px; border:0; border-radius:8px;
    background:#2563eb; color:#fff; min-width:86px;
  }
  button:active{background:#1e40af;}
  button.tut{background:#b45309;}
  button.kapat{background:#b91c1c;}
  button.foto{background:#444;}
  #kapiInfo{margin-top:8px; font-size:15px; color:#ccc; min-height:20px;}
</style></head><body>
<h2>Zemin Siniflandirma (HSV esik tabanli, ESP32-CAM)</h2>
<div id="durum">YUKLENIYOR...</div>
<img id="stream" src="">
<div id="etiket">-</div>
<p id="info">Yukleniyor...</p>

<h3>Otomatik birakma (toprak gorunce kapi kendisi acilir)</h3>
<div><button id="otoBtn" onclick="otoDegistir()">OTOMATIK: ?</button></div>
<div id="otoInfo" style="font-size:15px;color:#ccc;margin-top:4px;">-</div>

<h3>Elle kapi kontrolu (basinca o kadar sure acik kalir, sonra kapanir)</h3>
<div>
  <button onclick="kapi('ms=200')">200 ms</button>
  <button onclick="kapi('ms=400')">400 ms</button>
  <button onclick="kapi('ms=600')">600 ms</button>
  <button onclick="kapi('ms=800')">800 ms</button>
  <button onclick="kapi('ms=1000')">1000 ms</button>
  <button onclick="kapi('ms=1200')">1200 ms</button>
  <button onclick="kapi('ms=1500')">1500 ms</button>
</div>
<div>
  <button class="tut" onclick="kapi('tut=1')">ACIK TUT</button>
  <button class="kapat" onclick="kapi('kapat=1')">KAPAT</button>
</div>
<div id="kapiInfo">Kapi: kapali</div>

<h3>Karar gecikmesi</h3>
<div id="gecikme" style="font-size:15px;color:#ccc;line-height:1.6;">olcum bekleniyor...</div>
<div><button class="foto" onclick="fetch('/classify?sifirla=1')">Olcumu sifirla</button></div>

<h3>Kart durumu</h3>
<div id="kart" style="font-size:15px;color:#ccc;">-</div>

<script>
const img = document.getElementById('stream');
const info = document.getElementById('info');
const durum = document.getElementById('durum');
const etiket = document.getElementById('etiket');
const kapiInfo = document.getElementById('kapiInfo');
const renkler = {toprak:'#c97a3d', cimen_bitki:'#22c55e', asfalt_beton:'#aaaaaa', belirsiz:'#ef4444'};

// Stream ayri bir portta (81) calisiyor - adresi dinamik kur.
img.src = 'http://' + location.hostname + ':81/stream';

let otoDurum = 0;
var otoDegistir = async () => {
  try { await fetch('/gate?oto=' + (otoDurum ? 0 : 1)); } catch(e) {}
};

var kapi = async (sorgu) => {
  try {
    const r = await fetch('/gate?' + sorgu);
    const d = await r.json();
    if (d.hata) { kapiInfo.textContent = 'Hata: ' + d.hata; return; }
    if (d.oto !== undefined) return;
    if (d.durum === 'kapali') kapiInfo.textContent = 'Kapi: kapali';
    else if (d.tut) kapiInfo.textContent = 'Acilis #' + d.no + ': ACIK TUTULUYOR (KAPAT ile kapat)';
    else kapiInfo.textContent = 'Acilis #' + d.no + ': ' + d.ms + ' ms';
  } catch(e) { kapiInfo.textContent = 'baglanti hatasi: ' + e; }
};

var guncelle = async () => {
  try {
    const r = await fetch('/classify');
    const d = await r.json();
    const oran = d[d.label] !== undefined ? d[d.label] : 0;
    etiket.textContent = d.label + ' ' + oran.toFixed(2);
    etiket.style.color = renkler[d.label] || '#ef4444';
    info.textContent = `toprak:${d.toprak.toFixed(2)}  cimen:${d.cimen_bitki.toFixed(2)}  asfalt:${d.asfalt_beton.toFixed(2)}`;

    otoDurum = d.oto;
    const ob = document.getElementById('otoBtn');
    ob.textContent = d.oto ? 'OTOMATIK: ACIK (kapatmak icin bas)' : 'OTOMATIK: KAPALI (acmak icin bas)';
    ob.style.background = d.oto ? '#15803d' : '#555';
    document.getElementById('otoInfo').textContent =
      'otomatik acilis sayisi: ' + d.oto_no + (d.kapi ? '  |  KAPI ACIK' : '');
    document.getElementById('kart').innerHTML =
      'son yeniden baslama nedeni: <b>' + d.reset + '</b><br>calisma suresi: ' + d.calisma_s + ' s';
    document.getElementById('gecikme').innerHTML =
      'son olcum: kare alma+cozme ' + d.cozme_ms + ' ms + siniflandirma ' + d.sinif_ms + ' ms, iki karar arasi ' + d.aralik_ms + ' ms<br>' +
      '<b>en kotu gecikme: ortalama ' + d.ort_ms + ' ms, en yuksek ' + d.maks_ms + ' ms</b> (' + d.n + ' olcum)';

    if (d.servo_aktif) {
      durum.textContent = 'AKTIF — TOPRAK';
      durum.className = 'aktif';
    } else {
      durum.textContent = 'PASIF — TOPRAK DEGIL';
      durum.className = 'pasif';
    }
    if (!d.kapi && kapiInfo.textContent.indexOf('Hata') < 0 && kapiInfo.textContent.indexOf('TUTULUYOR') >= 0)
      kapiInfo.textContent = 'Kapi: kapali';
  } catch(e) { info.textContent = 'baglanti hatasi: ' + e; }
};
setInterval(guncelle, 500);
</script>
</body></html>
)HTML";

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

static esp_err_t classify_handler(httpd_req_t *req) {
  char buf[512];
  char sorgu[32] = "";
  char deger[8] = "";
  // /classify?sifirla=1 -> gecikme istatistiklerini sifirla
  if (httpd_req_get_url_query_str(req, sorgu, sizeof(sorgu)) == ESP_OK &&
      httpd_query_key_value(sorgu, "sifirla", deger, sizeof(deger)) == ESP_OK) {
    g_olcumSayisi = 0; g_gecikmeToplam = 0; g_gecikmeMaks = 0;
  }
  uint32_t n = g_olcumSayisi;
  uint32_t ort = n ? (g_gecikmeToplam / n) : 0;
  xSemaphoreTake(resultMutex, portMAX_DELAY);
  // SERVO TETIKLEME KURALI: zemin "toprak" (cipici/ekilebilir) olarak
  // siniflandirildiginda servo AKTIF (tohum atilir) kabul edilir.
  // Cimen/bitki ve asfalt/beton uzerine tohum atilmaz -> PASIF.
  int servoAktif = (strcmp(g_label, "toprak") == 0) ? 1 : 0;
  snprintf(buf, sizeof(buf),
           "{\"label\":\"%s\",\"toprak\":%.2f,\"cimen_bitki\":%.2f,\"asfalt_beton\":%.2f,\"servo_aktif\":%d,\"kapi\":%d,"
           "\"cozme_ms\":%lu,\"sinif_ms\":%lu,\"aralik_ms\":%lu,\"n\":%lu,\"ort_ms\":%lu,\"maks_ms\":%lu,\"reset\":\"%s\",\"calisma_s\":%lu,\"oto\":%d,\"oto_no\":%lu}",
           g_label, g_toprak, g_cimen, g_asfalt, servoAktif, g_kapiAcik ? 1 : 0,
           (unsigned long)g_cozmeMs, (unsigned long)g_sinifMs, (unsigned long)g_aralikMs,
           (unsigned long)n, (unsigned long)ort, (unsigned long)g_gecikmeMaks,
           g_resetNedeni, (unsigned long)(millis() / 1000),
           g_otomatik ? 1 : 0, (unsigned long)g_otoAcilis);
  xSemaphoreGive(resultMutex);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, strlen(buf));
}

// KAPI ISTEGI:  /gate?ms=800  -> 800 ms acik kal, sonra kapan
//               /gate?tut=1   -> KAPAT'a kadar acik tut
//               /gate?kapat=1 -> hemen kapat

static esp_err_t gate_handler(httpd_req_t *req) {
  char sorgu[64] = "";
  char deger[16] = "";
  char buf[120];
  httpd_req_get_url_query_str(req, sorgu, sizeof(sorgu));

  if (httpd_query_key_value(sorgu, "oto", deger, sizeof(deger)) == ESP_OK) {
    g_otomatik = (atoi(deger) != 0);
    Serial.println(g_otomatik ? "OTOMATIK birakma ACIK" : "OTOMATIK birakma KAPALI");
    snprintf(buf, sizeof(buf), "{\"durum\":\"%s\",\"oto\":%d}", g_kapiAcik ? "acik" : "kapali", g_otomatik ? 1 : 0);
  } else if (httpd_query_key_value(sorgu, "kapat", deger, sizeof(deger)) == ESP_OK) {
    kapiKapat();
    Serial.println("kapi KAPATILDI");
    snprintf(buf, sizeof(buf), "{\"durum\":\"kapali\"}");
  } else if (httpd_query_key_value(sorgu, "tut", deger, sizeof(deger)) == ESP_OK) {
    kapiAc(0);
    snprintf(buf, sizeof(buf), "{\"durum\":\"acik\",\"tut\":1,\"no\":%lu}", (unsigned long)g_acilisNo);
  } else if (httpd_query_key_value(sorgu, "ms", deger, sizeof(deger)) == ESP_OK) {
    long ms = atol(deger);
    if (ms < 1 || ms > (long)MAKS_SURE_MS) {
      snprintf(buf, sizeof(buf), "{\"hata\":\"sure 1-%lu ms olmali\"}", (unsigned long)MAKS_SURE_MS);
    } else {
      kapiAc((uint32_t)ms);
      snprintf(buf, sizeof(buf), "{\"durum\":\"acik\",\"tut\":0,\"ms\":%ld,\"no\":%lu}", ms, (unsigned long)g_acilisNo);
    }
  } else {
    snprintf(buf, sizeof(buf), "{\"hata\":\"ms, tut, kapat ya da oto gerekli\"}");
  }
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, strlen(buf));
}

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// SINIFLANDIRMA GOREVI: web sayfasindan bagimsiz, surekli calisir.
// Her turda: kare al -> RGB565'e coz -> siniflandir
// -> gerekiyorsa kapiyi ac.

void sinifGorevi(void *arg) {
  uint32_t tSonKarar = 0;
  int ardisikToprak = 0;
  int hataSayisi = 0;

  while (true) {
    uint32_t t0 = micros();
    camera_fb_t *fb = esp_camera_fb_get();
    bool tamam = false;

    if (fb && g_rgb565buf && fb->format == PIXFORMAT_JPEG &&
        jpg2rgb565(fb->buf, fb->len, g_rgb565buf, JPG_SCALE_NONE)) {
      uint32_t t1 = micros();
      siniflandirKareyiGuncelle(g_rgb565buf, fb->width, fb->height);
      uint32_t t2 = micros();
      tamam = true;

      // --- sure olcumu ---
      g_cozmeMs = (t1 - t0) / 1000;      // kare alma + JPEG cozme
      g_sinifMs = (t2 - t1) / 1000;      // siniflandirma
      if (tSonKarar != 0 && (t2 - tSonKarar) < 5000000UL) {
        g_aralikMs = (t2 - tSonKarar) / 1000;
        // En kotu durum: zemin bir karardan hemen sonra degisirse, bir
        // sonraki karara kadar (aralik) beklenir.
        uint32_t gecikme = g_aralikMs + g_cozmeMs + g_sinifMs;
        g_olcumSayisi++;
        g_gecikmeToplam += gecikme;
        if (gecikme > g_gecikmeMaks) g_gecikmeMaks = gecikme;
      }
      tSonKarar = t2;
      if (SURE_OLCUMU) {
        Serial.printf("kare+cozme %lu ms | siniflandirma %lu ms | iki karar arasi %lu ms\n",
                      (unsigned long)g_cozmeMs, (unsigned long)g_sinifMs, (unsigned long)g_aralikMs);
      }
    }
    if (fb) esp_camera_fb_return(fb);

    if (tamam) {
      hataSayisi = 0;
      g_kameraHatasi = false;

      // --- karari oku ---
      xSemaphoreTake(resultMutex, portMAX_DELAY);
      bool toprak = (strcmp(g_label, "toprak") == 0);
      float oran = g_toprak;
      xSemaphoreGive(resultMutex);

      // --- otomatik birakma ---
      ardisikToprak = toprak ? (ardisikToprak + 1) : 0;
      if (g_otomatik && ardisikToprak >= ARDISIK_KARE && !g_kapiAcik &&
          (millis() - g_kapatmaAni) >= BEKLEME_MS) {
        float k = (oran - MIN_GUVEN) / (1.0f - MIN_GUVEN);
        if (k < 0) k = 0;
        if (k > 1) k = 1;
        uint32_t sure = OTO_MIN_SURE_MS + (uint32_t)(k * (OTO_MAKS_SURE_MS - OTO_MIN_SURE_MS));
        g_otoAcilis++;
        Serial.printf("OTOMATIK: toprak orani %.2f -> ", oran);
        kapiAc(sure);
        ardisikToprak = 0;
      }
    } else {
      // --- GUVENLIK: kare alinamiyorsa tohum birakma ---
      hataSayisi++;
      ardisikToprak = 0;
      if (hataSayisi >= 5) {
        g_kameraHatasi = true;
        if (g_kapiAcik && !g_acikTut) kapiKapat();
        xSemaphoreTake(resultMutex, portMAX_DELAY);
        strncpy(g_label, "KAMERA HATASI", sizeof(g_label) - 1);
        xSemaphoreGive(resultMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }
    vTaskDelay(pdMS_TO_TICKS(SINIF_ARALIK_MS));
  }
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t jpgLen = 0;
  uint8_t *jpgBuf = NULL;
  char partBuf[64];

  res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Kamera kare yakalama basarisiz");
      res = ESP_FAIL;
    } else {
      if (fb->format != PIXFORMAT_JPEG) {
        bool ok = frame2jpg(fb, 80, &jpgBuf, &jpgLen);
        esp_camera_fb_return(fb);
        fb = NULL;
        if (!ok) res = ESP_FAIL;
      } else {
        jpgLen = fb->len;
        jpgBuf = fb->buf;
      }
    }

    if (res == ESP_OK) res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
    if (res == ESP_OK) {
      size_t hlen = snprintf(partBuf, sizeof(partBuf), STREAM_PART, (unsigned)jpgLen);
      res = httpd_resp_send_chunk(req, partBuf, hlen);
    }
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)jpgBuf, jpgLen);

    if (fb) {
      esp_camera_fb_return(fb);
      fb = NULL;
      jpgBuf = NULL;
    } else if (jpgBuf) {
      free(jpgBuf);
      jpgBuf = NULL;
    }
    if (res != ESP_OK) break;
  }
  return res;
}

// ARTIK IKI AYRI SUNUCU: biri (80) index+classify,(81) sadece stream.


void startServer() {
  //Kontrol sunucusu: port 80, index + classify
  httpd_config_t controlConfig = HTTPD_DEFAULT_CONFIG();
  controlConfig.server_port = CONTROL_PORT;
  controlConfig.ctrl_port = CONTROL_PORT;
  controlConfig.max_uri_handlers = 6;

  httpd_uri_t index_uri    = {.uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL};
  httpd_uri_t classify_uri = {.uri = "/classify", .method = HTTP_GET, .handler = classify_handler, .user_ctx = NULL};
  httpd_uri_t gate_uri     = {.uri = "/gate", .method = HTTP_GET, .handler = gate_handler, .user_ctx = NULL};

  if (httpd_start(&control_httpd, &controlConfig) == ESP_OK) {
    httpd_register_uri_handler(control_httpd, &index_uri);
    httpd_register_uri_handler(control_httpd, &classify_uri);
    httpd_register_uri_handler(control_httpd, &gate_uri);
  } else {
    Serial.println("HATA: Kontrol sunucusu (port 80) baslatilamadi");
  }

  // --- Stream sunucusu: port 81
  httpd_config_t streamConfig = HTTPD_DEFAULT_CONFIG();
  streamConfig.server_port = STREAM_PORT;
  streamConfig.ctrl_port = STREAM_PORT;
  streamConfig.max_uri_handlers = 2;

  httpd_uri_t stream_uri = {.uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL};

  if (httpd_start(&stream_httpd, &streamConfig) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  } else {
    Serial.println("HATA: Stream sunucusu (port 81) baslatilamadi");
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  g_resetNedeni = resetNedeniYazi();
  Serial.print("Yeniden baslama nedeni: ");
  Serial.println(g_resetNedeni);

  resultMutex = xSemaphoreCreateMutex();

  // Kart ustundeki flas LED'i kapali tutulur
  pinMode(FLAS_LED_PIN, OUTPUT);
  digitalWrite(FLAS_LED_PIN, LOW);

  if (WIFI_ACIK) {
    // Wi-Fi ONCE acilir: kamera hata verse bile ag gorunur (ariza ayirmak icin)
    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_SIFRE);
    // Verici gucunu dusur: Wi-Fi'in anlik akim tepeleri kuculur.
    WiFi.setTxPower(WIFI_POWER_8_5dBm);
    Serial.print("Wi-Fi agi acildi: ");
    Serial.println(WIFI_SSID);
    Serial.print("Tarayicida ac: http://");
    Serial.println(WiFi.softAPIP());
  } else {
    WiFi.mode(WIFI_OFF);
    Serial.println("Wi-Fi KAPALI (WIFI_ACIK = false).");
  }

  camera_config_t config = {};
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  if (psramFound()) {
    config.frame_size = FRAMESIZE_QVGA;   // 320x240
    config.jpeg_quality = 12;
    config.fb_count = 2;
    config.grab_mode = CAMERA_GRAB_LATEST;   // her zaman en yeni kare (gecikme azalir)
  } else {
    Serial.println("UYARI: PSRAM bulunamadi - daha kucuk kare boyutu kullaniliyor.");
    config.frame_size = FRAMESIZE_QQVGA;  // 160x120
    config.jpeg_quality = 15;
    config.fb_count = 1;
    config.fb_location = CAMERA_FB_IN_DRAM;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera baslatilamadi, hata 0x%x\n", err);
    strncpy(g_label, "KAMERA HATASI", sizeof(g_label) - 1);
    g_kameraHatasi = true;
    g_otomatik = false;                 // kamera yoksa otomatik birakma yok
    if (WIFI_ACIK) startServer();       // sayfa yine acilsin, kapi butonlari calissin
    delay(500);
    servoBaslat();
    return;
  }

  // RGB565 ara tampon: secilen kare boyutuna gore ayir
  int bufW = (config.frame_size == FRAMESIZE_QVGA) ? 320 : 160;
  int bufH = (config.frame_size == FRAMESIZE_QVGA) ? 240 : 120;
  g_rgb565bufLen = (size_t)bufW * bufH * 2;
  g_rgb565buf = (uint8_t *)heap_caps_malloc(g_rgb565bufLen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!g_rgb565buf) {
    g_rgb565buf = (uint8_t *)malloc(g_rgb565bufLen);  // PSRAM yoksa dahili RAM'den dene
  }
  if (!g_rgb565buf) {
    Serial.println("UYARI: Siniflandirma tamponu ayrilamadi - siniflandirma calismayacak.");
  }

  if (WIFI_ACIK) startServer();

  // Servo EN SON baslatilir: acilista Wi-Fi ve kamera akim cekerken servo
  // da hareket ederse besleme duser ve kart surekli yeniden baslar.
  delay(500);
  servoBaslat();   // kapi kapali konuma gelir
  Serial.println("Servo hazir (kapi kapali).");

  // Siniflandirma gorevini baslat (web sayfasindan bagimsiz calisir)
  xTaskCreatePinnedToCore(sinifGorevi, "sinif", 8192, NULL, 2, NULL, 1);
  Serial.println(g_otomatik ? "Otomatik birakma ACIK." : "Otomatik birakma KAPALI.");
}

void loop() {
  // Sureli acilis: zamani gelince kapiyi kapat
  if (g_kapiAcik && !g_acikTut && (int32_t)(millis() - g_kapanmaMs) >= 0) {
    kapiKapat();
    Serial.println("   kapandi");
  }
  // Kapanistan 600 ms sonra servoyu dinlendir
  if (g_dinlenecek && !g_kapiAcik && (millis() - g_kapatmaAni) >= 600) {
    g_dinlenecek = false;
    servoDinlendir();
  }
  delay(2);
}
