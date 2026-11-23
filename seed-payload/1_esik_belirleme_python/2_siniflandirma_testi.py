# -*- coding: utf-8 -*-
"""
ADIM 2 - ESIK DEGERLERIYLE SINIFLANDIRMA VE DOGRULUK TESTI
Bitirme Projesi: Otonom Tohum Dagitim Sistemi

Esik degerleri (S_ASFALT_ESIK, H_TOPRAK_*, H_CIMEN_*), 1_kesif_analizi.py
ciktisindaki istatistiklere ve 1_kesif_histogram.png grafigine bakilarak
secildi, ardindan bu dosyadaki testle kucuk duzeltmeler yapildi.
Son degerlerle 15 fotografin 14'u dogru siniflandirildi (%93.3).

"""

import cv2
import numpy as np
import glob
import os
import sys

# ESIK DEGERLERI
# 1_kesif_analizi.py ciktisina (OLCUM 1 ve OLCUM 2 tablolarina ve
# 1_kesif_histogram.png grafigine) bakilarak elle secildi.

S_ASFALT_ESIK = 18      # Saturation bu degerin ALTINDAYSA -> asfalt/beton
H_TOPRAK_MIN, H_TOPRAK_MAX = 10, 30   # Hue bu aralikta -> toprak
H_CIMEN_MIN, H_CIMEN_MAX = 28, 90     # Hue bu aralikta -> cimen/bitki

SINIFLAR = ["toprak", "cimen_bitki", "asfalt_beton"]


def hsv_oku(path):
    """Bir fotografi okuyup orta bolgesinin (%25-%75) H, S, V kanallarini dondurur."""
    img = cv2.imread(path)
    h, w = img.shape[:2]
    y0, y1 = int(h * 0.25), int(h * 0.75)
    x0, x1 = int(w * 0.25), int(w * 0.75)
    crop = img[y0:y1, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)
    gecerli = (V > 20) & (V < 250)
    return H[gecerli], S[gecerli]


def siniflandir(path):
    """
    piksellerin yuzde kacinin hangi sinifin kurallarina uydugunu
    sayar, en yuksek orana sahip sinifi secer."""
    H, S = hsv_oku(path)
    toplam = len(H)

    asfalt_mask = S < S_ASFALT_ESIK
    toprak_mask = (S >= S_ASFALT_ESIK) & (H >= H_TOPRAK_MIN) & (H <= H_TOPRAK_MAX)
    cimen_mask  = (S >= S_ASFALT_ESIK) & (H >= H_CIMEN_MIN)  & (H <= H_CIMEN_MAX)

    oranlar = {
        "asfalt_beton": asfalt_mask.sum() / toplam,
        "toprak": toprak_mask.sum() / toplam,
        "cimen_bitki": cimen_mask.sum() / toplam,
    }
    tahmin = max(oranlar, key=oranlar.get)
    return tahmin, oranlar


print("=" * 70)
print("SINIFLANDIRMA TESTI")
print(f"  S_ASFALT_ESIK={S_ASFALT_ESIK}, "
      f"TOPRAK=[{H_TOPRAK_MIN}-{H_TOPRAK_MAX}], "
      f"CIMEN=[{H_CIMEN_MIN}-{H_CIMEN_MAX}]")
print("=" * 70)

KLASOR = sys.argv[1] if len(sys.argv) > 1 else "veri"
UZANTILAR = (".jpg", ".jpeg", ".png")
print(f"Klasor: {KLASOR}/")

dogru, toplam = 0, 0
karisiklik = {g: {t: 0 for t in SINIFLAR} for g in SINIFLAR}

print(f"{'Dosya':<16}{'Gercek':<14}{'Tahmin':<14}{'Oranlar (T/C/A)':<10}  Sonuc")
print("-" * 80)
for gercek in SINIFLAR:
    dosyalar = sorted(glob.glob(os.path.join(KLASOR, gercek, "*")))
    for path in dosyalar:
        if path.lower().endswith(".heic"):
            print(f"{os.path.basename(path):<16}ATLANDI: HEIC okunamaz, JPG'ye cevir")
            continue
        if not path.lower().endswith(UZANTILAR):
            continue
        if cv2.imread(path) is None:
            print(f"{os.path.basename(path):<16}ATLANDI: dosya okunamadi")
            continue
        tahmin, oranlar = siniflandir(path)
        toplam += 1
        ok = (tahmin == gercek)
        dogru += ok
        karisiklik[gercek][tahmin] += 1
        isim = os.path.basename(path)
        print(f"{isim:<16}{gercek:<14}{tahmin:<14}"
              f"T:{oranlar['toprak']:.2f} C:{oranlar['cimen_bitki']:.2f} "
              f"A:{oranlar['asfalt_beton']:.2f}  "
              f"{'DOGRU' if ok else '>>> YANLIS'}")

print("-" * 80)
if toplam == 0:
    sys.exit(f"{KLASOR}/ altinda fotograf bulunamadi (toprak/, cimen_bitki/, asfalt_beton/).")
print(f"\nTOPLAM DOGRULUK: {dogru}/{toplam} = %{100*dogru/toplam:.1f}\n")

print("Karisiklik matrisi (satir=gercek sinif, sutun=tahmin edilen sinif):")
print(f"{'':<14}" + "".join(f"{s:<14}" for s in SINIFLAR))
for g in SINIFLAR:
    print(f"{g:<14}" + "".join(f"{karisiklik[g][t]:<14}" for t in SINIFLAR))

if os.path.basename(os.path.normpath(KLASOR)) == "veri":
  print("\nNOT: Bu test, esikleri BULMAK icin kullandigimiz 15 fotografi")
  print("tekrar test etmek icin kullaniyor")
else:
  print("\nNOT: Bu klasordeki fotograflar esik secerken KULLANILMADI;")
