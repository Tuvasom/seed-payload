# -*- coding: utf-8 -*-
"""
ADIM 0 - KESIF / ANALIZ KODU
Bitirme Projesi: Otonom Tohum Dagitim Sistemi

Amac: Esik degerlerini (S_ASFALT_ESIK, H_TOPRAK, H_CIMEN ...) BULMADAN
ONCE, elimizdeki fotograflarin Hue (renk tonu) ve Saturation (renk
doygunlugu) degerlerinin GERCEKTE ne oldugunu gormek.

Esik degerlerine bu ciktiya bakarak karar verdim.

"""

import os
import cv2
import numpy as np
import glob
import matplotlib
matplotlib.use("Agg")  
import matplotlib.pyplot as plt

SINIFLAR = ["toprak", "cimen_bitki", "asfalt_beton"]


def hsv_oku(path):
    """Bir fotografi okuyup orta bolgesinin gecerli piksellerinin H ve S degerlerini geri verir"""
    img = cv2.imread(path)
    h, w = img.shape[:2]
    y0, y1 = int(h * 0.2), int(h * 0.8)
    x0, x1 = int(w * 0.2), int(w * 0.8)
    crop = img[y0:y1, x0:x1]
    hsv = cv2.cvtColor(crop, cv2.COLOR_BGR2HSV)
    H, S, V = cv2.split(hsv)
    # Cok karanlik / cok parlak pikselleri at (golge/parlama, guvenilmez)
    gecerli = (V > 20) & (V < 250)
    return H[gecerli], S[gecerli]


# 
# OLCUM 1: Her fotografin Saturation degerleri
#          (amac: asfalt/beton'un digerlerinden Saturation olarak
#          ayrilip ayrilmadigini GORMEK)
# 
print("=" * 70)
print("OLCUM 1: Saturation degerleri (her fotograf icin)")
print("=" * 70)
print(f"{'Sinif':<14}{'Dosya':<16}{'ort.S':>8}{'medyan':>8}"
      f"{'%25':>8}{'%75':>8}{'%90':>8}")
print("-" * 70)
for sinif in SINIFLAR:
    for path in sorted(glob.glob(f"veri/{sinif}/*.jpg")):
        _, S = hsv_oku(path)
        isim = os.path.basename(path)
        p25, p75, p90 = np.percentile(S, [25, 75, 90])
        print(f"{sinif:<14}{isim:<16}{S.mean():8.1f}{np.median(S):8.1f}"
              f"{p25:8.1f}{p75:8.1f}{p90:8.1f}")
    print()

# 
# OLCUM 2: Her fotografin Hue degerleri
#          (amac: toprak ve cimenin Hue olarak ayrilip ayrilmadigini
#          GORMEK - Saturation cok dusukken Hue anlamsiz oldugu icin,
#          sadece Saturation>30 olan pikseller kullanildi)
# 
print("=" * 70)
print("OLCUM 2: Hue degerleri (sadece Saturation > 30 olan pikseller)")
print("=" * 70)
print(f"{'Sinif':<14}{'Dosya':<16}{'ort.H':>8}{'medyan':>8}"
      f"{'%10':>8}{'%90':>8}{'%95':>8}")
print("-" * 70)
for sinif in ["toprak", "cimen_bitki"]:
    for path in sorted(glob.glob(f"veri/{sinif}/*.jpg")):
        H, S = hsv_oku(path)
        mask = S > 30
        Hm = H[mask]
        isim = os.path.basename(path)
        p10, p90, p95 = np.percentile(Hm, [10, 90, 95])
        print(f"{sinif:<14}{isim:<16}{Hm.mean():8.1f}{np.median(Hm):8.1f}"
              f"{p10:8.1f}{p90:8.1f}{p95:8.1f}")
    print()

print("NOT: %95 sutunu bazi cimen fotograflarinda birden yuksek")
print("     cikabilir (kirmizimsi nesneler yuzunden). Bu yuzden")
print("     SADECE medyana degil, yuzdelik dilimlere")
print("     bakmak gerekiyor. bir sonraki script'te esik secerken")
print("     bunu goz onunde bulundurduk.\n")

# OLCUM 3: Histogram grafigi (tum siniflarin Hue/Saturation dagilimi)

print("OLCUM 3: Histogram grafigi olusturuluyor -> 1_kesif_histogram.png")

renkler = {"toprak": "#B5651D", "cimen_bitki": "#2F7D4F", "asfalt_beton": "#555555"}
fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))

for sinif in SINIFLAR:
    hue_all, sat_all = [], []
    for path in glob.glob(f"veri/{sinif}/*.jpg"):
        H, S = hsv_oku(path)
        hue_all.append(H.ravel())
        sat_all.append(S.ravel())
    hue_all = np.concatenate(hue_all)
    sat_all = np.concatenate(sat_all)
    axes[0].hist(hue_all, bins=60, range=(0, 180), alpha=0.5,
                 label=sinif, color=renkler[sinif], density=True)
    axes[1].hist(sat_all, bins=60, range=(0, 255), alpha=0.5,
                 label=sinif, color=renkler[sinif], density=True)

axes[0].set_title("Hue dagilimi (0-179)")
axes[0].set_xlabel("Hue"); axes[0].set_ylabel("Yogunluk (density)"); axes[0].legend()
axes[1].set_title("Saturation dagilimi (0-255)")
axes[1].set_xlabel("Saturation"); axes[1].set_ylabel("Yogunluk (density)"); axes[1].legend()
plt.tight_layout()
plt.savefig("1_kesif_histogram.png", dpi=150)
print("  Kaydedildi: 1_kesif_histogram.png\n")

print("=" * 70)
print("BITTI. Esikler yukaridaki sayilara ve 1_kesif_histogram.png")
print("grafigine bakilarak secildi. Dogruluk testi: 2_siniflandirma_testi.py")
print("=" * 70)
