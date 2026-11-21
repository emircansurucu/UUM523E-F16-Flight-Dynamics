# Midterm Project: Trimli Uçuş, Kararlılık Parametreleri ve Dinamik Modlar

UUM523E Advanced Flight Dynamics, İTÜ, Güz 2026 — Grup 2
Dilan Gelturan, Emirhan Oskay, Emircan Sürücü, Berat Alku

HW-1 modeli ve HW-2 trim/doğrusallaştırma için bkz. [README.md](README.md).

## Genel tanım

Midterm, HW-1 Simulink modelini ve HW-2 fonksiyonlarını kullanarak üç trim koşulunu inceliyor. Üçü de 0.6 Mach, 10 000 ft ve γ = 0:

| Koşul | Uçuş | x_cg | HW-2 durumu |
|:-:|---|:-:|:-:|
| i | Kanatlar düz | 0.25c̄ | 1 |
| ii | 60° yatışlı düz dönüş, β = 0 | 0.25c̄ | 4 |
| iii | Kanatlar düz | 0.35c̄ | 6 |

- **Problem 1:** Doğrusal olmayan model trimden başlatılıyor, kumandalar trimde sabit tutuluyor. i ve iii 60 s, ii bir tam tur (360°) koşturuluyor.
- **Problem 2:** Rüzgâr ekseni A matrisinden kararlılık parametreleri ve boyutsuz türevler çıkarılıyor. Yuvarlanma ve sapma satırlarındaki I_xz bağlaşımı ayrıştırılıyor; X_u ile X_Tu motor modeli kullanılarak ayrılıyor.
- **Problem 3:** Tam 8×8 A matrisinin ve A11, A22 bloklarının modları hesaplanıyor.

**LEF.** Midterm metni Problem 2 için hücum kenarı flapının sabit tutulmasını istiyor, bu yüzden P2'de δlef trim değerinde sabit. HW-2 geri bildiriminde ise doğrusallaştırmanın LEF tarifesiyle yeniden yapılması istendi; P3 modları bu tarifeli matristen hesaplanıyor. İki matris de `linearize_F16` ile elde ediliyor: `A_fix` (`'sabit'`) ve `A_sch` (`'tarife'`).

## Dosyalar

| Dosya / klasör | İçerik |
|---|---|
| `run_MT.m` | Midterm'in bütün sonuçlarını üretir: trim, iki doğrusal model, P3 modları, P2 türevleri, P1 benzetimleri, grafikler ve sonuç dosyası. |
| `MT_Group2.mat` | Sonuç dosyası; yalnızca ödevin istediği 10 değişkeni içerir (aşağıda). |
| `figures/MT_*.png` | Benzetim grafikleri (`MT_Sim_i/ii/iii`), dönüşün yer izi (`MT_Track_ii`), özdeğerler (`MT_Eigenvalues`), modelin kök seviyesi (`MT_RootModel`). |
| `report/` | AIAA formatında rapor (İngilizce, Overleaf'te derlenir): `main.tex`, `tables/`, `figures/`. |
| `tr_report/` | Raporun Türkçe kopyası, aynı yapıda. |
| `presentation/` | Sunum, İngilizce ve Türkçe (`.pptx` ve `.pdf`). |

`run_MT` yeni bir model kurmuyor: `F16_6DOF.slx`'i ve HW-2'nin `trim_F16`, `linearize_F16`, `lefTarife`, `f16_eval`, `f16_modelInit` fonksiyonlarını kullanıyor. Benzetimler `sim_F16` ile koşturuluyor.

## Çalıştırma

Gereksinim: MATLAB ve Simulink R2023a veya daha yeni bir sürüm, `fmincon` için Optimization Toolbox.

1. MATLAB'da bu klasörü açın (Current Folder).
2. Komut penceresine `run_MT` yazın.

Script sırasıyla şunları yapar:

1. Üç koşulu trim eder; her biri için `A_sch` (LEF tarifeli) ve `A_fix` (LEF sabit) rüzgâr ekseni matrislerini hesaplar. `A_sch`'i `HW2_Group2.mat` ile karşılaştırır (fark 0).
2. Problem 1: üç benzetimi koşturur ve grafikleri kaydeder.
3. Problem 2: `A_fix`'ten parametreleri ve türevleri hesaplar; X_Tu için motor alt sistemini tek başına çalıştırır.
4. Problem 3: `A_sch`'in tam ve ayrık modlarını bulur ve `MT_Eigenvalues.png`'yi çizer.
5. `MT_Group2.mat` dosyasını kaydeder.

Toplam süre yaklaşık 2 dakika. `F16_6DOF.slx` bu sırada değiştirilmez.

## Sonuç dosyası (`MT_Group2.mat`)

Bütün değerler SI birimlerinde, açılar rad.

| Değişken | Boyut | İçerik |
|---|---|---|
| `Sim_i`, `Sim_iii` | 6001 × 17 | 0.01 s aralıkla: t, V_T, α, β, p, q, r, φ, θ, ψ, x_N, y_E, h, δT, δh, δa, δr |
| `Sim_ii` | 7306 × 17 | Aynı sütunlar, bir tam tur (73.05 s) |
| `StabParams` | 3 × 11 | X_u, X_Tu, M_u, M_α, M_q, L_β, N_β, L_p, N_p, L_r, N_r (satırlar i, ii, iii) |
| `StabDerivs` | 3 × 10 | C_Du, C_mu, C_mα, C_mq, C_lβ, C_nβ, C_lp, C_np, C_lr, C_nr (1/rad) |
| `Eig_Full` | 8 × 3 | Tam modelin özdeğerleri |
| `Eig_Lon`, `Eig_Lat` | 4 × 3 | A11 ve A22'nin özdeğerleri |
| `Modes_Full`, `Modes_Dec` | 5 × 6 × 3 | Satırlar SP, fugoid, yuvarlanma, spiral, Hollanda yuvarlanması; sütunlar ω_n, ζ, periyot, τ, t_yarı, t_2kat (uygulanmayanlar NaN) |

## Sonuçların özeti

Ayrıntılı tablolar ve yorumlar raporda.

**Problem 1 — benzetim**
- **i:** Trim korunuyor. 60 s'de |ΔV_T| < 4·10⁻⁴ m/s, irtifa değişimi yaklaşık 1 cm. Kalan küçük harekette fugoid (≈ 88 s) ve sönümlenen Hollanda yuvarlanması görülüyor.
- **ii:** Tam tur 73.05 s sürüyor (trimden 2π/ψ̇ = 73.04 s). Dönüş yarıçapı 2289 m (teorik 2285 m), irtifa değişimi 9 cm.
- **iii:** Durumlar salınmadan yavaşça trimden uzaklaşıyor (60 s'de Δθ ≈ 3.9·10⁻³ derece, büyüme hızı ≈ 0.04 1/s). Trim kararsız bir denge noktası.

**Problem 2 — statik kararlılık (LEF sabit)**

| [1/rad] | i | ii | iii |
|---|:-:|:-:|:-:|
| C_mα | −0.230 | −0.250 | **+0.156** |
| C_lβ | −0.102 | −0.134 | −0.098 |
| C_nβ | 0.219 | 0.213 | 0.181 |
| C_mq | −8.22 | −8.37 | −5.23 |

- i ve ii üç eksende de statik kararlı. iii'te uçak yunuslamada kararsız (nötr nokta ≈ 0.31c̄).
- İtki hız sönümüne katkı veriyor: i'de X_u + X_Tu'nun yaklaşık %23'ü X_Tu'dan geliyor.
- Düz uçuşta C_Du ve C_mu sayısal olarak sıfır, çünkü veri Mach'a bağlı değil.
- ii'de trim açısal hızları A'ya eylemsizlik terimleri ekliyor; C_np'nin işareti bu yüzden değişiyor.

**Problem 3 — dinamik modlar (LEF tarifeli, tam 8×8 model)**

| Mod | i | ii | iii |
|---|:-:|:-:|:-:|
| Kısa periyot | ω_n 2.85, ζ 0.45 | ω_n 2.96, ζ 0.44 | −1.978, −0.075 (gerçel) |
| Fugoid | T 88 s, ζ 0.10 | T 57 s, ζ 0.10 | **kararsız:** T 76 s, t_2kat 20.5 s |
| Yuvarlanma | τ 0.29 s | τ 0.32 s | τ 0.29 s |
| Spiral | τ 89 s | τ 116 s | τ 76 s |
| Hollanda yuv. | ω_n 3.44, ζ 0.12 | ω_n 3.69, ζ 0.14 | ω_n 3.12, ζ 0.12 |

- iii'te kısa periyot iki gerçel köke ayrılıyor; yavaş kök fugoidle bağlaşıp kararsız bir salınım oluşturuyor. LEF sabit olsaydı +0.787 1/s'lik hızlı bir ıraksama olurdu (t_2kat 0.88 s). LEF tarifesi α'ya geri besleme gibi davranarak bunu yavaş bir ıraksamaya çeviriyor.
- Düz uçuşta (i, iii) tam model ile ayrık bloklar aynı sonucu veriyor. Dönüşte (ii) fugoid ve spiral bağlaşıyor: ayrık bloklar fugoid periyodunu 111 s, tam model 57 s veriyor. Dönüşte tam model kullanılmalı.

## Önceki ödevlere göre düzeltmeler

Rapor önceki ödevlerdeki düzeltmelerin listelenmesini istiyor; ayrıntılar raporun II.C ve III. bölümlerinde.

- **HW-1 geri bildirimi:** Eyleyiciler ayrı alt sistemlerde, g(h) Atmosphere'den, m–I–x_cg Mass & Inertia'dan, jiroskopik moment Propulsion'da, kuvvet ve momentler EOM içinde toplanıyor. Bkz. [README](README.md#hw-1-geri-bildirimi-sonrası-düzeltmeler).
- **HW-2 geri bildirimi:**
  - Durum 5'te n_z kısıtı Ders 4'teki gibi f_NLF = n_z − 1/cos μ.
  - Doğrusal modellerde δlef tarifeyi izliyor.
  - Kütle 9295.4 kg.

  - Doğrusallaştırma adımı Ders 5'teki gibi 10⁻³.
  - `run_HW2` ve `run_MT` sadeleştirildi.

  Trim ve matrisler artık HW-2 çözüm kitapçığıyla örtüşüyor; `HW2_Group2.mat` yeniden üretildi.

## Rapor ve sunum

- **Rapor:** Overleaf'te AIAA şablonuyla ("Template for Submissions to AIAA Journals and Conferences", `new-aiaa.cls`) derleniyor. `main.tex` ile `tables/` ve `figures/` klasörleri projenin kök dizinine konmalı. Tablolardaki sayılar `MT_Group2.mat` ve `HW2_Group2.mat`'tan alındı.
- **Sunum:** 13 ana slayt ve 4 ek sonuç slaytı; 4 konuşmacıya bölünmüş (HW1 modeli, HW2 ve Problem 1, LEF etkisi ve Problem 2, Problem 3 ve sonuçlar).
