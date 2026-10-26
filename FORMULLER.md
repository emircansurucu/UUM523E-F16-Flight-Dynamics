# F-16 Formül Kağıdı

[← README](README.md)

UUM523E ders notları (Lecture 1–4), NASA TP-1538 (Ek B) ve Stevens–Lewis–Johnson (App. A.6) esas alınmıştır. Tüm model **SI** birimindedir; tablolar dışında açılar **radyan**, tablo girişleri **derece**dir.

**İçindekiler:** [Birimler](#0-birimler-ve-dönüşümler) · [Eksenler](#1-eksen-takımları-ve-dönüşümler) · [Hareket denklemleri](#2-hareket-denklemleri) · [Kütle ve atalet](#3-kütle-ve-atalet) · [Atmosfer](#4-atmosfer) · [Aerodinamik](#5-aerodinamik-tp-1538) · [Motor ve itki](#6-motor-ve-itki) · [Eyleyiciler](#7-eyleyiciler) · [Statik kararlılık](#8-statik-kararlılık) · [Trim](#9-trim) · [Kontrol listesi](#10-doğrulama-kontrol-listesi)

---

## 0. Birimler ve dönüşümler

| Büyüklük | Dönüşüm |
|---|---|
| Uzunluk | 1 ft = 0.3048 m |
| Kuvvet | 1 lbf = 4.448222 N |
| Kütle | 1 slug = 4.448222 / 0.3048 = 14.5939 kg |
| Atalet | 1 slug·ft² = 1.355818 kg·m² |
| Yoğunluk | 1 slug/ft³ = 515.379 kg/m³ |
| Sıcaklık | T[K] = T[°R] / 1.8 |
| Gaz sabiti | 1 ft²/(s²·°R) = 0.3048² × 1.8 = 0.167225 m²/(s²·K) |

---

## 1. Eksen takımları ve dönüşümler

| Eksen | Tanım |
|---|---|
| **NED** (Dünya-sabit) | $x_N$ kuzey, $y_E$ doğu, $z_D$ aşağı; düz ve dönmeyen Dünya varsayımı |
| **Gövde** (body) | $x_b$ burun, $y_b$ sağ kanat, $z_b$ aşağı; orijin ağırlık merkezinde |
| **Kararlılık** (stability) | Gövde ekseninin $y_b$ etrafında $\alpha$ kadar döndürülmüşü |
| **Rüzgar** (wind) | $x_w$ hava hızı vektörü yönünde; kararlılık ekseninin $z$ etrafında $\beta$ kadar döndürülmüşü |

**NED → gövde** (3-2-1 Euler sırası: $\psi \to \theta \to \phi$):

$$
C_{b/n}=
\begin{bmatrix}
c\theta c\psi & c\theta s\psi & -s\theta\\
s\phi s\theta c\psi - c\phi s\psi & s\phi s\theta s\psi + c\phi c\psi & s\phi c\theta\\
c\phi s\theta c\psi + s\phi s\psi & c\phi s\theta s\psi - s\phi c\psi & c\phi c\theta
\end{bmatrix},
\qquad C_{n/b}=C_{b/n}^{T}
$$

**Hava hızı, hücum ve yanal kayma açıları:**

$$
V_T=\sqrt{u^2+v^2+w^2},\qquad \alpha=\operatorname{atan2}(w,u),\qquad \beta=\arcsin\!\left(\frac{v}{V_T}\right)
$$

$$
u=V_T\cos\alpha\cos\beta,\qquad v=V_T\sin\beta,\qquad w=V_T\sin\alpha\cos\beta
$$

**Gövde ↔ kararlılık ekseni katsayıları** (Problem 2a, $C_L$–$C_D$):

$$
C_L = C_X\sin\alpha - C_Z\cos\alpha,\qquad C_D = -C_X\cos\alpha - C_Z\sin\alpha
$$

---

## 2. Hareket denklemleri

Varsayımlar: rijit gövde, sabit kütle, düz ve dönmeyen Dünya, $xz$ düzleminde simetri ($I_{xy}=I_{yz}=0$).

**Ötelenme dinamiği** (gövde ekseni; $X,Y,Z$ aerodinamik, $T$ itki):

$$
\begin{aligned}
\dot u &= rv - qw - g\sin\theta + (X+T)/m\\
\dot v &= pw - ru + g\sin\phi\cos\theta + Y/m\\
\dot w &= qu - pv + g\cos\phi\cos\theta + Z/m
\end{aligned}
$$

**Dönme dinamiği** (matris biçimi; $h_e$ motor açısal momentumu, $x_b$ yönünde):

$$
I\,\dot{\boldsymbol\omega} = \mathbf M - \boldsymbol\omega\times\left(I\boldsymbol\omega + \mathbf h_e\right),
\qquad
I=\begin{bmatrix} I_{xx} & 0 & -I_{xz}\\ 0 & I_{yy} & 0\\ -I_{xz} & 0 & I_{zz}\end{bmatrix}
$$

Açık biçim (Stevens–Lewis), $\Gamma = I_{xx}I_{zz}-I_{xz}^2$:

$$
\begin{aligned}
\dot p &= (c_1 r + c_2 p)\,q + c_3 L + c_4\,(N + q\,h_e)\\
\dot q &= c_5\,p r - c_6\,(p^2-r^2) + c_7\,(M - r\,h_e)\\
\dot r &= (c_8 p - c_2 r)\,q + c_4 L + c_9\,(N + q\,h_e)
\end{aligned}
$$

| | | |
|---|---|---|
| $c_1=\dfrac{(I_{yy}-I_{zz})I_{zz}-I_{xz}^2}{\Gamma}$ | $c_2=\dfrac{(I_{xx}-I_{yy}+I_{zz})I_{xz}}{\Gamma}$ | $c_3=\dfrac{I_{zz}}{\Gamma}$ |
| $c_4=\dfrac{I_{xz}}{\Gamma}$ | $c_5=\dfrac{I_{zz}-I_{xx}}{I_{yy}}$ | $c_6=\dfrac{I_{xz}}{I_{yy}}$ |
| $c_7=\dfrac{1}{I_{yy}}$ | $c_8=\dfrac{I_{xx}(I_{xx}-I_{yy})+I_{xz}^2}{\Gamma}$ | $c_9=\dfrac{I_{xx}}{\Gamma}$ |

**Dönme kinematiği** (Euler açıları; $\theta=\pm 90^\circ$'de tekil):

$$
\dot\phi = p + \tan\theta\,(q\sin\phi + r\cos\phi),\qquad
\dot\theta = q\cos\phi - r\sin\phi,\qquad
\dot\psi = \frac{q\sin\phi + r\cos\phi}{\cos\theta}
$$

Tekillikten kaçınmak için kuaterniyon: $\dot{\mathbf q} = \tfrac12\,\Omega(\boldsymbol\omega)\,\mathbf q$, her adımda $\lVert\mathbf q\rVert=1$ normalizasyonu.

**Ötelenme kinematiği (navigasyon):**

$$
\begin{bmatrix}\dot x_N\\ \dot y_E\\ \dot z_D\end{bmatrix} = C_{n/b}\begin{bmatrix}u\\ v\\ w\end{bmatrix},
\qquad \dot h = -\dot z_D = u\sin\theta - v\sin\phi\cos\theta - w\cos\phi\cos\theta
$$

**Rüzgar ekseni durumları** ($V_T,\alpha,\beta$ ile çalışmak için):

$$
\dot V_T = \frac{u\dot u + v\dot v + w\dot w}{V_T},\qquad
\dot\alpha = \frac{u\dot w - w\dot u}{u^2+w^2},\qquad
\dot\beta = \frac{\dot v\,V_T - v\,\dot V_T}{V_T^2\cos\beta}
$$

**Yerçekimi kuvveti (gövde ekseni):**

$$
\mathbf F_g = m g\,\begin{bmatrix}-\sin\theta & \sin\phi\cos\theta & \cos\phi\cos\theta\end{bmatrix}^T
$$

---

## 3. Kütle ve atalet

NASA TP-1538 Tablo I değerleri (SI):

| Parametre | İngiliz | SI |
|---|---|---|
| Ağırlık $W$ | 20 500 lbf | 91 188.6 N → $m$ = 9 298.6 kg ($g$ = 9.80665) |
| Kanat alanı $S$ | 300 ft² | 27.8709 m² |
| Kanat açıklığı $b$ | 30 ft | 9.144 m |
| Ortalama aerodinamik veter $\bar c$ | 11.32 ft | 3.4503 m |
| $I_{xx}$ | 9 496 slug·ft² | 12 874.8 kg·m² |
| $I_{yy}$ | 55 814 slug·ft² | 75 673.6 kg·m² |
| $I_{zz}$ | 63 100 slug·ft² | 85 552.1 kg·m² |
| $I_{xz}$ | 982 slug·ft² | 1 331.4 kg·m² |
| Motor açısal momentumu $h_e$ | 160 slug·ft²/s | 216.9 kg·m²/s |
| Referans AM $x_{cg,ref}$ | 0.35 $\bar c$ | 0.35 $\bar c$ |

---

## 4. Atmosfer

Stevens–Lewis–Johnson App. A.6 modelinin SI karşılığı ($h$ metre):

$$
T^* = 1 - k\,h,\qquad
T = \begin{cases} T_0\,T^* & h < h_{tr}\\ T_{tr} & h \ge h_{tr}\end{cases},\qquad
\rho = \rho_0\,(T^*)^{4.14}
$$

$$
a = \sqrt{1.4\,R_a\,T},\qquad M = V_T/a,\qquad \bar q = \tfrac12\rho V_T^2,\qquad p_s = R_p\,\rho\,T
$$

| Sabit | İngiliz (kaynak) | SI |
|---|---|---|
| $k$ | 0.703×10⁻⁵ 1/ft | 2.30643×10⁻⁵ 1/m |
| $T_0$ | 519 °R | 288.333 K |
| $T_{tr}$ | 390 °R | 216.667 K |
| $h_{tr}$ | 35 000 ft | 10 668 m |
| $\rho_0$ | 2.377×10⁻³ slug/ft³ | 1.22506 kg/m³ |
| $R_a$ (ses hızı için) | 1716.3 ft²/(s²·°R) | 287.009 m²/(s²·K) |
| $R_p$ (statik basınç için) | 1715 ft²/(s²·°R) | 286.792 m²/(s²·K) |

> Kaynak modelde $\rho$, $h_{tr}$ üstünde de $T^*$ formülüyle hesaplanır; yalnızca $T$ sabitlenir. Kaynak modelde iki farklı $R$ kullanılır, aynen korunmalıdır.

**HW-1 referans koşulu** (0.4 Mach, 10 000 ft = 3048 m), Tablo 1 sonuçlarını kontrol etmek için:

| $T^*$ | $T$ [K] | $\rho$ [kg/m³] | $a$ [m/s] | $V_T$ [m/s] | $\bar q$ [Pa] | $p_s$ [Pa] | $\bar q S$ [N] | $\bar c/2V_T$ [s] | $b/2V_T$ [s] |
|---|---|---|---|---|---|---|---|---|---|
| 0.9297 | 268.06 | 0.90593 | 328.19 | 131.28 | 7806.3 | 69 646 | 217 569 | 0.013141 | 0.034827 |

---

## 5. Aerodinamik (TP-1538)

### İşaret kuralları ve girişler

| Değişken | Pozitif yön | Aralık (tablo) |
|---|---|---|
| $\delta_h$ yatay stabilatör | burun aşağı | −25° … 25° |
| $\delta_a$ kanatçık | sola yatış | ±21.5° |
| $\delta_r$ dümen | sola sapma | ±30° |
| $\delta_{lef}$ hücum kenarı flapı | hücum kenarı aşağı | 0° … 25° |
| $\delta_{sb}$ hava freni | açılma | 0° … 60° |

- $\alpha$, $\beta$ ve tüm $\delta$'lar tablolara **derece** olarak girer; $p,q,r$ **rad/s** olarak boyutsuzlaştırılır: $\hat p=\frac{pb}{2V_T}$, $\hat q=\frac{q\bar c}{2V_T}$, $\hat r=\frac{rb}{2V_T}$.
- **"lef" artımı**, LEF'in 25°'den 0°'ye **tamamen geri çekilmesiyle** oluşan farktır. Ana tablolar $\delta_{lef}=25^\circ$, "lef" tabloları $\delta_{lef}=0^\circ$ içindir. Bu yüzden çarpan $\left(1-\frac{\delta_{lef}}{25}\right)$'tir.
- **"a20" / "r30"** tabloları $\delta_a=20^\circ$ ve $\delta_r=30^\circ$ için verilmiştir; doğrusal ölçekleme $\delta_a/20$, $\delta_r/30$ ile yapılır.
- Ana tablolar $\alpha\in[-20^\circ,90^\circ]$ (20 nokta), "lef" tabloları $\alpha\in[-20^\circ,45^\circ]$ (14 nokta) aralığındadır, $\beta\in[-30^\circ,30^\circ]$ (19 nokta).

Kısaltmalar: $\;\Lambda \equiv 1-\dfrac{\delta_{lef}}{25}$

### Kuvvet katsayıları

$$
\begin{aligned}
C_{X,t} &= C_X(\alpha,\beta,\delta_h) + \Delta C_{X,lef}\,\Lambda + \Delta C_{X,sb}(\alpha)\frac{\delta_{sb}}{60}
+ \frac{q\bar c}{2V_T}\Big[C_{X_q}(\alpha) + \Delta C_{X_q,lef}(\alpha)\,\Lambda\Big]\\[4pt]
C_{Z,t} &= C_Z(\alpha,\beta,\delta_h) + \Delta C_{Z,lef}\,\Lambda + \Delta C_{Z,sb}(\alpha)\frac{\delta_{sb}}{60}
+ \frac{q\bar c}{2V_T}\Big[C_{Z_q}(\alpha) + \Delta C_{Z_q,lef}(\alpha)\,\Lambda\Big]\\[4pt]
C_{Y,t} &= C_Y(\alpha,\beta) + \Delta C_{Y,lef}\,\Lambda
+ \Big[\Delta C_{Y,\delta_a=20} + \Delta C_{Y,\delta_a=20,lef}\,\Lambda\Big]\frac{\delta_a}{20}
+ \Delta C_{Y,\delta_r=30}\frac{\delta_r}{30}\\
&\quad + \frac{rb}{2V_T}\Big[C_{Y_r}(\alpha) + \Delta C_{Y_r,lef}(\alpha)\,\Lambda\Big]
+ \frac{pb}{2V_T}\Big[C_{Y_p}(\alpha) + \Delta C_{Y_p,lef}(\alpha)\,\Lambda\Big]
\end{aligned}
$$

### Moment katsayıları (AM: $x_{cg}$, $\bar c$ cinsinden)

$$
\begin{aligned}
C_{m,t} &= C_m(\alpha,\beta,\delta_h)\,\eta_{\delta_h}(\delta_h) + C_{Z,t}\,(x_{cg,ref}-x_{cg}) + \Delta C_{m,lef}\,\Lambda
+ \frac{q\bar c}{2V_T}\Big[C_{m_q}(\alpha) + \Delta C_{m_q,lef}(\alpha)\,\Lambda\Big]\\
&\quad + \Delta C_{m,sb}(\alpha)\frac{\delta_{sb}}{60} + \Delta C_m(\alpha) + \Delta C_{m,ds}(\alpha,\delta_h)\\[4pt]
C_{l,t} &= C_l(\alpha,\beta,\delta_h) + \Delta C_{l,lef}\,\Lambda
+ \Big[\Delta C_{l,\delta_a=20} + \Delta C_{l,\delta_a=20,lef}\,\Lambda\Big]\frac{\delta_a}{20}
+ \Delta C_{l,\delta_r=30}\frac{\delta_r}{30}\\
&\quad + \frac{rb}{2V_T}\Big[C_{l_r}(\alpha) + \Delta C_{l_r,lef}(\alpha)\,\Lambda\Big]
+ \frac{pb}{2V_T}\Big[C_{l_p}(\alpha) + \Delta C_{l_p,lef}(\alpha)\,\Lambda\Big] + \Delta C_{l_\beta}(\alpha)\,\beta\\[4pt]
C_{n,t} &= C_n(\alpha,\beta,\delta_h) + \Delta C_{n,lef}\,\Lambda - C_{Y,t}\,(x_{cg,ref}-x_{cg})\frac{\bar c}{b}
+ \Big[\Delta C_{n,\delta_a=20} + \Delta C_{n,\delta_a=20,lef}\,\Lambda\Big]\frac{\delta_a}{20}
+ \Delta C_{n,\delta_r=30}\frac{\delta_r}{30}\\
&\quad + \frac{rb}{2V_T}\Big[C_{n_r}(\alpha) + \Delta C_{n_r,lef}(\alpha)\,\Lambda\Big]
+ \frac{pb}{2V_T}\Big[C_{n_p}(\alpha) + \Delta C_{n_p,lef}(\alpha)\,\Lambda\Big] + \Delta C_{n_\beta}(\alpha)\,\beta
\end{aligned}
$$

### Artım tanımları ($i \in \{X, Z, m, l, n\}$, $C_Y$ için $\delta_h$ bağımlılığı yoktur)

$$
\begin{aligned}
\Delta C_{i,lef} &= C_{i,lef}(\alpha,\beta) - C_i(\alpha,\beta,\delta_h=0)\\
\Delta C_{i,\delta_a=20} &= C_{i,\delta_a=20}(\alpha,\beta) - C_i(\alpha,\beta,\delta_h=0)\\
\Delta C_{i,\delta_a=20,lef} &= C_{i,\delta_a=20,lef}(\alpha,\beta) - C_{i,lef}(\alpha,\beta) - \Delta C_{i,\delta_a=20}\\
\Delta C_{i,\delta_r=30} &= C_{i,\delta_r=30}(\alpha,\beta) - C_i(\alpha,\beta,\delta_h=0)
\end{aligned}
$$

### Kuvvetler ve momentler (gövde ekseni, AM etrafında)

$$
X=\bar q S C_{X,t},\quad Y=\bar q S C_{Y,t},\quad Z=\bar q S C_{Z,t},\qquad
L=\bar q S b\,C_{l,t},\quad M=\bar q S\bar c\,C_{m,t},\quad N=\bar q S b\,C_{n,t}
$$

### Veri dosyasındaki karşılıklar (`data/f16_AerodynamicData.mat`)

| Formüldeki terim | `f16_AerodynamicData.data.` | Breakpoint |
|---|---|---|
| $C_X(\alpha,\beta,\delta_h)$, $C_Z$, $C_m$ | `CX.CX`, `CZ.CZ`, `Cm.Cm` (20×19×5) | $\delta_h$ = [−25 −10 0 10 25] |
| $C_l(\alpha,\beta,\delta_h)$, $C_n$ | `Cl.Cl`, `Cn.Cn` (20×19×3) | $\delta_h$ = [−25 0 25] |
| $C_{i,lef}(\alpha,\beta)$ | `CX.CXlef`, …, `Cn.Cnlef` (14×19) | $\alpha$: 14 nokta |
| $\eta_{\delta_h}(\delta_h)$ | `Cm.n_dHT` = [1 1 1 1 0.95] | $\delta_h$ = [−25 −10 0 10 25] |
| $\Delta C_{m,ds}(\alpha,\delta_h)$ | `Cm.deltaCmds` (20×7) | $\delta_h$ = [−25 −10 0 10 15 20 25] |
| $\Delta C_{i,sb}(\alpha)$ | `CX.deltaCXSB`, `CZ.deltaCZSB`, `Cm.deltaCmSB` | $\alpha$: 20 nokta |

> **Veri uyarısı:** Ders verisinde `Cl.Clp` tablosunun $\alpha=55^\circ$ değeri **−1200** girilmiş. Komşu değerler −0.10 / −0.14, TP-1538'de **−0.12**. Büyük olasılıkla yazım hatası; kullanmadan önce düzeltilmeli ve raporda belirtilmeli.

### Problem 2b: statik kararlılık türevi

$C_{m_\alpha}$, $C_{m,t}(\alpha)$ eğrisinden merkezi farkla alınır ($\Delta\alpha$ küçük, derece cinsinden sonuç 1/deg):

$$
C_{m_\alpha}(\alpha) \approx \frac{C_{m,t}(\alpha+\Delta\alpha)-C_{m,t}(\alpha-\Delta\alpha)}{2\,\Delta\alpha}
$$

---

## 6. Motor ve itki

**Gaz kolu dişlisi** ($\delta_{th}\in[0,1]$ → güç komutu $P_c$ [%]):

$$
P_c = \begin{cases} 64.94\,\delta_{th} & \delta_{th}\le 0.77\\ 217.38\,\delta_{th}-117.38 & \delta_{th}>0.77\end{cases}
$$

**İtki** ($T_{idle}, T_{mil}, T_{max}$ tabloları Mach ve irtifadan doğrusal ara değerlemeyle alınır):

$$
T = \begin{cases} T_{idle} + (T_{mil}-T_{idle})\,\dfrac{P}{50} & P<50\\[6pt] T_{mil} + (T_{max}-T_{mil})\,\dfrac{P-50}{50} & P\ge 50\end{cases}
\qquad \mathbf F_T = [\,T,\ 0,\ 0\,]^T \ \text{(gövde } x\text{, AM'den geçer)}
$$

**Motor dinamiği** (birinci dereceden gecikme, $\dot P = \frac{1}{\tau}(P_2 - P)$):

| Durum | $P_2$ | $1/\tau$ [1/s] |
|---|---|---|
| $P_c\ge 50$, $P\ge 50$ | $P_c$ | 5 |
| $P_c\ge 50$, $P<50$ | 60 | $r(P_2-P)$ |
| $P_c<50$, $P\ge 50$ | 40 | 5 |
| $P_c<50$, $P<50$ | $P_c$ | $r(P_2-P)$ |

$r(\Delta P)$ = 1.0 ($\Delta P\le 25$), 0.1 ($\Delta P\ge 50$), arada doğrusal ($1.9-0.036\,\Delta P$).

> HW-1 Tablo 2 **kararlı hal** itkisini ister: $P=P_c$ alınır, gecikme sonucu değiştirmez. Tablo 2'nin PDF'inde irtifa sütunu bir satır kaymış görünüyor; okuma raporda açıkça yazılmalı.

---

## 7. Eyleyiciler

Birinci dereceden eyleyici ($\tau = 1/k$), konum ve hız sınırlayıcılı:

$$
\frac{\delta}{\delta_c} = \frac{1}{\tau s + 1}
$$

| Yüzey | Konum sınırı | Hız sınırı | $\tau$ |
|---|---|---|---|
| Stabilatör $\delta_h$ | ±25° | 60 °/s | 0.0495 s |
| Kanatçık $\delta_a$ | ±21.5° | 80 °/s | 0.0495 s |
| Dümen $\delta_r$ | ±30° | 120 °/s | 0.0495 s |
| Hücum kenarı flapı $\delta_{lef}$ | 0° … 25° | 25 °/s | 0.136 s |
| Hava freni $\delta_{sb}$ | 0° … 60° | — | — |

*Değerler Stevens–Lewis / TP-1538 F-16 modelinden alınmıştır; raporda kaynak olarak verilecek.*

**LEF programı** (pilot kontrol etmez; HW-1 doğrulama durumlarında **kullanılmaz**, $\delta_{lef}$ tablodan verilir):

$$
\delta_{lef} = 1.38\,\frac{2s+7.25}{s+7.25}\,\alpha \;-\; 9.05\,\frac{\bar q}{p_s} \;+\; 1.45 \qquad [\text{deg}]
$$

---

## 8. Statik kararlılık

| Eksen | Koşul | Fiziksel anlamı |
|---|---|---|
| Boylamsal | $C_{m_\alpha} < 0$ | $\alpha$ artınca burun aşağı moment |
| Yönsel | $C_{n_\beta} > 0$ | Rüzgar gülü etkisi; burun rüzgara döner |
| Yanal | $C_{l_\beta} < 0$ | Dihedral etkisi; kayılan kanat kalkar |

**Nötr nokta ve statik marj** ($\bar c$ cinsinden):

$$
C_{m_\alpha} = -C_{L_\alpha}\,(x_{np}-x_{cg}),\qquad SM = x_{np}-x_{cg}
$$

**AM kaydırma** (tablolar $x_{cg,ref}=0.35\bar c$'ye göredir): AM geri gittikçe ($x_{cg}\uparrow$) $C_m$'e eklenen $C_Z(x_{cg,ref}-x_{cg})$ terimi kararlılığı azaltır.

**Yüksek $\alpha$'da dinamik yönsel kararlılık** (TP-1538):

$$
C_{n_\beta,dyn} = C_{n_\beta}\cos\alpha - \frac{I_{zz}}{I_{xx}}\,C_{l_\beta}\sin\alpha
$$

---

## 9. Trim

**Kararlı hal:** $\dot u=\dot v=\dot w=0$ (veya $\dot V_T=\dot\alpha=\dot\beta=0$) ve $\dot p=\dot q=\dot r=0$.

**Maliyet fonksiyonu** (her zaman ≥ 0; trimde 0):

$$
J = \sum_i w_i\,f_i^2,\qquad f=\big[\dot u,\ \dot v,\ \dot w,\ \dot p,\ \dot q,\ \dot r,\ \dots\big]
$$

Belirtilen parametreler (ör. $V_T$, $h$, $\gamma$, $\dot\psi$) ile bilinmeyenler (ör. $\alpha$, $\beta$, $\delta_{th}$, $\delta_h$, $\delta_a$, $\delta_r$, $\phi$) sayıca kısıt denklemleriyle eşleşmelidir. Çözücü: `fminsearch`, `fmincon` veya Newton–Raphson.

| Trim durumu | Belirtilen | Ek kısıt |
|---|---|---|
| Düz, seviye uçuş | $V_T$, $h$, $\gamma=0$ | $\phi=\beta=p=q=r=0$ |
| Tırmanış / dalış | $V_T$, $h$, $\gamma$ | $\sin\gamma = a\sin\theta - b\cos\theta$ |
| Koordineli dönüş | $V_T$, $h$, $\dot\psi$ | $p=-\dot\psi\sin\theta,\ q=\dot\psi\sin\phi\cos\theta,\ r=\dot\psi\cos\phi\cos\theta$; $\beta\approx 0$ |
| Çekiş (pull-up) | $V_T$, $h$, $n$ | $q = (n-1)\,g/V_T$ |
| Kararlı yanal kayma | $V_T$, $h$, $\beta$ | $p=q=r=0$, $\phi$ bilinmeyen |

Burada $a=\cos\alpha\cos\beta$, $b=\sin\phi\sin\beta+\cos\phi\sin\alpha\cos\beta$.

---

## 10. Doğrulama kontrol listesi

- [ ] Birimler: tabloya derece, dinamiğe radyan; İngiliz → SI dönüşümleri yalnızca bir yerde.
- [ ] Referans koşul: $V_T$ = 131.28 m/s, $\bar q$ = 7806.3 Pa (Bölüm 4 tablosu).
- [ ] $\delta_{lef}=25^\circ$ iken tüm "lef" terimleri sıfırlanmalı.
- [ ] Simetrik durumlarda ($\beta=p=r=\delta_a=\delta_r=0$) $C_Y=C_l=C_n=0$ olmalı.
- [ ] $x_{cg}=x_{cg,ref}$ iken AM düzeltme terimleri sıfır olmalı.
- [ ] Tablo 1 durum 9–10 trim noktasına benziyor (δlef değerleri LEF programıyla uyumlu); moment katsayılarının ≈ 0 çıkması beklenir.
- [ ] Çözücünün "başarılı" demesi yeterli değil: residual'lar, fiziksel sınırlar ve zaman benzetimiyle kontrol et.
