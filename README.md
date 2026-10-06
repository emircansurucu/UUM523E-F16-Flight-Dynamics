<div align="center">

# UUM523E — Advanced Flight Dynamics

**İTÜ · Uçak ve Uzay Mühendisliği · Güz 2026**

![MATLAB](https://img.shields.io/badge/MATLAB-R2024b-orange?logo=mathworks)
![Simulink](https://img.shields.io/badge/Simulink-Aerospace%20Blockset-blue)
![License](https://img.shields.io/badge/Lisans-MIT-green)

</div>

Bu depo, UUM523E dersi boyunca yapılacak ödev ve projelerin çalışma alanıdır. Ders tek bir yol üzerine kuruludur: grup 3. haftada oluşur ve aynı grup HW-1, HW-2, ara dönem projesi ve final projesini birlikte yürütür. Her aşama, grubun bir önceki aşamadaki çalışmasının üzerine inşa edilir.

Ders içeriği, yol planı ve teslim beklentileri için: **[PROJECT.md](PROJECT.md)**

## Ders akışı

```mermaid
flowchart LR
    G["<b>Grup kurulumu</b><br/>4 kişilik gruplar<br/>12 Ekim"]
    H1["<b>HW-1</b><br/>F-16 modelleme<br/>ve benzetim<br/><i>10 puan · 26 Ekim</i>"]
    H2["<b>HW-2</b><br/>Trim ve<br/>doğrusallaştırma<br/><i>10 puan · 9 Kasım</i>"]
    M["<b>Ara dönem projesi</b><br/>Statik kararlılık ↔ doğrusal model<br/>Dinamik modlar<br/><i>30 puan · 23 Kasım</i>"]
    F["<b>Final projesi</b><br/>Bir ileri konu seçilir<br/><i>50 puan · final haftası</i>"]
    G --> H1 --> H2 --> M --> F

    F --> T1["Uçuş niteliği standartları<br/>ve regülasyonlar"]
    F --> T2["Yüksek hücum<br/>açısında uçuş"]
    F --> T3["Modelleme<br/>belirsizlikleri"]
    F --> T4["Artık eyleyicili<br/>uçak"]
    F --> T5["Kütle eyleyicili<br/>uçak"]

    style G fill:#eef6ee,stroke:#2e7d32
    style F fill:#fdf3e3,stroke:#b26a00
```

Final projesinde her grup beş konudan birini seçer; bir konuyu en fazla iki grup alabilir. Konumuz henüz belirlenmedi.

## Gereksinimler

- MATLAB **R2024b**
- Simulink ve Aerospace Blockset
- Simulink Control Design (trim ve doğrusallaştırma)
- Optimization Toolbox

## Başlangıç modeli hakkında notlar

Çalışmalara, Raktim Bhattacharya'nın açık kaynaklı doğrusal olmayan F-16 MATLAB/Simulink modeli (NASA TP-1538 aerodinamik verisi) ile başlanıyor. Bu model yalnızca bir başlangıç noktası; ders ilerledikçe değiştirilecek ve kendi çalışmalarımızla genişletilecek. İlk incelemede fark edilen ve doğrulanması gereken noktalar:

- Model pratikte **α ≤ 45°** aralığında çalışıyor; bu sınır aşılırsa hesap bozuluyor.
- `F16_2022a.mdl` ve `F16_2023a.mdl` ana modelin eski hâli; `F16.slx` kullanılmalı.
- Eyleyici dinamiği, yüzey sınırları ve motor dinamiği modelde yok.
- Eylemsizlik çarpımı (Ixz) işareti ders kitabındaki kurala göre ters olabilir.
- Hazır trim betiği (`TrimF16.m`) kontrol edilmeden kullanılmamalı.
- Model ağırlık merkezini %30 c̄ alıyor, TP-1538'deki nominal değer %35 c̄; bu seçim statik kararlılığı doğrudan etkiliyor.

## Kaynaklar

**Ders kaynakları**
- Yechout, T. R., *Introduction to Aircraft Flight Mechanics*, AIAA, 2003.
- Cook, M. V., *Flight Dynamics Principles: A Linear Systems Approach to Aircraft Stability and Control*, Butterworth-Heinemann, 2012.
- De Marco, A., Duke, E. L., Berndt, J. S., *A General Solution to the Aircraft Trim Problem*, AIAA 2007-6703, 2007.
- Stengel, R. F., *Flight Dynamics*, 2nd ed., Princeton University Press, 2022.
- Klein, V., Morelli, E. A., *Aircraft System Identification: Theory and Practice*, AIAA, 2006.
- Thomas, S., Kwatny, H. G., Chang, B. C., *Nonlinear Reconfiguration for Asymmetric Failures in a Six Degree-of-Freedom F-16*, ACC, 2004.

**Model ve veri**
- Nguyen vd., *Simulator Study of Stall/Post-Stall Characteristics of a Fighter Airplane With Relaxed Longitudinal Static Stability*, NASA TP-1538, 1979 — [NTRS](https://ntrs.nasa.gov/citations/19800005879)
- Stevens, B. L., Lewis, F. L., *Aircraft Control and Simulation*, Wiley, 1992.

## Atıf ve lisans

- **Başlangıç modeli:** © 2023 Raktim Bhattacharya, Texas A&M University — [MIT Lisansı](LICENSE). Orijinal dosyalardaki yazar bilgileri korunmuştur.
- **Ders kapsamındaki çalışmalar:** Emircan Sürücü, İTÜ — UUM523E, Güz 2026.
