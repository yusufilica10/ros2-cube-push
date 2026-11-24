# Görüntülü otonom görev doğrulaması — 24 Kasım 2026

**BAŞARILI: 3/3 teslim, 422,31 gerçek saniye (7 dakika 2,31 saniye).**

`gui24` koşusunda Gazebo penceresi açık, RViz kapalıydı. Görev sayacı başladıktan
sonra elle müdahale edilmedi. Süre sınırı değiştirilmedi: 480 gerçek saniye.
Hazırlık süreye dahil değil; bu koşuda 57,69 saniye pay kaldı.

## Bağımsız fizik kontrolü

`full_run_mission.json`: COMPLETE, yeşil/mavi/kırmızı teslim, hata yok.
`full_run_physical_check.json`: her küpün döndürülmüş sekiz köşesinin eşleşen
0,60 m hedefin tamamen içinde olduğunu bağımsız olarak doğrular.

| Küp | Son dünya merkezi (m) | Tamamen içeride |
|---|---|---|
| green | (2.3146, -0.0265) | Evet |
| blue | (2.2526, -1.1079) | Evet |
| red | (2.3173, 1.1127) | Evet |

Görev kontrolü Gazebo gerçek pozlarını okumaz. Bu pozlar yalnızca test sonundaki
bağımsız değerlendirmede kullanılır. Tam günlükler `docs/runs/gui24/` içindedir.

## Düzeltmeler ve koşullar

- ROS 2 Lyrical / Gazebo Jetty 10.5, dört sanal CPU, yazılımsal görüntüleme.
- İletişim: Cyclone DDS, loopback, domain 47. Önceki Fast DDS koşularında köprü
  yayın hatası, gecikme ve Nav2 bağlantı kopmaları gözlenmiştir.
- Kamera 320×240, 5 Hz. En güncel, odometri TF’si ulaşmış kare seçilir;
  kamera kuyruğu simülasyon saati yavaşlasa da duvar saatiyle boşaltılır.
- Kamera araması sınırlı; kesilmiş renkli yüzey sadece yön ipucu verir.
  Konum için 18 cm üst yüzey geometrisi şartı korunur.
- Dönüş kontrolü gecikmeye göre yavaşlatıldı. Kararlılık, harita sıçraması yerine
  gövde koordinatlarında üç ayrı görüntüyle kontrol edilir.
- Gereksiz orta durak kaldırıldı; küpler iki geçitten doğrudan transfer edilir.
- Tam merkezde 4 cm hassasiyet zorlanmaz. Tam içeride olma, yarım köşegen ve
  6 cm güvenlik payıyla belirlenir; geri çekilince dört görüntüyle teyit edilir.
- Sensör sunucusu ekransız; küçük OGRE görüntüleyicisi ayrı çalışır.
- Fizik adımı 2 ms, eklemler 20 Hz, LiDAR 10 Hz, ROS saati en fazla 50 Hz.
- Paket derlemesi, 15 birim kontrolü ve saat köprüsü entegrasyon kontrolü geçti.

Bu, tek başarılı tam görüntülü koşunun kanıtıdır; tekrar başarı oranı ölçümü değildir.
Eski `full14` koşusu ekransızdı: 464,45 saniye, 3/3. Kaydı
`docs/runs/full14/` altında korunur. Başarısız ara teşhis denemeleri `.debug/`
altında yerel olarak saklanır; yeni başarı iddiasının kanıtı değildir.

## Yeniden çalıştırma

Her terminalde önce `source ./env.sh` çalıştırılmalıdır. Eski Fast DDS oturumuyla
komut göndermeyin. Bağımlılıklar ve ayrıntılı komutlar README içindedir.

```bash
cd ~/Desktop/ros2_ws
source ./env.sh
ros2 launch cube_push_sim competition.launch.py rviz:=false
```

Başka bir terminalde aynı ortamı yükleyip görevi başlatın:

```bash
cd ~/Desktop/ros2_ws
source ./env.sh
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status --field data --full-length
```

COMPLETE sonrası simülasyon açıkken bağımsız kontrol:

```bash
ros2 run cube_push_sim evaluate_run --output /tmp/physical_delivery_check.json
```

Yeni koşu için simülasyonun tamamını kapatıp yeniden açın. Resmi görevde sayaç
başladıktan sonra teleop, RViz hedefi, yazılım/poz değişikliği ve küp taşıma yoktur.
Ders PDF’si gerçek/simülasyon saati ayrımını açıkça belirtmez; bu uygulama daha
sıkı olan gerçek süre yorumuyla test edilmiştir. Hocadan ayrım ayrıca teyit edilebilir.
