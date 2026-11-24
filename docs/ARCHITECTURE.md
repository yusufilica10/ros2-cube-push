# Sistem mimarisi

## Çalışan bileşenler

- `ros_gz_sim`: sabit yarışma dünyası ve takım robotu.
- `robot_state_publisher`: URDF'den sabit sensör/şasi TF'leri ve teker TF'leri.
- `ros_gz_bridge`: `/clock`, `/scan`, `/odom`, `/tf`, `/joint_states`, kamera;
  ROS `/cmd_vel` → Gazebo diferansiyel sürüş.
- Haritalamada `slam_toolbox`: `map → odom` ve `/map`.
- Görevde `map_server` + `amcl`: kaydedilmiş gerçek SLAM haritası ve `map → odom`.
- Nav2: global plan, Regulated Pure Pursuit, hız yumuşatma, çarpışma izleme.
- `navigation_ready`: sensör verileri gelmeden lifecycle başlangıcını engeller.
- `cube_detector`: HSV ile parlak küp üst yüzeyleri; kamera iç/dış parametreleri
  ve bilinen 0.18 m üst düzleminden metrik küp merkezleri.
- `cube_mission`: NavigateToPose ile yaklaşma, kamera geri beslemesiyle itme,
  geri çekilme, teslimi doğrulama; durum ve sonuç raporu.
- `velocity_mux`: Nav2 ve görev hız komutları için tek sahip seçimi ve watchdog.

## Veri yolları

`LiDAR → /scan → SLAM (haritalama) veya AMCL/Nav2 (görev)`

`RGB + CameraInfo → cube_detector → /perception/cubes → cube_mission`

`NavigateToPose → controller_server → /cmd_vel_nav → velocity_smoother →
/cmd_vel_smoothed → velocity_mux → /control/cmd_vel → collision_monitor →
/cmd_vel → Gazebo`

İtişte `cube_mission → /mission/cmd_vel → velocity_mux`; Nav2 hızı aynı anda
motora verilmez. Mux bir saniyeden eski komutları sıfırlar. Görev durdurulursa
veya bir hata oluşursa hız kaynağı `stop` olur.

## TF ve koordinatlar

`map → odom → base_footprint → base_link → lidar_link / camera_link → camera_optical_frame`

LiDAR tarama düzlemi z=0.25 m. Küpler z=0.18 m olduğundan tarama küpleri görmez.
Kamera şasi önünde x=0.18 m, yerden z=0.50 m, aşağı eğim 0.95 rad;
yatay görüş açısı 90 derece. Sabit kamera itişte küpün üst yüzeyini görür.
Kamera optik eksenleri X sağ, Y aşağı, Z ileri; `camera_link` X ileri, Y sol,
Z yukarıdır. Algılayıcının `position_camera` alanı `camera_link` çerçevesindedir.

Kaydedilen SLAM haritasında robot başlangıcı (0,0,0); Gazebo dünyasında
(-2.4,-1.4,0). Sabit arena hedefleri haritada yaklaşık (+2.4,+1.4) öteleme ile
ifade edilir. Başka bir harita kaydedilirse bu öteleme yeniden kalibre edilmelidir.

## Görev akışı ve sınırlar

`READY → WAITING_FOR_NAV2 → MISSION_STARTED → NAVIGATE → ACQUIRE → PUSH →
PUSH_REACHED → VERIFY → DELIVERED → COMPLETE`

Küpler sırayla yeşil, mavi, kırmızı. Önce yeşil taşınarak merkez alan boşaltılır.
Mavi ve kırmızı, sol bölgede doğrudan (-0.9, 0) orta hattına taşınır. Sabit arena bilgisine dayalı ara noktalar
iki duvar geçidinden ilerler; küpün güncel konumu ve rengi kameradan alınır.
İtiş sırasında görüş/TF kaybında robot durur. Varsayılan süre 480 saniye
**duvar saati** ile ölçülür; simülasyon süresi ayrıca raporlanır. Hazırlık ve
Nav2 açılışı bu süreye dahil değildir. Her teslim dört yeni kamera karesiyle,
küpün olası dönüşünü kapsayan yarım köşegen payıyla doğrulanır.

Kamera görüş alanı dışındaki veya görüntü kenarında kesilen küplere konum
uydurulmaz. Algılama bu simülasyonun ışığı, düz zemin, küp ölçüsü ve sabit kamera
kalibrasyonunu varsayar. Hedeflerin pastel rengi küp üst yüzeylerinden düşük
doygunlukla ayrılır; hedef ölçüleri ve koordinatları değiştirilmez.

`evaluate_run` yalnızca bağımsız test aracıdır: Gazebo gerçek pozlarından sekiz
köşeyi hesaplayıp hedefin tamamen içinde olmayı kontrol eder. Görev kodu bu
aracı, dosyasını veya Gazebo poz topic'lerini kullanmaz. Kamera ile teslim
iddiası bağımsız fizik testiyle karşılaştırılmadan üç küp başarılı sayılmaz.

## Çalışma ve sürüş kalibrasyonu

Nav2 bileşenleri ayrı executor kullanan tek bir component container içinde
çalışır. Proje `env.sh` ile ROS domain 47 ve Cyclone DDS kullanır. İletişim bu
sanal makinenin loopback arayüzünde yapılır; multicast kullanılmaz.
BLAS/OpenMP ve yazılımsal görüntüleme işçisi tek iş parçacığıyla çalışır.
Teker görsel genişliği 25 mm, temas silindiri genişliği 2 mm; yarıçap 50 mm,
merkezler arası mesafe 300 mm. İnce temas yüzeyi dönme sırasındaki odometri
hatasını kalibrasyon ölçümünde yaklaşık %8.6’dan %0.4’e indirmiştir.
Doğrusal ölçümde odometri ve fiziksel hareket yaklaşık 0.3866 m eşleşmiştir.
İvme sınırları doğrusal 0.5 m/s², açısal 1 rad/s²’dir.

Son hedefe yaklaşırken aynı görüntüdeki küp ve pastel hedefin göreli konumu
kullanılır; ortak TF öteleme hatası teslim kontrolünü etkilemez. Normal
itiş 0.25 m/s; temasa yaklaşırken ve hedef yakınında hız düşürülür.

## Sanal makinede yayın yükü

Gazebo 2 ms fizik adımıyla çalışır. Saat köprüsü ham `/simulation/clock_raw`
verisini C++ `clock_throttle` düğümünde süzer ve `/clock` üzerinde simülasyon
zamanına göre en fazla 50 Hz dağıtır. Zaman damgaları değiştirilmez; geriye
sıfırlama ve ilk örnek hemen iletilir. Fizik adımı ve 480 saniyelik gerçek
süre sınırı aynıdır. Teker eklem durumu 20 Hz, LiDAR 10 Hz, RGB kamera 5 Hz’dir.

Saat köprüsünün bağımsız entegrasyon testi:
`python3 src/cube_push_sim/test/check_clock_throttle.py`.
Test yalnızca `/test_clock/*` konularını kullanır; çalışan `/clock` akışını
değiştirmez.

Ayar kaynağı: [Gazebo JointStatePublisher](https://gazebosim.org/api/sim/10/classgz_1_1sim_1_1systems_1_1JointStatePublisher.html).

## Görüntülü kullanım düzeltmeleri

- Sensörler ekransız EGL sunucusunda, görüntüleyici ayrı küçük OGRE
  penceresinde çalışır. Böylece sensör üretimi pencerenin çiziminden ayrılır.
- Kamera aboneliği en güncel tek kareyi tutar. Duvar saatiyle çalışan 20 Hz
  kuyruk kontrolü, poz TF’si ulaşmış en yeni kareyi seçer; eski kareler atılır.
- Gövde odometrisi görüntünün çekildiği zamandan, AMCL’nin map–odom düzeltmesi
  güncel dönüşümden alınır; düzeltme iki simülasyon saniyesinden eskiyse
  harita konumu üretilmez.
- Metrik ölçüm için kesilmemiş 18 cm üst yüzey şartı korunur. Kenarda görünen
  renkli yüzey yalnızca dönüş yönü ipucu sağlar; konum uydurulmaz.
- Kamera merkezleme kazancı 0.6, açısal sınır 0.15 rad/s. Arama tam tur atmaz.
  Kararlılık harita konumu yerine üç ayrı görüntüdeki gövde konumuyla ölçülür.
- Transfer, yeşilde (1.8, -0.55)’e; diğer renklerde önce (-0.9, 0)’a, ardından
  (1.8, -0.55)’e gider. Aradaki gereksiz (0.6, 0) duruşu kaldırılmıştır.
- Son itiş hedef merkezinden 4 cm uzaklığa zorlamaz. Küp yarım köşegeni ve
  6 cm güvenlik payı kullanılarak tamamen içerideyse durur; geri çekildikten
  sonra teslim dört yeni görüntüyle ayrıca doğrulanır.
- Nav2 bond konusu her açılışta ayrılır; görev başlamadan önce navigasyon
  ve kamera üç saniye kararlı olmalıdır. Görev sınırı hâlâ 480 gerçek saniyedir.

Resmi DDS yapılandırması: https://cyclonedds.io/docs/cyclonedds/latest/config/config_file_reference.html
Yerel, sudo gerektirmeyen kurulum: `bash tools/install_middleware_local.sh`.
Sistem kurulumu alternatifi: `sudo apt install ros-lyrical-rmw-cyclonedds-cpp`.
