# ISTUN — Otonom küp itme (ROS 2)

Sabit 6×4 m yarışma arenasında SLAM, AMCL/Nav2 ve RGB kamera tabanlı
üç renkli küp itme uygulaması. **24 Kasım 2026 tarihli otonom testte 3/3 teslim,
422,31 gerçek saniyede (7 dakika 2 saniye), Gazebo penceresi açıkken bağımsız fizik kontrolüyle doğrulandı.**

## Ortam

- Ubuntu 26.04.1 LTS, ROS 2 Lyrical
- Gazebo Jetty / gz-sim 10.5.0, ros_gz 3.0.10
- SLAM Toolbox 2.10.0, Navigation2 1.5.2, OpenCV 4.10.0

Kullanıcının tercihiyle Lyrical/Jetty kullanılıyor. Ders PDF'si 24.04/Jazzy/Harmonic
veya 22.04/Humble/Fortress belirtiyor; bu sürüm farkı ders tesliminde açıklanmalı.
Arena duvarları, küp ölçüleri/başlangıçları ve hedef koordinatları PDF'ye göredir.
Hedeflerin pastel rengi algılamada küplerden ayrılır; ölçü ve konumları değişmez.

## Derleme

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/lyrical/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select cube_push_sim
source ./env.sh
```

Cyclone DDS gerekir. Bu makinede proje içine kuruldu. Yeni bilgisayarda:

```bash
sudo apt install ros-lyrical-rmw-cyclonedds-cpp
```

Sudo kullanmadan resmi paketleri çalışma alanına açmak için alternatif:

```bash
bash tools/install_middleware_local.sh
```

`env.sh` yerel kurulumu otomatik bulur; `.vendor/` Git deposuna girmez.

Bu makinede gerekli paketler kurulu. Her yeni terminalde `source ./env.sh`
çalıştırın; ROS 2, çalışma alanı ve proje iletişim ayarlarını (domain 47, Cyclone DDS) yükler. Python çalıştırılabilirlerinde değişiklikten sonra yeniden derleyin;
CMake RENAME ile kurulan dosyalar symlink-install'da da kopyalanabilir.

## Tek komutla hazırlık

Önceki simülasyon, SLAM, Nav2, WASD ve görev süreçlerini kapatın. Aynı dünyayı
iki kere açmayın. Workspace kökünde:

```bash
source ./env.sh
ros2 launch cube_push_sim competition.launch.py
```

Sanal makinede saat dağıtımı 50 Hz, teker eklem yayını 20 Hz ile sınırlıdır;
fizik adımı ve sensör ölçüm sıklıkları korunur.

Nav2, sensörler yayın yapmaya başladıktan sonra otomatik hazırlanır. Görev
kendiliğinden başlamaz. RViz'de harita ve başlangıç konumunu kontrol edin.
Sensörler ayrı ekransız sunucuda, Gazebo görüntüleyicisi küçük OGRE penceresinde
çalışır. Yazılımsal görüntüleme ve tek iş parçacığı ayarları otomatik uygulanır.
Gerekirse aynı ayarı açıkça vermek için:

```bash
LIBGL_ALWAYS_SOFTWARE=1 ros2 launch cube_push_sim competition.launch.py
```

İşlem yükünü azaltan ekransız test:

```bash
LIBGL_ALWAYS_SOFTWARE=1 EGL_PLATFORM=surfaceless ros2 launch cube_push_sim competition.launch.py headless:=true rviz:=false
```

### Görevi başlat / durdur

```bash
ros2 service call /mission/start std_srvs/srv/Trigger '{}'
ros2 topic echo /mission/status --field data --full-length
```

Acil durdurma:

```bash
ros2 service call /mission/stop std_srvs/srv/Trigger '{}'
```

Görev başladıktan sonra WASD, RViz hedefleri veya Gazebo poz değiştirme
kullanmayın. Test bittiğinde yeni deneme için tüm simülasyonu sıfırdan başlatın.
Süre hazırlık tamamlandıktan sonra başlar; 480 saniye duvar saatiyle sınırlıdır.
Sonuç `/tmp/cube_mission_latest.json` dosyasına yazılır; geçmiş testleri korumak
istiyorsanız bunu her yeni denemeden önce başka bir ada kopyalayın.

### Kamera

```bash
ros2 run rqt_image_view rqt_image_view /perception/debug_image
ros2 topic echo /perception/cubes --once --field data --full-length
```

320×240 RGB ve canlı CameraInfo kullanılır. Kamera üst yüzeyden konum hesaplar;
kısmen kesilen, örtülen veya beklenen 18 cm kareye uymayan yüzeyleri reddeder.
Gövde dönüşümü görüntü zamanından, AMCL map–odom düzeltmesi güncel TF’den
alınır; gerekli TF yoksa map konumu null olur. Yöntem düz zemin, bilinen küp ölçüsü ve bu dünyanın aydınlatmasına
bağlıdır. Gazebo nesne pozları görev algoritmasına verilmez.

## Bileşenleri ayrı açma

- `ros2 launch cube_push_sim sim.launch.py`
- `ros2 launch cube_push_sim mapping.launch.py` (yalnızca haritalamada)
- `ros2 run cube_push_sim teleop_wasd` (yalnızca hazırlık/haritalama)
- `ros2 launch cube_push_sim navigation.launch.py` (SLAM kapalı olmalı)
- `ros2 launch cube_push_sim perception.launch.py viewer:=true`

W/S ileri–geri 0.8 m/s, A/D dönüş 1.1 rad/s; boşluk dur, Q çıkış.
Tuş tekrarı kesildiğinde hız sıfırlanır. Nav2'nin hızı WASD'den bağımsızdır.
LiDAR 0.25 m yüksekte olduğundan 0.18 m küpleri görmemesi doğrudur.

Haritalama tamamlanınca:

```bash
mkdir -p maps
ros2 run nav2_map_server map_saver_cli -f "$PWD/maps/arena_new" --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true
```

Varsayılan navigasyon haritası öğrencinin oluşturduğu gerçek SLAM kaydı
`src/cube_push_sim/maps/arena_check_20261122_153109.yaml` dosyasıdır.
Başlangıcı (0,0,0), Gazebo karşılığı (-2.4,-1.4,0). Başka bir SLAM haritasına
geçerseniz hedeflerin harita koordinatları için ötelemeyi tekrar kontrol edin.
`maps/arena_reference.yaml` ayrıca plandan türetilmiş bir referanstır; SLAM
başarısının kanıtı değildir ve görevde kullanılmaz.

## Testler ve kanıtlar

```bash
source ./env.sh
colcon test --packages-select cube_push_sim
colcon test-result --verbose
python3 src/cube_push_sim/test/check_clock_throttle.py
gz sdf -k src/cube_push_sim/worlds/competition.sdf
ros2 run cube_push_sim evaluate_run --output /tmp/physical_delivery_check.json
```

`evaluate_run` bağımsız test aracıdır. Gazebo gerçek pozlarından küpün sekiz
köşesinin eşleşen hedef sınırları içinde olduğunu kontrol eder; görev kodu bu
veriyi okumaz. Sayaç yalnızca merkez konumuna bakmaz.

Doğrulanmış kontroller: paket derlemesi, SDF/URDF dönüşümü, renk/üst yüzey
geometrisi ve sınır testleri; kayıtlı haritada kısa Nav2 hedefi. Son tamamlanan üç küplük
fizik testinde 3/3 teslim 422,31 gerçek saniyede doğrulandı (Gazebo açık, RViz kapalı).
Bu sonuç tek tam koşunun kanıtıdır. Güncel ayrıntılar ve bağımsız kontrol:
`docs/VALIDATION.md`.
Eski ön kamera örneği `docs/cube_detection_live.png`, kısa Nav2 action çıktısı
`docs/nav2_first_goal_test.txt`. Mimari: `docs/ARCHITECTURE.md`.

## Kaynaklar

- Verilen ISTUN_CSE412_BYM412_Simulation_Competition_EN.pdf
- https://github.com/gazebosim/ros_gz
- https://gazebosim.org/docs/jetty/ros_installation/
- Kurulu nav2_bringup Lyrical nav2_params.yaml (Apache-2.0): Nav2 ayarlarının başlangıç kaynağı
- https://github.com/ros-navigation/navigation2

Güncel görüntülü sürüm yedeği: `artifacts/cube_push_sim_gui_20261124.zip`.
Eski `cube_push_sim_20261124.zip`, önceki ekransız test sürümüdür.
