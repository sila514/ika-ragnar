# ika-ragnar

RAGNAR IKA projesinin ROS 2 (Jazzy) paketleri. Bir paletli/tekerlekli aracın
parkurda otonom gezinmesi, kamerayla hedef tespiti ve taretle hedefe
kilitlenip ateş etmesi üzerine kurulu.

## Paketler

### `ika_vision`
Kameradan (`/image_raw`) görüntü alıp hedef tespiti yapan ve
`/detected_targets` (`vision_msgs/Detection2DArray`) topic'ine yayınlayan
node'lar.

- **`yolo_node`** — Gerçek donanım (Raspberry Pi + Hailo-8L) için YOLOv8
  çıkarımı. Hailo SDK'sı gerektirir, bu makinede çalışmaz.
- **`mock_yolo_cpu`** — Hailo donanımı olmayan geliştirme makinelerinde aynı
  `/detected_targets` formatını üretmek için CPU/OpenCV tabanlı mock:
  HOG+SVM insan tespiti + (sahnede insan yoksa) kontur tabanlı genel nesne
  tespiti. `yolo_node`'a hiç dokunmaz, sadece test amaçlı alternatiftir.

### `turret_mission`
Görsel hizalama (taret PID) ve görev akışı (waypoint navigasyonu) node'ları.

- **`turret_aim_node`** — `/detected_targets`'taki en yüksek skorlu hedefi
  alıp piksel hatasını P kontrolcüyle `/turret_cmd`'ye çevirir; hizalanınca
  ve `/estop` kapalıyken `/fire_cmd` yayınlar.
- **`mission_node`** (`nav2_simple_commander` tabanlı) — Waypoint'leri
  sırayla Nav2'ye verir; hedef görülünce navigasyonu duraklatıp taret
  angajmanını bekler, hedef kaybolunca/ateş tamamlanınca kaldığı yerden
  devam eder.
- **`mock_target_publisher`** — Gerçek kamera/YOLO olmadan `/detected_targets`
  yayınlayan, `/turret_cmd`'yi dinleyip kapalı çevrimde hedef hatasını
  simüle eden test node'u (PID yakınsamasını test etmek için).
- **`simple_twist_mux`** — `ros-jazzy-twist-mux`'ın bu ortamda çalışmaması
  nedeniyle yazılmış minimal alternatif: `/cmd_vel_joy` (öncelikli) +
  `/cmd_vel_nav` → `/cmd_vel`.

### `web_ui`
Aracı tarayıcıdan izlemek/kontrol etmek için `roslibjs` tabanlı tek sayfalık
statik web arayüzü (`index.html`).

## Entegrasyon / Teslim Notları (RPi Fiziksel Test)

**`turret_mission` (turret_aim_node + mission_node) henüz RPi'a (ragnar) deploy
edilmedi** — RPi'da şu an sadece `ika_vision` çalışıyor. Görev 7/8'in fiziksel
testine geçildiğinde:

1. `turret_mission` paketi RPi'a kopyalanıp `colcon build --packages-select
   turret_mission` ile derlenmeli.
2. `target_class_id` filtresi (varsayılan `"sign_target"`) mutlaka
   doğrulanmalı — koniye (class 0) ateş etmemesi bu filtreye bağlı.
3. Pan/tilt yön işareti (sağ/sol tersliği) gerçek donanımda test edilmeli.
4. TB6600 ENA polaritesi Hüseyin tarafında doğrulanmalı.
5. **`odom→base_link` TF eksik** — ne bu repoda (`turret_mission`/`ika_vision`)
   ne de `ros_io.rs`'de bir EKF/`robot_localization` node'u veya TF
   broadcaster yok. Sadece simülasyonda (Gazebo diff_drive plugin) otomatik
   geliyor. Biri eklenmeden Nav2 gerçek robotta hatasız ama sessizce hiç
   navigasyon yapmaz. Ayrıca `/turret_cmd` (Vector3) derece/mutlak açı değil,
   `kp_pan/kp_tilt × piksel_hatası` (varsayılan ±5.0 clamp) — ros_io.rs
   tarafında hız komutu gibi yorumlanmalı, bunu Hüseyin'e teyit ettirin.
   **Doğrulanmış bug:** Hüseyin'in `ros_io.rs`'i bu değeri `rem_euclid(360)`
   ile derece gibi sarıyor, örn. `-3` → `357` oluyor — taretin anlık tam tur
   atmasına yol açar. Wrap-around KALDIRILMALI, değer direkt küçük
   hız/delta komutu olarak kullanılmalı.
6. **`maps/parkur_map.pgm` ve Nav2 bringup dosyaları (AMCL/costmap
   parametreleri) hiçbir repoda yok** — `mission_node` bunlara (satır
   27/56 yorumlarında belirtildiği gibi) güveniyor ama harita dosyası da
   Nav2'nin kendi launch/parametre seti de commit edilmemiş. `mission_node`
   gerçek/simüle robotta bunlar olmadan çalıştırılamaz.
7. RPi'da Rust toolchain (`cargo`) kurulu değil — `src/uvm` (muhtemelen
   `ros_io.rs`) colcon tarafından paket olarak tanınmıyor bile
   ("ament_cargo... FileNotFoundError: cargo"). Hüseyin'e iletilmeli.
8. **Koni kaçınma — ÖNERİLEN: `fused_avoidance_node` (lidar+kamera
   birleşik).** Bu, `obstacle_avoidance_node` (sadece lidar) ve
   `vision_avoidance_node`'un (sadece kamera) YERİNE geçer — o ikisi
   hâlâ repoda duruyor (referans/yedek için) ama **aynı anda
   `fused_avoidance_node` ile çalıştırılmamalı**, hepsi `cmd_vel_nav`'a
   yazıyor, çakışır. Sadece birini seçin, önerilen `fused_avoidance_node`:
   - Lidar (`/scan`) = güvenlik son savunma hattı: önde gerçekten çok
     yakın bir şey varsa (koni olsun olmasın) kamera ne görürse görsün
     durur/döner.
   - Kamera (`/detected_targets`, "koni") = yönlendirme: iki koni sol/sağ
     dengeliyse aralarındaki boşluğa yönelir, tek koniden uzaklaşır.
   - Kamera koni görmüyor ama lidar orta mesafede engel görüyorsa
     (duvar vb.): sadece lidar'a göre yavaşlayıp döner.
   6 senaryolu mock testle doğrulandı (güvenlik-kazanır testi dahil).
   **Ama gerçek donanımda/kamerayla hiç test edilmedi** — bbox eşikleri
   (`panic_width_px=220`, `danger_width_px=60`) kalibre edilmemiş, yarın
   ilk denemede ayarlamak gerekebilir. RPi'a deploy edildi.
   Çalıştırma: `simple_twist_mux` + `fused_avoidance_node` (mux olmadan
   `cmd_vel_nav` hiçbir yere gitmez).

## Simülasyonda test

Gazebo simülasyonu ve `run_sim.sh` ayrı bir repoda:
[sim-ws](https://github.com/sila514/sim-ws).

```bash
colcon build
source install/setup.bash
~/sim_ws/run_sim.sh --mock-vision   # Hailo yerine mock_yolo_cpu ile
```
