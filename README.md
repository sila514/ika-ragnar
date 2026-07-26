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

## Simülasyonda test

Gazebo simülasyonu ve `run_sim.sh` ayrı bir repoda:
[sim-ws](https://github.com/sila514/sim-ws).

```bash
colcon build
source install/setup.bash
~/sim_ws/run_sim.sh --mock-vision   # Hailo yerine mock_yolo_cpu ile
```
