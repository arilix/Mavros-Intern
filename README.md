# Divisi VTOL — MAVROS

## Apa itu MAVROS?

MAVROS adalah sebuah paket ROS (Robot Operating System) yang berfungsi sebagai jembatan komunikasi antara ROS dengan autopilot berbasis protokol MAVLink, seperti PX4 atau ArduPilot. Dengan MAVROS, pengguna dapat mengontrol, memonitor, dan mengintegrasikan drone atau kendaraan otonom lain ke dalam ekosistem ROS dengan mudah.

---

## Cara Menginstal MAVROS

Panduan resmi MAVROS (contoh untuk ROS 2 Humble) tersedia di [sini](https://docs.ros.org/en/humble/p/mavros/).

Berikut ringkasan singkat cara memasang MAVROS pada beberapa distribusi ROS yang sering digunakan:

### ROS 2 — Humble (Ubuntu 22.04)

```bash
# Via apt (jika paket tersedia)
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Atau build dari source
source /opt/ros/humble/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/mavlink/mavros.git
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### ROS 2 — Foxy (Ubuntu 20.04)

> **Catatan:** Foxy menggunakan API ROS 2 yang lebih lama; beberapa nama topic/service bisa berbeda. Jika paket biner `ros-foxy-mavros` tidak tersedia, disarankan build dari source.

```bash
source /opt/ros/foxy/setup.bash
mkdir -p ~/ros2_foxy_ws/src && cd ~/ros2_foxy_ws/src
git clone https://github.com/mavlink/mavros.git
cd ~/ros2_foxy_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

### ROS 1 — Noetic (Ubuntu 20.04)

> **Catatan:** Noetic adalah ROS 1 dan menggunakan `catkin`. Untuk integrasi ROS 1 ↔ ROS 2, gunakan `ros1_bridge`.

```bash
sudo apt update
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
sudo apt install python3-rosdep python3-rosinstall-generator
sudo rosdep init || true
rosdep update

# Beberapa fitur MAVROS memerlukan data GeographicLib
sudo apt install geographiclib-tools
# Lihat dokumentasi MAVROS untuk instruksi lengkap pemasangan data GeographicLib
```

---

## Fungsi MAVROS dalam Drone

MAVROS memiliki beberapa fungsi utama dalam sistem drone berbasis ROS:

- Menghubungkan ROS dengan autopilot menggunakan protokol MAVLink.
- Mengirim dan menerima perintah misi, seperti takeoff, landing, waypoint, dan mode penerbangan.
- Monitoring status drone (baterai, GPS, attitude, dll.) secara real-time.
- Mengontrol aktuator dan sensor melalui topik ROS.
- Integrasi dengan berbagai sensor dan perangkat lunak lain di ekosistem ROS.

Singkatnya, MAVROS memudahkan pemrograman untuk integrasi sistem pada drone.

---

## Contoh Tipe-tipe Command MAVROS

Berikut beberapa contoh command MAVROS yang umum digunakan pada drone.

### Penjelasan Tipe Command / Service

| Tipe | Keterangan |
|------|-----------|
| **CommandBool** | Service untuk perintah boolean, misal arming/disarming (`value: true` / `false`). |
| **SetMode** | Service untuk mengubah mode penerbangan (GUIDED, OFFBOARD, dsb.). |
| **CommandTOL** | Service untuk perintah TakeOff / Landing (altitude, latitude, longitude, yaw, dsb.). |
| **GlobalPositionTarget** | Message untuk mengirim target posisi global (lat, lon, alt) ke autopilot. |
| **PoseStamped** | Message untuk mengirim target posisi lokal (x, y, z) dalam frame lokal. |
| **TwistStamped** | Message untuk mengirim perintah kecepatan linear dan angular ke drone. |

### 1. Set Mode (Guided / Offboard)

Mengubah mode penerbangan drone ke GUIDED (ArduPilot) atau OFFBOARD (PX4):

```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "GUIDED"}'
# PX4
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "OFFBOARD"}'
```

### 2. Arming

Mengaktifkan (arming) motor drone:

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
```

### 3. Takeoff

Perintah takeoff otomatis:

```bash
ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL '{altitude: 10.0}'
```

### 4. Menuju Waypoint (Global / Lokal / Velocity)

- **Waypoint global:**
  ```bash
  ros2 topic pub /mavros/setpoint_position/global mavros_msgs/msg/GlobalPositionTarget ...
  ```
- **Waypoint lokal:**
  ```bash
  ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped ...
  ```
- **Kontrol kecepatan (velocity):**
  ```bash
  ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped ...
  ```

---

Perintah lainnya bisa dilihat di wiki MAVROS [di sini](https://docs.ros.org/en/humble/p/mavros/).

Coba juga tutorial simulasi drone [di sini](https://github.com/arilix/Drone-sim/blob/main/Drone_sim/Tutorial.md).





