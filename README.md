# VTOL Division Internship - MAVROS

## MAVROS..?

MAVROS adalah sebuah paket ROS (Robot Operating System) yang berfungsi sebagai jembatan komunikasi antara sistem operasi robot (ROS) dengan autopilot berbasis protokol MAVLink, seperti PX4 atau ArduPilot. Dengan MAVROS, pengguna dapat mengontrol, memonitor, dan mengintegrasikan drone atau kendaraan otonom lain ke dalam ekosistem ROS dengan mudah.

## Cara Menginstal MAVROS
Panduan resmi MAVROS (contoh untuk ROS 2 Humble) tersedia di
[Install Mavros](https://docs.ros.org/en/humble/p/mavros/).

Di bawah ini ringkasan singkat cara memasang MAVROS pada beberapa distribusi ROS yang sering digunakan.

- **ROS 2 — Humble (Ubuntu 22.04)**
	- Cara singkat (paket biner bila tersedia) atau dari source di workspace:
		```bash
		# via apt (jika paket tersedia untuk distro Anda)
		sudo apt update
		sudo apt install ros-humble-mavros ros-humble-mavros-extras

		# atau: build dari source di workspace ROS2
		source /opt/ros/humble/setup.bash
		cd ~/ros2_ws/src
		git clone https://github.com/mavlink/mavros.git
		cd ~/ros2_ws
		rosdep update
		rosdep install --from-paths src --ignore-src -r -y
		colcon build
		source install/setup.bash
		```

- **ROS 2 — Foxy (Ubuntu 20.04)**
	- Catatan: Foxy menggunakan API ROS 2 yang lebih tua; beberapa nama topic/service bisa berbeda. Jika paket biner `ros-foxy-mavros` tidak tersedia, rekomendasi build dari source:
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

- **ROS 1 — Noetic (Ubuntu 20.04)**
	- Catatan: Noetic adalah ROS 1 dan menggunakan `catkin`. Untuk integrasi ROS1↔ROS2, gunakan `ros1_bridge`.
		```bash
		sudo apt update
		sudo apt install ros-noetic-mavros ros-noetic-mavros-extras
		sudo apt install python3-rosdep python3-rosinstall-generator
		sudo rosdep init || true
		rosdep update

		# Beberapa fitur MAVROS memerlukan data GeographicLib
		sudo apt install geographiclib-tools
		# Untuk instruksi lengkap pemasangan data GeographicLib, lihat dokumentasi MAVROS
		```




## Fungsi MAVROS dalam Drone

MAVROS memiliki beberapa fungsi utama dalam sistem drone berbasis ROS, antara lain:

- Menghubungkan ROS dengan autopilot menggunakan protokol MAVLink.
- Mengirim dan menerima perintah misi, seperti takeoff, landing, waypoint, dan mode penerbangan.
- Monitoring status drone (baterai, GPS, attitude, dll) secara real-time.
- Mengontrol aktuator dan sensor melalui topik ROS.
- Integrasi dengan berbagai sensor dan perangkat lunak lain di ekosistem ROS.

Jadi MAVROS memudahkan programming untuk integrasi sistem pada drone.

## Contoh Tipe-tipe Command MAVROS

Berikut beberapa contoh command MAVROS yang umum digunakan pada drone:

### Penjelasan Tipe Command/Service

- **CommandBool**: Service message untuk perintah boolean, misal arming/disarming drone (`value: true` untuk arming, `false` untuk disarming).
- **SetMode**: Service message untuk mengubah mode penerbangan drone, seperti GUIDED, OFFBOARD, dsb.
- **CommandTOL**: Service message untuk perintah TakeOff/Landing, berisi parameter seperti altitude, latitude, longitude, yaw, dsb.
- **GlobalPositionTarget**: Message untuk mengirim target posisi global (latitude, longitude, altitude) ke autopilot.
- **PoseStamped**: Message untuk mengirim target posisi lokal (x, y, z) dalam frame lokal.
- **TwistStamped**: Message untuk mengirim perintah kecepatan (velocity) linear dan angular ke drone.

1. **Set Mode (Guided/Offboard)**
	 - Mengubah mode penerbangan drone ke GUIDED (ArduPilot) atau OFFBOARD (PX4):
		 ```bash
		 ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "GUIDED"}'
		 # PX4
		 ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode '{custom_mode: "OFFBOARD"}'
		## Supported ROS distributions

		```

2. **Arming**
	 - Mengaktifkan (arming) motor drone:
		 ```bash
		 ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool '{value: true}'
		 ```

3. **Takeoff**
	 - Perintah takeoff otomatis:
		 ```bash
		 ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL '{altitude: 10.0}'
		 ```

4. **Menuju Waypoint (Global/Local/Velocity)**
	 - Menuju waypoint global:
		 ```bash
		 ros2 topic pub /mavros/setpoint_position/global mavros_msgs/msg/GlobalPositionTarget ...
		 ```
	 - Menuju waypoint lokal:
		 ```bash
		 ros2 topic pub /mavros/setpoint_position/local geometry_msgs/msg/PoseStamped ...
		 ```
	 - Kontrol kecepatan (velocity):
		 ```bash
		 ros2 topic pub /mavros/setpoint_velocity/cmd_vel geometry_msgs/msg/TwistStamped ...
		 ```

more command bisa lihat di mavros wiki [disini](https://docs.ros.org/en/humble/p/mavros/)

lets try another [disini](https://github.com/arilix/Drone-sim/blob/main/Drone_sim/Tutorial.md)





