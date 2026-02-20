# VTOL Division Internship - MAVROS

## MAVROS..?

MAVROS adalah sebuah paket ROS (Robot Operating System) yang berfungsi sebagai jembatan komunikasi antara sistem operasi robot (ROS) dengan autopilot berbasis protokol MAVLink, seperti PX4 atau ArduPilot. Dengan MAVROS, pengguna dapat mengontrol, memonitor, dan mengintegrasikan drone atau kendaraan otonom lain ke dalam ekosistem ROS dengan mudah.

## How to Install MAVROS 
untuk install mavros bisa lewat sini
[Install Mavros](https://docs.ros.org/en/humble/p/mavros/)



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





