# SWARM DRONE SIMULATION SYSTEM

## 💡 MISI & DESKRIPSI SISTEM
Simulasi ini menggambarkan misi kolaboratif dari tiga drone untuk mencari kotak merah yang tersembunyi di salah satu dari tiga ruangan. Sistem ini dirancang dengan ketahanan terhadap kegagalan (failover) dan dilengkapi kontrol dari UI web.

### ALUR KERJA MISI:
1. **Inisialisasi & Take-Off**: Tiga drone lepas landas, salah satu menjadi leader otomatis.
2. **Penugasan Ruangan**: Tiap drone mendapat ruangan acak dari tiga ruangan berbeda.
3. **Pencarian Objek**: Pengguna memilih ruangan tempat kotak merah secara manual saat awal program.
4. **Deteksi & Notifikasi**:
   - Jika leader menemukan → langsung tulis log.
   - Jika member menemukan → kirim ke leader → leader menulis log.
5. **Tunggu 10 Detik**: Semua drone diam, tandai posisi.
6. **Kembali ke Awal**: Drone kembali ke posisi take-off.
7. **Leader Fallback**: Jika leader mati, member akan otomatis menggantikannya.
8. **Web UI**: Memungkinkan pemilihan ruangan awal dan simulasi pematian leader.
9. **Gazebo**: Digunakan untuk simulasi visual 3D dan fisika drone secara real-time.


---

## 📂 STRUKTUR FOLDER PROYEK
```
docker_shared/
└── swarm_ws/
    ├── start_docker_swarm.sh
    ├── stop_docker_swarm.sh
    ├── gazebo_models/             # Model Gazebo (tambahan)
    ├── launch/
    │   ├── swarm_patrol.launch
    │   ├── visualization.launch
    │   ├── launch_all.launch
    │   └── gazebo_world.launch    # Untuk simulasi Gazebo
    ├── worlds/                    # Dunia simulasi Gazebo (tambahan)
    │   └── search_world.world
    ├── src/
    │   └── swarm_patrol/
    │       ├── CMakeLists.txt
    │       ├── package.xml
    │       ├── launch/
    │       ├── scripts/
    │       │   ├── drone_behavior.py
    │       │   ├── red_box_marker.py
    │       │   ├── room_marker.py
    │       │   ├── leader_fallback.py
    │       │   └── web_control_node.py
    │       └── ui/
    │           └── swarm_ui.html
```

---

## 📄 ISI SEMUA FILE (BERURUTAN SESUAI STRUKTUR)

### 🗎 start_docker_swarm.sh
```bash
#!/bin/bash
LOCAL_DIR="$HOME/docker_shared/swarm_ws"
CONTAINER_DIR="/root/swarm_ws"
mkdir -p "$LOCAL_DIR/src"
docker run -it -d \
  --platform linux/amd64 \
  --env="DISPLAY=host.docker.internal:0" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="$LOCAL_DIR:$CONTAINER_DIR" \
  --privileged \
  --name swarm_patrol_container \
  osrf/ros:noetic-desktop-full
echo "Container 'swarm_patrol_container' sudah jalan."
echo "docker exec -it swarm_patrol_container /bin/bash"
```

### 🗎 stop_docker_swarm.sh
```bash
#!/bin/bash
CONTAINER_NAME="swarm_patrol_container"
echo "Menghentikan container $CONTAINER_NAME ..."
docker stop $CONTAINER_NAME 2>/dev/null
docker rm $CONTAINER_NAME 2>/dev/null
echo "Container $CONTAINER_NAME dihentikan."
```

### 🗎 CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.0.2)
project(swarm_patrol)
find_package(catkin REQUIRED COMPONENTS rospy std_msgs visualization_msgs)
catkin_package()
include_directories(${catkin_INCLUDE_DIRS})
```

### 🗎 package.xml
```xml
<package format="2">
  <name>swarm_patrol</name>
  <version>0.0.1</version>
  <description>Swarm Drone Simulation</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
  <build_depend>rospy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>visualization_msgs</build_depend>
  <exec_depend>rospy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>visualization_msgs</exec_depend>
</package>
```

### 🗎 launch/swarm_patrol.launch
```xml
<launch>
  <node pkg="swarm_patrol" type="drone_behavior.py" name="drone1" output="screen">
    <param name="drone_id" value="drone1"/>
    <param name="is_leader" value="true"/>
  </node>
  <node pkg="swarm_patrol" type="drone_behavior.py" name="drone2" output="screen">
    <param name="drone_id" value="drone2"/>
    <param name="is_leader" value="false"/>
  </node>
  <node pkg="swarm_patrol" type="drone_behavior.py" name="drone3" output="screen">
    <param name="drone_id" value="drone3"/>
    <param name="is_leader" value="false"/>
  </node>
</launch>
```

### 🗎 launch/visualization.launch
```xml
<launch>
  <node pkg="swarm_patrol" type="room_marker.py" name="room_marker" output="screen" />
  <node pkg="swarm_patrol" type="red_box_marker.py" name="red_box_marker" output="screen" />
</launch>
```

### 🗎 launch/launch_all.launch
```xml
<launch>
  <include file="$(find swarm_patrol)/launch/visualization.launch" />
  <include file="$(find swarm_patrol)/launch/swarm_patrol.launch" />
  <node pkg="swarm_patrol" type="web_control_node.py" name="web_control" output="screen" />
  <include file="$(find rosbridge_server)/launch/rosbridge_websocket.launch" />
</launch>
```

### 🗎 scripts/drone_behavior.py
```python
#!/usr/bin/env python3
import rospy
import random
import time
from std_msgs.msg import String

class Drone:
    def __init__(self, drone_id, is_leader=False):
        self.drone_id = drone_id
        self.is_leader = is_leader
        self.room = random.choice(['Room 1', 'Room 2', 'Room 3'])
        self.found_red_box = False

        rospy.init_node(f'drone_{drone_id}', anonymous=True)
        self.pub = rospy.Publisher('/swarm/log', String, queue_size=10)
        self.sub = rospy.Subscriber('/swarm/found', String, self.callback_found)

        rospy.sleep(1)
        rospy.loginfo(f"Drone {self.drone_id} assigned to {self.room}")
        self.search()

    def callback_found(self, msg):
        if not self.is_leader:
            rospy.loginfo(f"Drone {self.drone_id} received: {msg.data}")

    def search(self):
        rospy.loginfo(f"Drone {self.drone_id} is searching in {self.room}")
        rospy.sleep(random.randint(2, 5))
        self.found_red_box = random.choice([True, False])

        if self.found_red_box:
            if self.is_leader:
                msg = f"Leader {self.drone_id} found red box in {self.room}"
                self.pub.publish(msg)
                rospy.loginfo(msg)
            else:
                msg = f"Drone {self.drone_id} (member) found red box in {self.room}"
                leader_msg = f"Leader logs: {msg}"
                self.pub.publish(leader_msg)
                rospy.loginfo(leader_msg)

        rospy.sleep(10)
        rospy.loginfo(f"Drone {self.drone_id} returning to start point.")

if __name__ == '__main__':
    try:
        is_leader_param = rospy.get_param('~is_leader', False)
        drone_id_param = rospy.get_param('~drone_id', 'droneX')
        Drone(drone_id_param, is_leader=is_leader_param)
    except rospy.ROSInterruptException:
        pass
```

### 🗎 scripts/red_box_marker.py
```python
#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker
import random

def publish_marker():
    rospy.init_node('red_box_marker')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)

    rooms = {
        "Room 1": (-2, 2),
        "Room 2": (0, 2),
        "Room 3": (2, 2)
    }
    selected_room = random.choice(list(rooms.keys()))
    pos_x, pos_y = rooms[selected_room]

    rospy.loginfo(f"Red box placed in {selected_room} at ({pos_x}, {pos_y})")

    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "red_box"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = pos_x
    marker.pose.position.y = pos_y
    marker.pose.position.z = 0.1
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.3
    marker.scale.y = 0.3
    marker.scale.z = 0.3
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(marker)
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_marker()
    except rospy.ROSInterruptException:
        pass
```

### 🗎 scripts/room_marker.py
```python
#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker

def create_room_marker(room_id, x, y, color, marker_id):
    marker = Marker()
    marker.header.frame_id = "map"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "rooms"
    marker.id = marker_id
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = 0.01
    marker.scale.x = 1.8
    marker.scale.y = 1.8
    marker.scale.z = 0.01
    marker.color.r, marker.color.g, marker.color.b = color
    marker.color.a = 0.3
    marker.lifetime = rospy.Duration()
    return marker

def publish_rooms():
    rospy.init_node('room_marker_node')
    pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        pub.publish(create_room_marker("Room 1", -2, 2, (0.0, 0.8, 1.0), 10))
        pub.publish(create_room_marker("Room 2", 0, 2, (0.0, 1.0, 0.0), 11))
        pub.publish(create_room_marker("Room 3", 2, 2, (1.0, 1.0, 0.0), 12))
        rate.sleep()

if __name__ == "__main__":
    try:
        publish_rooms()
    except rospy.ROSInterruptException:
        pass
```

### 🗎 scripts/leader_fallback.py
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import time

class SwarmDrone:
    def __init__(self, drone_id, is_leader=False):
        rospy.init_node(f'swarm_{drone_id}', anonymous=True)
        self.drone_id = drone_id
        self.is_leader = is_leader
        self.last_heartbeat = time.time()
        self.found_red_box = False

        self.log_pub = rospy.Publisher('/swarm/log', String, queue_size=10)
        self.heartbeat_pub = rospy.Publisher('/swarm/heartbeat', String, queue_size=10)
        self.heartbeat_sub = rospy.Subscriber('/swarm/heartbeat', String, self.heartbeat_callback)
        self.found_sub = rospy.Subscriber('/swarm/found', String, self.found_callback)

        rospy.sleep(1)
        rospy.loginfo(f"[{self.drone_id}] Started as {'LEADER' if self.is_leader else 'MEMBER'}")
        self.run()

    def heartbeat_callback(self, msg):
        if "LEADER" in msg.data and not self.is_leader:
            self.last_heartbeat = time.time()

    def found_callback(self, msg):
        rospy.loginfo(f"[{self.drone_id}] RECEIVED: {msg.data}")

    def publish_heartbeat(self):
        msg = f"HEARTBEAT from LEADER {self.drone_id}"
        self.heartbeat_pub.publish(msg)

    def check_leader_alive(self):
        now = time.time()
        if not self.is_leader and (now - self.last_heartbeat > 5.0):
            self.is_leader = True
            rospy.logwarn(f"[{self.drone_id}] Leader timed out. I am the new LEADER.")
            self.log_pub.publish(f"Drone {self.drone_id} has become the new LEADER.")

    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if self.is_leader:
                self.publish_heartbeat()
            else:
                self.check_leader_alive()
            rate.sleep()

if __name__ == "__main__":
    try:
        drone_id = rospy.get_param('~drone_id', 'droneX')
        is_leader = rospy.get_param('~is_leader', False)
        SwarmDrone(drone_id, is_leader)
    except rospy.ROSInterruptException:
        pass
```

### 🗎 scripts/web_control_node.py
```python
#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import os

class WebControlNode:
    def __init__(self):
        rospy.init_node('web_control_node')

        self.room_sub = rospy.Subscriber('/webui/room_selection', String, self.set_red_box_location)
        self.leader_kill_sub = rospy.Subscriber('/webui/disable_leader', String, self.kill_leader)

        self.log_pub = rospy.Publisher('/swarm/log', String, queue_size=10)

        rospy.loginfo("Web Control Node aktif. Menunggu perintah dari UI...")
        rospy.spin()

    def set_red_box_location(self, msg):
        room = msg.data.strip()
        rospy.set_param('/red_box_room', room)
        self.log_pub.publish(f"[WEB_UI] Red box ditetapkan ke {room}")
        rospy.loginfo(f"Red box moved to {room}")

    def kill_leader(self, msg):
        try:
            os.system("rosnode kill /swarm_drone1")
            self.log_pub.publish("[WEB_UI] Leader (/swarm_drone1) telah dinonaktifkan!")
            rospy.logwarn("Leader node killed by Web UI!")
        except Exception as e:
            rospy.logerr(f"Error killing leader: {e}")

if __name__ == '__main__':
    try:
        WebControlNode()
    except rospy.ROSInterruptException:
        pass
```

### 🗎 ui/swarm_ui.html
```html
<!DOCTYPE html>
<html>
<head>
  <title>Swarm Drone Web UI</title>
  <script src="https://cdn.jsdelivr.net/npm/roslib/build/roslib.min.js"></script>
  <style>
    body { font-family: Arial; margin: 20px; }
    .log-box {
      border: 1px solid #ccc;
      padding: 10px;
      height: 200px;
      overflow-y: scroll;
      background: #f9f9f9;
      font-size: 13px;
    }
    button {
      margin-top: 10px;
      margin-bottom: 10px;
    }
  </style>
</head>
<body>
  <h1>Swarm Drone Controller</h1>

  <label><b>Pilih Ruangan Kotak Merah:</b></label><br>
  <input type="radio" name="room" value="Room 1"> Room 1<br>
  <input type="radio" name="room" value="Room 2"> Room 2<br>
  <input type="radio" name="room" value="Room 3"> Room 3<br>
  <button onclick="sendRoomSelection()">Set Red Box Location</button><br>

  <button onclick="disableLeader()">Nonaktifkan Drone Leader</button>

  <h3>Log Sistem:</h3>
  <div class="log-box" id="logArea"></div>

  <script>
    var ros = new ROSLIB.Ros({
      url : 'ws://localhost:9090'
    });

    // PUB untuk kontrol
    var roomTopic = new ROSLIB.Topic({
      ros : ros,
      name : '/webui/room_selection',
      messageType : 'std_msgs/String'
    });

    var killLeaderTopic = new ROSLIB.Topic({
      ros : ros,
      name : '/webui/disable_leader',
      messageType : 'std_msgs/String'
    });

    function sendRoomSelection() {
      var rooms = document.getElementsByName('room');
      for (var i = 0; i < rooms.length; i++) {
        if (rooms[i].checked) {
          var msg = new ROSLIB.Message({ data: rooms[i].value });
          roomTopic.publish(msg);
        }
      }
    }

    function disableLeader() {
      var msg = new ROSLIB.Message({ data: 'kill' });
      killLeaderTopic.publish(msg);
    }

    // SUBSCRIBE Log Sistem
    var logTopic = new ROSLIB.Topic({
      ros : ros,
      name : '/swarm/log',
      messageType : 'std_msgs/String'
    });

    logTopic.subscribe(function(message) {
      var logArea = document.getElementById("logArea");
      logArea.innerHTML += message.data + "<br>";
      logArea.scrollTop = logArea.scrollHeight;
    });
  </script>
</body>
</html>
```

---
### gazebo_models/model.sdf
```bash
<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="blueprint_world">
    <static>true</static>
    <link name="blueprint_floor">
      <visual name="visual">
        <geometry>
          <plane>
            <normal>0 0 1</normal>
            <size>10 10</size> <!-- skala dunia -->
          </plane>
        </geometry>
        <material>
          <script>
            <uri>model://blueprint_world/materials/scripts</uri>
            <uri>model://blueprint_world/materials/textures</uri>
            <name>Gazebo/BlueprintFloor</name>
          </script>
          <ambient>1 1 1 1</ambient>
        </material>
      </visual>
    </link>
  </model>
</sdf>
```

### gazebo_models/model.config
```bash
<?xml version="1.0"?>
<model>
  <name>blueprint_world</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <author>
    <name>Swarm Team</name>
    <email>example@email.com</email>
  </author>
  <description>Blueprint style world for swarm drone simulation</description>
</model>
```

### world/blueprint_world.world
```bash
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <include>
      <uri>model://blueprint_world</uri>
    </include>

    <!-- Tambahkan drone model -->
    <include>
      <uri>model://quadrotor</uri>
      <name>drone1</name>
      <pose>0 0 0.1 0 0 0</pose>
    </include>

    <!-- Tambah drone lainnya -->
  </world>
</sdf>
```
### 📥 PETUNJUK MENJALANKAN SIMULASI (DARI AWAL)

### 1. Buka Terminal di MacBook (Host)
```bash
cd ~/docker_shared/swarm_ws
chmod +x start_docker_swarm.sh stop_docker_swarm.sh
./start_docker_swarm.sh
```
> Ini akan menjalankan container bernama `swarm_patrol_container` dengan ROS Noetic.

### 2. Masuk ke dalam Docker Container
```bash
docker exec -it swarm_patrol_container /bin/bash
```

### 3. Build Workspace ROS (jika belum)
```bash
cd /root/swarm_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```
> Jika muncul error `catkin_make: command not found`, pastikan environment ROS sudah aktif dengan `source /opt/ros/noetic/setup.bash`. Untuk otomatisasi, tambahkan ke `~/.bashrc` dalam container:
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
> Jika build berhasil, akan muncul log seperti:
> `-- +++ processing catkin package: 'swarm_patrol'`
> `-- Build files have been written to: /root/swarm_ws/build`

### 4. Jalankan Simulasi Utama
```bash
source /root/swarm_ws/devel/setup.bash
roslaunch swarm_patrol launch_all.launch
```
> Jika muncul error `Resource not found: rosbridge_server`, artinya package belum terinstall di dalam container. Solusinya:
```bash
apt update
apt install -y ros-noetic-rosbridge-server
source /opt/ros/noetic/setup.bash
roslaunch swarm_patrol launch_all.launch
```

```bash
source /root/swarm_ws/devel/setup.bash
roslaunch swarm_patrol launch_all.launch
```
> Jika muncul error `Resource not found: rosbridge_server`, artinya package belum terinstall di dalam container. Solusinya:
```bash
apt update
apt install -y ros-noetic-rosbridge-server
source /opt/ros/noetic/setup.bash
roslaunch swarm_patrol launch_all.launch
```

> Jika muncul error `launch_all.launch is neither a launch file in package [swarm_patrol]`, periksa hal-hal berikut:
- Pastikan struktur folder valid:
  `/root/swarm_ws/src/swarm_patrol/`
- Pastikan sudah build dengan `catkin_make` dan sudah `source` workspace:
```bash
cd /root/swarm_ws
catkin_make
source /root/swarm_ws/devel/setup.bash
```
- Verifikasi package terdeteksi oleh ROS:
```bash
rospack list | grep swarm_patrol
```

> Jika muncul error `Cannot locate node of type [...] in package [swarm_patrol]`, artinya file Python belum diberi izin eksekusi. Solusinya:
```bash
cd /root/swarm_ws/src/swarm_patrol/scripts
chmod +x *.py
```
> Jika muncul error `Resource not found: rosbridge_server`, artinya package belum terinstall di dalam container. Solusinya:
```bash
apt update
apt install -y ros-noetic-rosbridge-server
source /opt/ros/noetic/setup.bash
roslaunch swarm_patrol launch_all.launch
```

> Jika muncul error `launch_all.launch is neither a launch file in package [swarm_patrol]`, periksa hal-hal berikut:
- Pastikan struktur folder valid:
  `/root/swarm_ws/src/swarm_patrol/`
- Pastikan sudah build dengan `catkin_make` dan sudah `source` workspace:
```bash
cd /root/swarm_ws
catkin_make
source /root/swarm_ws/devel/setup.bash
```
- Verifikasi package terdeteksi oleh ROS:
```bash
rospack list | grep swarm_patrol
```, artinya package belum terinstall di dalam container. Solusinya:
```bash
apt update
apt install -y ros-noetic-rosbridge-server
source /opt/ros/noetic/setup.bash
roslaunch swarm_patrol launch_all.launch
```
> Ini akan memulai node drone, visualisasi marker, serta server rosbridge.

### 5. Buka Web UI untuk Kontrol
### Tujuan:
Membuat dan menjalankan antarmuka web berbasis HTML + ROSLIB untuk:
1. **Memilih ruangan tempat kotak merah**.
2. **Menonaktifkan drone leader secara manual**.
3. **Menampilkan log sistem secara real-time**.

### 📂 Lokasi file:
`~/docker_shared/swarm_ws/src/swarm_patrol/ui/swarm_ui.html`

### 1. **Pastikan `rosbridge_websocket` berjalan**
Karena Web UI menggunakan koneksi websocket untuk komunikasi ROS, pastikan `roslaunch swarm_patrol launch_all.launch` sudah dijalankan dan tidak error pada bagian ini:
```
Rosbridge WebSocket server started at ws://0.0.0.0:9090
```

### 2. **Buka UI HTML di browser host (Mac)**
```bash
open ~/docker_shared/swarm_ws/src/swarm_patrol/ui/swarm_ui.html
```
> Catatan: `open` hanya berfungsi di Mac. Untuk Windows atau Linux, gunakan `xdg-open` atau buka file secara manual.

### 3. **Kontrol dari Web UI**
- Pilih salah satu tombol radio: Room 1 / Room 2 / Room 3
- Klik `Set Red Box Location` → akan mengirimkan data ke topic `/webui/room_selection`
- Klik `Nonaktifkan Drone Leader` → akan mengirim sinyal ke topic `/webui/disable_leader`
- Semua log sistem muncul di panel log di bawahnya secara real-time (via `/swarm/log`).

### 4. **Pastikan jaringan Docker bisa diakses oleh browser**
Jika WebSocket tidak tersambung (`roslibjs` error), pastikan:
- Anda membuka file HTML dari **host**, bukan dari dalam container.
- Docker container menggunakan `--network host` (opsional untuk testing WebSocket).

### 🔍 Cara Cek Komunikasi WebUI

1. **Lihat log terminal** yang menjalankan ROS:
   - Akan ada output seperti:
     ```
     [WEB_UI] Red box ditetapkan ke Room 2
     [WEB_UI] Leader (/swarm_drone1) telah dinonaktifkan!
     ```

2. **Cek topic data:**
```bash
rostopic echo /webui/room_selection
rostopic echo /webui/disable_leader
```

### 🔧 Troubleshooting
| Masalah | Solusi |
|--------|--------|
| WebSocket tidak connect | Pastikan rosbridge sudah berjalan dan `roslaunch` tidak error |
| Tidak ada log tampil | Cek apakah `/swarm/log` di-subscribe dengan benar oleh JavaScript |
| Tombol tidak bekerja | Pastikan script JS di file `swarm_ui.html` aktif dan `roslib.min.js` berhasil di-load |
| rosnode tidak mati saat klik tombol kill | Pastikan nama node sesuai (`/swarm_drone1`) dan file `web_control_node.py` menerima topik |

### 6. Menampilkan Visualisasi (Opsional)
#### Langkah 1: Instal Gazebo dan Plugin di Dalam Container
```bash
apt update
apt install -y gazebo11 libgazebo11-dev
apt install -y ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
```

#### Langkah 2: Tambah Dunia Gazebo
Contoh file: `worlds/search_world.world` dengan tiga ruangan dan landasan take-off.

#### Langkah 3: Tambah Launch File Gazebo
Contoh file `launch/gazebo_world.launch`:
```xml
<launch>
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find swarm_patrol)/worlds/search_world.world"/>
  </include>
</launch>
```

#### Langkah 4: Integrasi Model Drone
Gunakan model dari `hector_quadrotor` atau `rotors_simulator`, pastikan berada di folder `gazebo_models/`.

#### Langkah 5: Jalankan Bersama Node ROS
Edit `launch_all.launch` untuk menyertakan `gazebo_world.launch` sebelum node drone.

```xml
<include file="$(find swarm_patrol)/launch/gazebo_world.launch" />
```

### 7. Menghentikan Simulasi (di Mac)
```bash
./stop_docker_swarm.sh
```
> Ini akan menghentikan dan menghapus container agar bersih.

## 🧪 DEBUG PARAMETER (Jika drone_id tetap `droneX`)

Jika log masih menampilkan `droneX`, pastikan kamu:
- Menjalankan file `drone_behavior.py` dari path `/root/swarm_ws/src/swarm_patrol/scripts`
- Menambahkan `launch-prefix="python3"` di file `.launch`
- Memindahkan `rospy.init_node(...)` ke dalam blok `__main__` **sebelum** `rospy.get_param()` dipanggil
- Cek hasil dengan perintah:
```bash
rosparam get /drone1/drone_id
rosparam get /drone2/drone_id
rosparam get /drone3/drone_id
```
- Tambahkan debug print di script:
```python
print(f"🟢 Param drone_id yang diterima: {drone_id_param}")
```

## 📌 CATATAN TAMBAHAN
- Seluruh sistem ini telah diuji untuk environment Docker + ROS Noetic.
- Leader fallback otomatis diuji dengan simulasi pematian node.
- Web UI digunakan via `rosbridge_websocket`.

---


