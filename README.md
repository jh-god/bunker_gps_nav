# Bunker GPS Navigation

Bunker Pro 2.0을 위한 ROS 2 GPS 기반 내비게이션 패키지입니다. Dual RTK GNSS, wheel odometry, IMU로 위치와 방향을 추정하고, Patchwork++의 장애물 정보를 Nav2에 연결해 정적 지도 없이 단일 목표까지 주행합니다.

RViz의 2D Goal Pose와 위도·경도·방향 입력을 지원합니다.

## 설치 및 빌드

지원 환경은 Ubuntu 22.04 / ROS 2 Humble입니다. `OWNER/REPOSITORY`는 실제 GitHub 저장소 경로로 바꾸세요.

```bash
mkdir -p ~/bunker_gps_navigation_ws/src
cd ~/bunker_gps_navigation_ws
git clone https://github.com/jh-god/bunker_gps_nav.git 
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./src/bunker_gps_navigation/scripts/build.sh
source src/bunker_gps_navigation/scripts/env.sh
```

Git, ROS 2 Humble, colcon, rosdep이 설치되어 있어야 합니다. rosdep을 처음 사용하는 장비는 `sudo rosdep init`과 `rosdep update`를 먼저 실행합니다.

기본은 Release 빌드, 병렬 패키지 2개, `BUILD_TESTING=OFF`입니다. `BUNKER_BUILD_WORKERS`로 병렬 패키지 수를 조절할 수 있습니다.

### 소스 업데이트

실행 중인 navigation을 종료한 뒤 업데이트하고 다시 빌드합니다.

```bash
cd ~/bunker_gps_navigation_ws
git -C src/bunker_gps_navigation pull --ff-only
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
./src/bunker_gps_navigation/scripts/build.sh
source src/bunker_gps_navigation/scripts/env.sh
```

```text
~/bunker_gps_navigation_ws/
├── src/bunker_gps_navigation/
├── build/
├── install/
└── log/
```

아래 명령은 워크스페이스 루트에서 실행합니다. 드라이버가 별도 워크스페이스에 설치되어 있다면 그 환경도 source하세요. 이 패키지는 ROS_DOMAIN_ID, localhost 설정, DDS 구현을 강제로 바꾸지 않습니다. 드라이버 및 RViz와 같은 통신 설정을 사용하세요.

## 드라이버 연결

Bunker, SMC GNSS 두 대, WitMotion IMU, Ouster, Patchwork++를 기존 방식으로 실행합니다.

| 필수 입력 | 타입 / 조건 |
|---|---|
| `/odom` | `nav_msgs/msg/Odometry`, odom → base_link |
| `/imu` | `sensor_msgs/msg/Imu`, gyro 사용 |
| `/smc_2000/fix` | `sensor_msgs/msg/NavSatFix`, 왼쪽 base |
| `/smc_plus/fix` | `sensor_msgs/msg/NavSatFix`, 오른쪽 rover |
| `/smc_plus/relposned` | `ublox_msgs/msg/NavRELPOSNED9`, base→rover baseline heading |
| `/patchworkpp/nonground` | `sensor_msgs/msg/PointCloud2`, frame_id=base_link |
| TF | Bunker가 odom → base_link 발행, 기존 URDF/TF가 센서 연결 |

Ouster의 `/ouster/points` → Patchwork++ 연결은 기존 드라이버 설정에서 구성합니다. Navigation의 perception 입력은 `/patchworkpp/nonground`입니다.

주 설정은 `src/bunker_gps_navigation/bunker_gps_nav_bringup/config/system.yaml`입니다. `topics`에서 실차 토픽명을 맞출 수 있습니다. `drivers` 항목은 모두 기본 `enabled: false`여서 실행 중인 드라이버를 사용합니다. 함께 실행하려면 실제 설치된 `package`, `launch_file`, `arguments`를 채우고 해당 항목만 `enabled: true`로 설정하세요.

## Navigation 실행

```bash
cd ~/bunker_gps_navigation_ws
source src/bunker_gps_navigation/scripts/env.sh
ros2 launch bunker_gps_nav_bringup gps_navigation.launch.py
```

RViz를 같은 장비에서 띄우려면 `rviz:=true`를 추가합니다. Launch는 실제 시간(`use_sim_time=false`)을 사용합니다.

설정을 편집한 뒤 다시 빌드하거나 외부 YAML을 지정합니다.

```bash
ros2 launch bunker_gps_nav_bringup gps_navigation.launch.py \
  system_config:=/absolute/path/system.yaml \
  nav2_config:=/absolute/path/nav2_params.yaml
```

TF 구조는 다음과 같습니다.

```text
map                    global EKF
 └─ odom               bunker_ros2
     └─ base_link
         ├─ imu
         └─ os_sensor ── os_lidar
```

Local EKF는 `publish_tf=false`로 `/odometry/local`만 발행합니다. Global EKF만 `map → odom`을 발행합니다. 센서 static TF는 기존 URDF/드라이버를 사용합니다.

실행 상태 확인:

```bash
ros2 topic echo /navigation/motion_allowed --once
ros2 topic echo /diagnostics --once
ros2 lifecycle get /bt_navigator
ros2 run tf2_ros tf2_echo map base_link
ros2 topic hz /navigation/obstacles
```

실제 RTK FIX, GPS 변환, IMU/odom/cloud 입력과 Nav2 활성화가 준비되면 `motion_allowed`가 true가 됩니다. 센서가 없으면 원점을 만들지 않고 이동을 차단합니다.

## 목표 입력

RViz의 Fixed Frame을 `map`으로 설정하고 **2D Goal Pose**로 목표와 방향을 지정합니다. 제공 설정은 `bunker_gps_nav_nav2/rviz/gps_navigation.rviz`입니다. `/goal_pose`를 브리지가 NavigateToPose로 전달하며 Nav2 내부 구독은 별도 토픽으로 remap해 중복 전달을 막았습니다.

GPS 목표는 현장 좌표를 입력합니다.

```bash
ros2 run bunker_gps_nav_goal send_gps_goal \
  --lat <목표_위도> --lon <목표_경도> --heading <목표_방향_deg>
```

`<...>`를 실제 숫자로 바꾸세요. Heading은 북=0°, 동=90°, 시계방향 증가입니다. 선택 인자 `--alt`는 미터이며 기본값은 0입니다. `/gps_goal`에 `geographic_msgs/msg/GeoPoseStamped`를 발행할 수도 있으며 quaternion은 ENU 기준입니다.

준비 전 목표는 거부하므로 상태가 준비되면 다시 보내세요. 초기 주행 거리는 5–20 m, 브리지 거리 제한은 25 m입니다. 목표 허용오차는 위치 0.3 m, 방향 7°이며 단일 목표만 지원합니다.

## 위치와 heading 설정

- Local EKF: wheel 속도 + IMU gyro, 40 Hz. 자기계 yaw는 융합하지 않습니다.
- Global EKF: local 속도 + GNSS 위치 + dual heading. Local pose를 중복 융합하지 않습니다.
- 두 GNSS 위치는 ECEF에서 중앙점을 계산합니다. Fix timestamp 차이가 0.15초보다 크거나 유효한 FIX가 아니면 발행하지 않습니다.
- `datum.mode: auto`는 실제 첫 RTK FIX 중앙점을 원점으로 사용합니다. 실행 설정에 시험용 위경도는 없습니다.
- `datum.mode: manual`을 선택하면 `datum.latitude`, `datum.longitude`에 실측 좌표를 반드시 지정합니다. `datum.altitude`의 단위는 미터이며 생략 시 0입니다.
- Datum manager가 `/datum` 서비스로 원점을 지정하고, GPS 목표는 동일한 `/fromLL` 변환을 사용합니다.
- `navsat_transform`의 odometry 입력은 global EKF입니다. Humble 3.5.4의 좌표 갱신과 일관되도록 UTM 경로(`use_local_cartesian=false`)와 자오선 수렴각 보정을 사용합니다.

왼쪽 base → 오른쪽 rover baseline이므로 `heading_mount_offset_deg` 기본값은 **+90.0°**입니다.

```text
yaw_ros = π/2 − heading_baseline + heading_mount_offset
```

차량이 북쪽이면 baseline은 동쪽(90°)이고 보정 후 ROS yaw는 +π/2입니다. 차량이 동쪽이면 baseline은 남쪽(180°)이고 보정 후 yaw는 0입니다. 미세 조정은 `system.yaml`에서 합니다.

RTK는 `NavRELPOSNED9`의 비트 상수로 FIXED/FLOAT/INVALID를 판정합니다. 이 메시지에는 Header가 없어 heading timestamp는 수신 시각을 사용하며, 반복되는 동일 `i_tow`는 버립니다.

## 장애물과 차체 영역

`/patchworkpp/nonground`는 이미 `base_link` 좌표여서 추가 TF 변환이 필요하지 않습니다. 다른 frame으로 입력되면 해당 TF를 먼저 적용합니다.

```text
remove non-finite → self-body exclusion → height → range → voxel downsample
```

차체 내부와 박스 경계면의 점을 제거합니다. 설정 파일은 `bunker_gps_nav_perception/config/obstacle_filter.yaml`입니다.

| 항목 | 초기값 |
|---|---|
| 차체 X 범위 | −0.65 ~ +0.65 m |
| 차체 Y 범위 | −0.45 ~ +0.45 m |
| 차체 Z 범위 | −0.30 ~ +0.60 m |
| 장애물 높이 | 0.05 ~ 1.5 m |
| 수평거리 | 0.3 ~ 50 m |
| Voxel 크기 | 0.10 m |

차체 박스와 현재 1.30 × 0.90 m footprint는 실측에 맞춰 조정해야 합니다. 필터 진단에는 점 수, 처리 시간, 주파수와 각 단계 제거 개수를 제공합니다.

Nav2는 global 60×60 m / 0.15 m, local 20×20 m / 0.05 m rolling costmap을 사용합니다. Planner는 SmacPlanner2D, controller는 RPP이며 1 Hz로 재계획합니다. 정적 지도, AMCL, SLAM은 사용하지 않습니다.

Nonground만으로는 빈 광선 정보가 부족해 이동 장애물의 잔상이 남을 수 있습니다. 현장 관측 후 clearing 입력을 보완할 수 있습니다. 미관측 공간은 free로 취급하므로 센서가 관측하는 짧은 거리의 시험을 대상으로 합니다.

## 속도와 상태 감시

```text
Nav2 → /navigation/cmd_vel_raw → supervisor → /cmd_vel → Bunker
```

- FIXED: 최대 0.6 m/s.
- FLOAT: 최대 0.3 m/s로 약 3초 유지한 뒤 정지·목표 취소.
- INVALID: 정지·목표 취소.
- Heading 단절 4초, GNSS/odom/IMU/cloud/localization timeout, Nav2 비활성화도 이동 차단.
- 속도 명령이 0.5초 동안 갱신되지 않으면 0으로 출력합니다.

설정은 `bunker_gps_nav_safety/config/safety.yaml`에 있습니다. 다른 publisher가 직접 `/cmd_vel`을 발행하면 supervisor를 우회하므로 기존 teleop/mux 연결을 맞추세요. 드라이버의 명령 timeout과 하드웨어 비상정지는 별도로 유지합니다.

## 패키지 구성 및 테스트

| 패키지 | 역할 |
|---|---|
| bunker_gps_nav_localization | GNSS 처리, datum, 두 EKF |
| bunker_gps_nav_perception | 차체·장애물 필터 |
| bunker_gps_nav_goal | GPS/RViz 목표와 CLI |
| bunker_gps_nav_safety | 상태 감시, 최종 속도 제한 |
| bunker_gps_nav_nav2 | Nav2 YAML, BT, RViz |
| bunker_gps_nav_bringup | 통합/단계별 launch |

단계별 실행은 `localization.launch.py`, `perception.launch.py`, `navigation.launch.py`입니다. 모두 실제 센서와 기존 TF를 사용합니다.

단위 테스트 실행:

```bash
BUNKER_BUILD_TESTING=ON ./src/bunker_gps_navigation/scripts/build.sh
source src/bunker_gps_navigation/scripts/env.sh
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --verbose
```

빌드·테스트 결과와 검증 범위는 [검증 기록](VALIDATION.md)을 참고하세요.
