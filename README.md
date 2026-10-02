# Bunker GPS Navigation

Bunker Pro 2.0을 위한 ROS 2 GPS 기반 내비게이션 패키지입니다. Dual RTK GNSS, wheel odometry, IMU로 위치와 방향을 추정하고, Patchwork++의 장애물 정보를 Nav2에 연결해 정적 지도 없이 단일 목표까지 주행합니다.

RViz의 **2D Goal Pose**와 **위도·경도·방향 입력**을 지원합니다.

## 설치 및 빌드

### 준비 사항

- 지원 환경: Ubuntu 22.04 / ROS 2 Humble
- 필요 도구: Git, colcon, rosdep
- rosdep 최초 사용 시: `sudo rosdep init` → `rosdep update`
- 명령 실행 기준: `~/bunker_gps_navigation_ws`

### 저장소 복제 및 빌드

```bash
mkdir -p ~/bunker_gps_navigation_ws/src
cd ~/bunker_gps_navigation_ws/src
git clone https://github.com/jh-god/bunker_gps_nav.git
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source install/setup.bash
```

### 소스 업데이트
- 소스 갱신 → 의존성 설치 → 재빌드 → 환경 적용 순서
```bash
cd ~/bunker_gps_navigation_ws/src
git -C pull --ff-only
```

## 드라이버 연결 및 실행

### 드라이버 준비

- Bunker, SMC GNSS 두 대, WitMotion IMU, Ouster, Patchwork++를 기존 방식으로 실행
- 별도 드라이버 워크스페이스 사용 시 해당 환경도 source
- 드라이버·navigation·RViz의 ROS_DOMAIN_ID, localhost 설정, DDS 구현을 일치시킴
- Ouster `/ouster/points` → Patchwork++ 연결은 기존 드라이버 설정에서 구성

| 필수 입력 | 메시지 타입 | 조건 |
|---|---|---|
| `/odom` | `nav_msgs/msg/Odometry` | odom → base_link |
| `/imu` | `sensor_msgs/msg/Imu` | gyro 사용 |
| `/smc_2000/fix` | `sensor_msgs/msg/NavSatFix` | 왼쪽 base |
| `/smc_plus/fix` | `sensor_msgs/msg/NavSatFix` | 오른쪽 rover |
| `/smc_plus/relposned` | `ublox_msgs/msg/NavRELPOSNED9` | base→rover baseline heading |
| `/patchworkpp/nonground` | `sensor_msgs/msg/PointCloud2` | frame_id=base_link |
| TF | — | Bunker가 odom → base_link 발행, 기존 URDF/TF로 센서 연결 |

주 설정: [system.yaml](bunker_gps_nav_bringup/config/system.yaml)

- `topics`: 센서와 제어 토픽명 설정
- `drivers.*.enabled: false`: 별도로 실행 중인 드라이버 사용 — 기본값
- 드라이버 통합 실행 방법:
  1. 설치된 드라이버의 `package`, `launch_file`, `arguments` 입력
  2. 해당 항목의 `enabled`를 `true`로 변경

### Navigation 실행

```bash
cd ~/bunker_gps_navigation_ws
source install/setup.bash
ros2 launch bunker_gps_nav_bringup gps_navigation.launch.py
```

- RViz 동시 실행: 명령 뒤에 `rviz:=true` 추가
- 사용 시간: 실제 시간 (`use_sim_time=false`)
- 소스 YAML 수정 후: 다시 빌드해 설치 경로에 반영
- 외부 YAML 사용 시: 아래 인자로 경로 지정

```bash
ros2 launch bunker_gps_nav_bringup gps_navigation.launch.py \
  system_config:=/absolute/path/system.yaml \
  nav2_config:=/absolute/path/nav2_params.yaml
```

### 실행 상태 확인

| 확인 항목 | 명령 |
|---|---|
| 이동 허용 여부 | `ros2 topic echo /navigation/motion_allowed --once` |
| 진단 정보 | `ros2 topic echo /diagnostics --once` |
| Nav2 활성화 | `ros2 lifecycle get /bt_navigator` |
| TF 연결 | `ros2 run tf2_ros tf2_echo map base_link` |
| 장애물 출력 주기 | `ros2 topic hz /navigation/obstacles` |

- 이동 허용 조건: RTK FIX, GPS 변환, IMU/odom/cloud 입력, Nav2 활성화 준비 완료
- 준비 완료 시: `/navigation/motion_allowed`가 `true`
- 센서 미연결 시: 원점 초기화 대기 및 이동 차단

## 목표 입력

### RViz

1. Fixed Frame을 `map`으로 설정
2. **2D Goal Pose**로 목표 위치와 방향 지정

- 제공 설정: [gps_navigation.rviz](bunker_gps_nav_nav2/rviz/gps_navigation.rviz)
- 전달 경로: `/goal_pose` → goal bridge → `NavigateToPose`
- 중복 전달 방지: Nav2 내부 goal 구독은 별도 토픽으로 remap

### GPS 좌표

```bash
ros2 run bunker_gps_nav_goal send_gps_goal \
  --lat <목표_위도> --lon <목표_경도> --heading <목표_방향_deg>
```

- `<...>`를 실제 숫자로 교체
- Heading 기준: 북=0°, 동=90°, 시계방향 증가
- 선택 인자 `--alt`: 고도(m), 기본값 0
- 메시지 직접 입력:
  - 토픽: `/gps_goal`
  - 타입: `geographic_msgs/msg/GeoPoseStamped`
  - 방향 quaternion: ENU 기준

### 목표 조건

| 항목 | 설정 |
|---|---|
| 지원 방식 | 단일 목표 |
| 초기 시험 거리 | 5–20 m |
| 브리지 거리 제한 | 25 m |
| 위치 허용오차 | 0.3 m |
| 방향 허용오차 | 7° |

- 준비 전 목표는 거부됨 — 이동 허용 상태를 확인한 뒤 다시 전송

## 위치 추정 및 TF

### 데이터 융합

| 구성 요소 | 입력 / 역할 |
|---|---|
| Local EKF | Wheel 속도 + IMU gyro, 40 Hz |
| Global EKF | Local 속도 + GNSS 위치 + dual heading |
| GNSS midpoint | 두 안테나 위치를 ECEF에서 중앙점으로 변환 |
| Datum manager | `/datum` 서비스로 지도 원점 설정 |
| GPS goal bridge | `/fromLL`로 위치 추정과 동일한 좌표 변환 사용 |

- 자기계 yaw는 융합하지 않음
- Global EKF에 local pose를 중복 융합하지 않음
- GNSS 중앙점 발행 조건:
  - 유효한 RTK FIX
  - 두 fix의 timestamp 차이 ≤ 0.15초
- `navsat_transform`의 odometry 입력: global EKF
- Humble 3.5.4 좌표 변환: UTM 경로 (`use_local_cartesian=false`) + 자오선 수렴각 보정

### 지도 원점

| `datum.mode` | 동작 | 필수 설정 |
|---|---|---|
| `auto` | 첫 RTK FIX 중앙점을 원점으로 사용 | 없음 |
| `manual` | 지정한 실측 좌표를 원점으로 사용 | `datum.latitude`, `datum.longitude` |

- 수동 원점의 `datum.altitude`: 단위 m, 생략 시 0
- 설정 위치: [system.yaml](bunker_gps_nav_bringup/config/system.yaml)

### TF 구조와 발행 주체

```text
map
 └─ odom
     └─ base_link
         ├─ imu
         └─ os_sensor ── os_lidar
```

| 발행 항목 | 담당 |
|---|---|
| `map → odom` | Global EKF |
| `odom → base_link` | bunker_ros2 |
| 센서 static TF | 기존 URDF/드라이버 |
| `/odometry/local` | Local EKF — `publish_tf=false` |

### GNSS heading 보정

- 안테나 배치: 왼쪽=base, 오른쪽=rover
- 수신 heading: base→rover baseline 방향
- `heading_mount_offset_deg` 기본값: **+90.0°**
- 미세 보정 위치: [system.yaml](bunker_gps_nav_bringup/config/system.yaml)

```text
yaw_ros = π/2 − heading_baseline + heading_mount_offset
```

| 차량 진행 방향 | Baseline 방향 | 보정 후 ROS yaw |
|---|---|---|
| 북쪽 | 동쪽, 90° | +π/2 |
| 동쪽 | 남쪽, 180° | 0 |

- RTK 판정: `NavRELPOSNED9` 비트 상수로 FIXED/FLOAT/INVALID 구분
- Heading timestamp: 메시지에 Header가 없어 수신 시각 사용
- 반복되는 동일 `i_tow`: 처리 제외

## 장애물 및 경로 계획

### PointCloud 처리

- 입력: `/patchworkpp/nonground`
- 기준 frame: `base_link`
- 다른 frame 입력 시: TF 변환 후 필터 적용
- 차체 제외 범위: 박스 내부와 경계면
- 설정 파일: [obstacle_filter.yaml](bunker_gps_nav_perception/config/obstacle_filter.yaml)

```text
remove non-finite → self-body exclusion → height → range → voxel downsample
```

| 항목 | 초기값 |
|---|---|
| 차체 X 범위 | −0.65 ~ +0.65 m |
| 차체 Y 범위 | −0.45 ~ +0.45 m |
| 차체 Z 범위 | −0.30 ~ +0.60 m |
| 장애물 높이 | 0.05 ~ 1.5 m |
| 수평거리 | 0.3 ~ 50 m |
| Voxel 크기 | 0.10 m |

- 차체 박스와 1.30 × 0.90 m footprint는 실측에 맞춰 조정
- 필터 진단: 점 수, 처리 시간, 주파수, 단계별 제거 개수

### Nav2 구성

| 항목 | 설정 |
|---|---|
| Global costmap | Rolling, 60×60 m, 해상도 0.15 m |
| Local costmap | Rolling, 20×20 m, 해상도 0.05 m |
| Planner | SmacPlanner2D |
| Controller | Regulated Pure Pursuit (RPP) |
| 재계획 주기 | 1 Hz |
| 정적 지도·AMCL·SLAM | 사용하지 않음 |

- Nonground만으로는 빈 광선 정보가 부족해 이동 장애물 잔상이 남을 수 있음
- 잔상 발생 시: 현장 관측에 따라 clearing 입력 보완
- 미관측 공간은 free로 취급 — 센서가 관측하는 짧은 거리 시험을 대상으로 함

## 속도 및 상태 감시

설정 파일: [safety.yaml](bunker_gps_nav_safety/config/safety.yaml)

```text
Nav2 → /navigation/cmd_vel_raw → supervisor → /cmd_vel → Bunker
```

| 상태 / 조건 | 동작 |
|---|---|
| FIXED | 최대 0.6 m/s |
| FLOAT | 최대 0.3 m/s로 약 3초 유지 후 정지·목표 취소 |
| INVALID | 정지·목표 취소 |
| Heading 단절 4초 | 이동 차단 |
| GNSS/odom/IMU/cloud/localization timeout | 이동 차단 |
| Nav2 비활성화 | 이동 차단 |
| 속도 명령 미갱신 0.5초 | 속도 0 출력 |

- 기존 teleop/mux는 supervisor를 거치도록 연결
- 다른 publisher가 직접 `/cmd_vel`을 발행하면 supervisor를 우회함
- 드라이버의 명령 timeout과 하드웨어 비상정지는 별도로 유지

## 패키지 구성 및 테스트

| 패키지 | 역할 |
|---|---|
| bunker_gps_nav_localization | GNSS 처리, datum, 두 EKF |
| bunker_gps_nav_perception | 차체·장애물 필터 |
| bunker_gps_nav_goal | GPS/RViz 목표와 CLI |
| bunker_gps_nav_safety | 상태 감시, 최종 속도 제한 |
| bunker_gps_nav_nav2 | Nav2 YAML, BT, RViz |
| bunker_gps_nav_bringup | 통합/단계별 launch |

### 단계별 실행

- 위치 추정: `localization.launch.py`
- 장애물 처리: `perception.launch.py`
- Navigation: `navigation.launch.py`
- 공통 조건: 실제 센서와 기존 TF 사용

### 단위 테스트

```bash
cd ~/bunker_gps_navigation_ws
source /opt/ros/humble/setup.bash
colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
colcon test --base-paths src --event-handlers console_direct+
colcon test-result --verbose
```
