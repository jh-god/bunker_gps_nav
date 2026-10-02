# 실차 배포 구성 검증

검증일: 2026-10-02. 빌드 환경: Ubuntu 22.04 x86_64 / ROS 2 Humble.
워크스페이스: `~/bunker_gps_navigation_ws`.

## 변경 범위

- 합성 센서 publisher, 데스크탑 데모 launch, 장애 주입 토픽 및 통합 시험 실행기 제거.
- Bringup의 시험 전용 의존성과 설치 항목 제거. 기존 설치에 남은 실행 파일도 정리.
- 실제 시간과 실제 드라이버 입력으로 통합/단계별 launch 구성.
- Auto datum에서 첫 실제 RTK FIX를 대기. Manual datum은 명시적인 위경도가 없으면 거부.
- Release / BUILD_TESTING=OFF를 배포 기본값으로 사용. GNSS와 PointCloud 계산 단위 테스트는 선택적으로 유지.

## 검증 범위

| 확인 항목 | 결과 |
|---|---|
| 전체 새 빌드 | 6개 패키지 성공 (Release, 검증용 BUILD_TESTING=ON) |
| 단위 테스트 | GNSS 4개 + PointCloud 4개 통과; colcon wrapper 포함 10 tests, 0 failures |
| ROS 의존성 | rosdep check 통과; 이 데스크탑에서는 기존 로컬 GNSS 의존성을 명시적으로 source |
| 실차 launch 4개 | 공개 인자는 system_config, nav2_config, rviz만 제공 |
| 소스·설치 잔존 검사 | 센서 생성기, 데모 launch 및 기존 시험 실행 파일 없음 |
| Datum 구성 | auto는 좌표 미지정으로 실행, manual은 위경도 누락 시 거부 |
| 실제 센서가 없는 상태의 기동 | 실차 노드 14개 기동, 실제 입력 토픽에 자체 publisher 없음 |
| 입력 대기 동작 | GNSS 위치/odom TF 자체 생성 없음, motion_allowed=false 및 cmd_vel=0 확인 |
| 프레임·시간·TF 설정 | base_link, 실제 시간, local EKF TF=false / global EKF TF=true 확인 |

기동 검사는 격리된 localhost ROS 도메인에서 12초간 실행했습니다. 센서가 없으므로 Nav2는 map TF를 기다리며 활성화되지는 않습니다. TF 대기 중 검사를 종료할 때 Nav2 lifecycle/context 종료 오류가 출력되어, 이 결과는 기동·입력 대기 상태 검증으로 한정합니다. 주행 성공을 의미하지 않습니다.

이전 센서 생성기를 사용한 주행 결과는 이번 배포 구성의 실차 검증으로 간주하지 않습니다. Jetson aarch64 빌드, 드라이버 연결, 실제 장애물 및 최종 도착 정확도는 현장에서 확인해야 합니다.
