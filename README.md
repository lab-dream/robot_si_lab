# robot_si_lab

Robot SI를 위한 모방학습 입문 실습 저장소

- 오전 실습: turtlesim으로 ROS 2 Topic, Service 명령과 정해진 작업 수행 확인
- 오후 실습: MuJoCo OMY 협동로봇 시뮬레이터에서 Pick and Place 시연 수집, LeRobot 데이터셋 변환
- 실물 로봇 없이 노트북 한 대(Ubuntu 24.04 또는 WSL2)로 전 과정 진행

```
 teleop / scripted_demo ──▶ omy_sim (MuJoCo) ──▶ ros2 bag record ──▶ bag_to_lerobot.py ──▶ LeRobot 데이터셋
        /omy/tcp_delta        /joint_states            mcap               (venv_lerobot)        학습 입력
        /omy/pick_place       /omy/action
                              /front_cam, /wrist_cam
```

## 1. 사전 준비

- 「실습 환경 사전 설치 가이드」 1~3단계 완료 상태 전제
  - ROS 2 Jazzy (`ros-jazzy-desktop`, `ros-dev-tools`), ffmpeg
  - `~/robot_si/venv_ros`: `--system-site-packages`, mujoco, pytest, pillow
  - `~/robot_si/venv_lerobot`: torch (CPU), `lerobot[dataset]`, rosbags
- `lerobot`만 설치 시 `datasets` 모듈 누락으로 import 실패, 반드시 `lerobot[dataset]` 설치

```bash
source ~/robot_si/venv_lerobot/bin/activate
pip install 'lerobot[dataset]' rosbags
deactivate
```

## 2. 저장소 설치와 빌드

- 저장소 위치: `~/robot_si_lab`
- 빌드는 반드시 venv_ros 활성화 후 `python -m colcon build`로 실행
  - 노드 실행 파일이 venv_ros의 python(mujoco 포함)을 사용하도록 하기 위함
  - 시스템 `colcon build` 사용 시 `ModuleNotFoundError: No module named 'robot_si'` 발생

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_si/venv_ros/bin/activate
cd ~/robot_si_lab
pip install -e .
python -m pytest -q            # 시뮬레이터 단위 테스트 10개
cd ros2_ws
python -m colcon build --symlink-install
source install/setup.bash
```

- 새 터미널마다 아래 3줄 실행 필요 (이하 "환경 설정")

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_si/venv_ros/bin/activate
source ~/robot_si_lab/ros2_ws/install/setup.bash
```

## 3. 오전 실습: turtlesim으로 Topic, Service 익히기

### 3-1. turtlesim 기본 명령

- 터미널 1: turtlesim 실행

```bash
ros2 run turtlesim turtlesim_node
```

- 터미널 2: 노드, 토픽, 서비스 목록 확인과 직접 명령

```bash
ros2 node list
ros2 topic list
ros2 topic echo --once /turtle1/pose
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0}, angular: {z: 1.8}}"
ros2 service list
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 2.0, y: 2.0, theta: 0.0}"
ros2 service call /clear std_srvs/srv/Empty
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{theta: 1.57}" --feedback
ros2 param list /turtlesim
ros2 param set /turtlesim background_r 150
rqt_graph
```

### 3-2. Publisher, Subscriber, Launch 작성

- 빈칸(`____`) 채우기 실습, 정답은 각 파일 상단 `Answer:` 경로
- turtlesim 실행 상태에서 진행 (터미널 1: `ros2 run turtlesim turtlesim_node`)

| 실습 | 파일 | 실행 |
|---|---|---|
| 1-1 Publisher | `exercises/turtle_circle_blank.py` | `python3 exercises/turtle_circle_blank.py` |
| 1-2 Subscriber | `exercises/pose_listener_blank.py` | `python3 exercises/pose_listener_blank.py` |
| 2 Launch | `exercises/my_turtle_blank.launch.py` | `ros2 launch exercises/my_turtle_blank.launch.py` |

- 정답 노드와 launch 파일 실행

```bash
ros2 launch robot_si_ros turtle_circle.launch.py speed:=3.0 turn:=1.5
ros2 param get /turtle_circle speed
```

- 실수 정수 혼동 주의: `speed:=3` 입력 시 파라미터 타입 오류, `3.0`으로 입력

### 3-3. 명령 한 번으로 정해진 작업 수행

- `turtle_task_server` 노드 제공 기능
  - Service `/turtle_task/draw_square` (std_srvs/Trigger): (3,3)에서 시작하는 한 변 5의 정사각형 주행 후 응답
  - Topic `/turtle_task/goal` (geometry_msgs/Point): 지정 좌표로 이동
  - 내부 동작: `/turtle1/pose` 구독, 20 Hz P 제어로 `/turtle1/cmd_vel` 발행
- 터미널 1: turtlesim과 작업 서버 동시 실행

```bash
ros2 launch robot_si_ros turtle_task.launch.py
```

- 터미널 2: 서비스 호출과 토픽 발행

```bash
ros2 service call /turtle_task/draw_square std_srvs/srv/Trigger
ros2 topic pub --once /turtle_task/goal geometry_msgs/msg/Point "{x: 8.0, y: 5.0}"
```

- 관찰 포인트
  - Service: 작업 완료까지 응답 대기, `success=True` 응답으로 결과 확인
  - Topic: 발행 즉시 반환, 결과는 `/turtle1/pose` 구독으로 확인
- 코드: `ros2_ws/src/robot_si_ros/robot_si_ros/turtle_task_server.py`

### 3-4. rosbag2 기록과 재생

- 터미널 1: `ros2 launch robot_si_ros turtle_task.launch.py`
- 터미널 2: 기록 시작 (종료는 Ctrl+C)

```bash
cd ~/robot_si_lab
ros2 bag record -o bags/turtle_square /turtle1/cmd_vel /turtle1/pose
```

- 터미널 3: 기록 중 작업 수행, 끝나면 터미널 2에서 Ctrl+C

```bash
ros2 service call /turtle_task/draw_square std_srvs/srv/Trigger
```

- 기록 확인, 작업 서버 종료(터미널 1 Ctrl+C) 후 turtlesim만 다시 실행, 명령만 재생

```bash
ros2 bag info bags/turtle_square
ros2 run turtlesim turtlesim_node                 # 터미널 1
ros2 bag play bags/turtle_square --topics /turtle1/cmd_vel
```

- 관찰 포인트: 같은 명령을 재생해도 끝점이 (3, 3)에서 약 2 이상 벗어남
  - 기록된 명령만 그대로 재생하는 개루프 방식, 현재 위치를 보고 보정하지 않음
  - 오후 모방학습에서 관측을 보고 행동을 내는 정책이 필요한 이유

## 4. 오후 실습: OMY 시뮬레이터에서 시연 수집

### 4-1. 시뮬레이터 실행

- 장면 구성: ROBOTIS OMY 6축 협동로봇, 빨간 큐브, 파란 목표 패드, 정면 카메라와 손목 카메라
- 터미널 1

```bash
ros2 launch robot_si_ros omy_sim.launch.py            # MuJoCo 뷰어 창 포함
ros2 launch robot_si_ros omy_sim.launch.py viewer:=false   # 창 없이 실행
```

- 인터페이스 (20 Hz, 모든 메시지 동일 타임스탬프)

| 종류 | 이름 | 타입 | 내용 |
|---|---|---|---|
| 발행 | `/joint_states` | sensor_msgs/JointState | Joint1~6, gripper 현재값 (관측) |
| 발행 | `/omy/action` | sensor_msgs/JointState | 해당 스텝 적용 명령 (행동) |
| 발행 | `/front_cam/image_raw`, `/wrist_cam/image_raw` | sensor_msgs/Image | 128x128 rgb8 |
| 발행 | `/omy/episode` | std_msgs/Int32 | 에피소드 번호, `/omy/reset` 호출마다 1 증가 |
| 발행 | `/tf` | tf2_msgs/TFMessage | world, base_unit, link1~6, tcp, 카메라 2개, cube, target 좌표계 |
| 구독 | `/joint_command` | std_msgs/Float64MultiArray | 관절 목표값 7개 (j1~j6, gripper) |
| 구독 | `/omy/tcp_delta` | std_msgs/Float64MultiArray | 끝단 이동량 [dx, dy, dz] (m), gripper |
| 서비스 | `/omy/reset` | std_srvs/Trigger | 큐브 위치 무작위 배치, 새 에피소드 시작 |
| 서비스 | `/omy/go_home` | std_srvs/Trigger | 초기 자세 복귀 |
| 서비스 | `/omy/pick_place` | std_srvs/Trigger | 큐브 집어 목표 패드에 놓기, 성공 여부 응답 |

### 4-2. 명령으로 로봇 작업 수행

- 터미널 2

```bash
ros2 service call /omy/reset std_srvs/srv/Trigger
ros2 service call /omy/pick_place std_srvs/srv/Trigger
ros2 service call /omy/go_home std_srvs/srv/Trigger
ros2 topic pub --once /joint_command std_msgs/msg/Float64MultiArray "{data: [0.0, -0.3, 1.2, 0.6, 1.57, 0.0, 0.0]}"
ros2 topic echo --once /joint_states
```

- 오전 turtlesim 실습과 동일 구조: Service는 작업 단위 명령, Topic은 연속 목표값 명령

### 4-3. TF 좌표계 확인

```bash
ros2 run tf2_ros tf2_echo world tcp           # 끝단 위치, 홈 자세에서 약 [0.30, 0.00, 0.25]
ros2 run tf2_ros tf2_echo tcp cube            # 끝단 기준 큐브 위치
ros2 run tf2_tools view_frames                # 좌표계 트리 frames_*.pdf 생성
rviz2                                         # Fixed Frame: world, Add > TF
```

- 좌표계 트리: world → base_unit → link1 → … → link6 → tcp, link6 → wrist_cam, world → front_cam, cube, target
- 카메라 좌표계: ROS 광학 좌표계 규약 (z 앞, x 오른쪽, y 아래), 이미지 메시지 `frame_id`와 동일 이름

### 4-4. 키보드 원격조작

- 터미널 2 (터미널 창에 포커스 유지)

```bash
ros2 run robot_si_ros omy_teleop_key
```

| 키 | 동작 |
|---|---|
| w / s | 끝단 +x / -x 1 cm |
| a / d | 끝단 +y / -y 1 cm |
| r / f | 끝단 위 / 아래 1 cm |
| space | 그리퍼 열기 / 닫기 |
| n | 새 에피소드 (`/omy/reset`) |
| h | 초기 자세 (`/omy/go_home`) |
| q | 종료 |

- 시연 순서 예시: n, 큐브 위로 이동, f로 하강, space로 파지, r로 상승, 패드 위로 이동, f로 하강, space로 놓기

### 4-5. 시연 녹화

- 터미널 3: 녹화 시작 (종료는 Ctrl+C)

```bash
cd ~/robot_si_lab
scripts/record_demos.sh bags/my_demos
```

- 녹화 중 에피소드마다 `n`(또는 `/omy/reset`)으로 시작 후 시연
- 녹화 시작 시점에 진행 중이던 에피소드는 변환 시 제외
- 사람 대신 스크립트 시연 (파이프라인 점검용)

```bash
ros2 run robot_si_ros scripted_demo --ros-args -p episodes:=3
```

- 용량 참고: 3 에피소드 약 37초, bag 약 70 MB, 변환 후 데이터셋 약 10 MB

### 4-6. LeRobot 데이터셋 변환과 확인

- ROS 불필요, venv_lerobot에서 실행 (새 터미널)

```bash
source ~/robot_si/venv_lerobot/bin/activate
cd ~/robot_si_lab
python tools/bag_to_lerobot.py bags/my_demos datasets/omy_pick_place
python tools/inspect_dataset.py datasets/omy_pick_place
```

- 데이터셋 구성 (fps 20)
  - `observation.state`: float32 x7, Joint1~6과 gripper
  - `action`: float32 x7, 같은 순서의 명령값
  - `observation.images.front`, `observation.images.wrist`: 128x128 이미지 (PNG 저장, 동영상 코덱 불필요)
  - `task`: "Pick up the red cube and place it on the blue pad."
- `inspect_dataset.py` 출력: 에피소드별 길이, 행동과 상태 차이, 행동 변화량, 그리퍼 동작 횟수, `previews/episode_XXX.gif` 미리보기
- 실행 시 `torchcodec ... cannot decode videos` 경고: 이미지 저장 방식이라 무시 가능

### 4-7. 학습된 정책 실행 (추론)

- venv_lerobot에서 실행, 시뮬레이터용 mujoco와 robot_si 추가 설치 필요 (최초 1회)
  - mujoco 버전은 venv_ros와 동일하게 지정

```bash
source ~/robot_si/venv_lerobot/bin/activate
pip install "mujoco==$(~/robot_si/venv_ros/bin/python -c 'import mujoco; print(mujoco.__version__)')"
pip install -e ~/robot_si_lab pytest
```

- 정책 폴더: `lerobot-train` 결과의 `.../checkpoints/<step>/pretrained_model` (config.json, model.safetensors, 전처리 설정 포함)
- ROS 없이 시뮬레이터에서 직접 실행, 성공률 출력

```bash
cd ~/robot_si_lab
python tools/run_policy.py outputs/act_omy/checkpoints/last/pretrained_model --episodes 10
python tools/run_policy.py outputs/act_omy/checkpoints/last/pretrained_model --episodes 3 --viewer
python tools/run_policy.py outputs/act_omy/checkpoints/last/pretrained_model --episodes 1 --gif outputs/policy.gif
```

- 한 스텝: 관절 7개와 카메라 2장 관측, 정책이 관절 목표 7개 출력, 50 ms 적용 (omy_sim과 같은 주기)
- 성공 판정: 큐브가 파란 패드 위에 놓이고 그리퍼가 열린 상태
- `--seed`: 큐브 위치 시작 시드 (기본 1000), 같은 시드로 정책 간 비교 가능
- GPU 없으면 자동으로 CPU 사용 (`--device cpu`로 지정 가능)

### 4-8. ROS 2로 정책 실행

- 키보드 원격조작 자리에 정책 노드 연결: 관측 토픽 구독, `/joint_command` 발행
- 터미널 1: 시뮬레이터 (환경 설정 후)

```bash
ros2 launch robot_si_ros omy_sim.launch.py
```

- 터미널 2: 정책 노드, ROS 2와 venv_lerobot을 함께 사용 (venv_ros 아님)

```bash
source /opt/ros/jazzy/setup.bash
source ~/robot_si/venv_lerobot/bin/activate
cd ~/robot_si_lab
python tools/ros_policy.py outputs/act_omy/checkpoints/last/pretrained_model
```

- 터미널 3: 새 에피소드마다 큐브 위치 변경, 정책이 다시 집어서 이동

```bash
ros2 service call /omy/reset std_srvs/srv/Trigger
```

- 구독: `/joint_states`, `/front_cam/image_raw`, `/wrist_cam/image_raw` (같은 타임스탬프 3개가 모이면 1회 추론), `/omy/episode`
- 발행: `/joint_command` 약 20 Hz
- `/omy/reset` 시 정책의 행동 묶음(action chunk) 초기화
- 정책 노드 종료(Ctrl+C) 후 로봇은 마지막 명령 자세 유지

## 5. 자동 점검

- 강의 전 환경 점검용, 저장소 최상위에서 환경 설정 후 실행

```bash
python -m pytest -q
bash tests/ros/test_turtle_pubsub.sh    # 약 40초, 빈칸 실습을 정답으로 채워 실행
bash tests/ros/test_turtle_task.sh      # 약 30초
bash tests/ros/test_turtle_bag.sh       # 약 60초, 재생 끝점 오차 출력
bash tests/ros/test_omy_sim.sh          # 약 40초
bash tests/ros/test_record_demos.sh     # 약 60초, 마지막 줄에 bag 경로 출력
```

- 각 스크립트 마지막 줄 `PASS ...` 출력과 종료 코드 0 확인
- 정책 실행 점검 (4-7절 설치 후): ROS를 source하지 않은 새 터미널에서 venv_lerobot으로 실행
  - ROS의 pytest 플러그인(launch_testing)이 venv_lerobot에 없는 lark를 불러와 실패하기 때문

```bash
cd ~/robot_si_lab
~/robot_si/venv_lerobot/bin/python -m pytest -q tests/test_policy.py   # 2 passed, 약 5초
```

- ROS 2 정책 노드 점검 (학습된 정책 필요, 환경 설정 후): 약 30초, 명령 주기와 큐브 이동 확인

```bash
bash tests/ros/test_ros_policy.sh outputs/act_omy/checkpoints/last/pretrained_model
```

## 6. 문제 해결

| 증상 | 원인과 조치 |
|---|---|
| `No module named 'robot_si'` | venv_ros 비활성 상태 빌드, 2절 순서로 `python -m colcon build` 재실행 |
| `ros2 topic hz /joint_states` 20 Hz 미만 | GPU 없이 소프트웨어 렌더링(llvmpipe) 중, WSL에서 `export GALLIUM_DRIVER=d3d12` 후 재실행 |
| 서비스 응답이 두 번 오거나 엉뚱한 결과 | 이전 실행 노드 잔존, `pkill -f omy_sim; pkill -f turtlesim_node` 후 재실행 |
| 뷰어 창 미표시 | `echo $DISPLAY` 확인 (WSLg는 `:0`), 창 없이 `viewer:=false`로 진행 가능 |
| `import lerobot` 실패 (`datasets`) | 1절의 `pip install 'lerobot[dataset]'` 실행 |

## 7. 구성

```
assets/robotis_omy/      OMY 모델 (ROBOTIS robotis_mujoco_menagerie, Apache-2.0) + 실습 장면
exercises/                오전 빈칸 채우기 실습 (Publisher, Subscriber, Launch)
robot_si/                MuJoCo 래퍼(sim.py, TF 좌표계 포함), 스크립트 작업(tasks.py)
ros2_ws/src/robot_si_ros turtle_circle, pose_listener, turtle_task_server, omy_sim, omy_teleop_key,
                         scripted_demo, launch 파일
scripts/record_demos.sh  rosbag2 녹화
tools/                   bag_to_lerobot.py, inspect_dataset.py, policy_runner.py, run_policy.py,
                         ros_policy.py
tests/                   pytest 단위 테스트, tests/ros 통합 점검 스크립트
```

- 참고: [jeongeun980906/lerobot-mujoco-tutorial](https://github.com/jeongeun980906/lerobot-mujoco-tutorial) (MuJoCo 시연 수집과 LeRobot 변환 흐름)
- OMY 모델 출처와 라이선스: `assets/robotis_omy/NOTICE`, `assets/robotis_omy/LICENSE`
