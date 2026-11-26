# rokey_cobot3

Isaac Sim 시뮬레이션 환경에서 두산 협동로봇 M0609와 OnRobot RG2 그리퍼, Intel RealSense D455 카메라를 불러와 관절 제어와 pick and place를 단계별로 익히고, ROS 2와 연동해 카메라 기반 색상 분류까지 구현하는 프로젝트입니다.

## 실행 환경

- ROS 2: Jazzy Jalisco
- 시뮬레이터: NVIDIA Isaac Sim (`isaacsim.*` 네임스페이스 API 사용)
- 로봇 모델: Doosan M0609 + OnRobot RG2 그리퍼 + RealSense D455 (`m0609_rg2_d455`)
- Isaac 예제 실행: Isaac Sim 내장 파이썬 (`isaac_python`)

## 시작하기

### Isaac Sim 예제

`isaacpjt/M0609/basic/`의 예제는 L0부터 차례로 실행합니다.

```bash
cd ~/cobot3_ws/isaacpjt/M0609/basic
isaac_python L0_standalone_basic.py
```

L4부터는 로봇 USD 파일(`isaacpjt/M0609/robot/m0609_rg2_d455.usd`)이 필요합니다. `*.usd`는 `.gitignore`에 들어 있으므로 각자 로컬에서 준비해야 합니다.

### ROS 2 패키지

```bash
cd ~/cobot3_ws
colcon build --packages-select m0609
source install/setup.bash

# Topic
ros2 run m0609 topic_pub
ros2 run m0609 topic_sub

# Service
ros2 run m0609 service_server
ros2 run m0609 service_client 3 5
```

### 색상 분류 pick and place

Isaac Sim(PC A)이 손목 카메라 영상을 `/rgb`로 보내면, 색상 감지 노드(PC B)가 큐브 색을 `/color_id`(1 = 파랑, 2 = 초록)로 돌려줍니다. 로봇은 이 값을 받아 큐브를 같은 색 마커 위에 놓습니다.

```bash
# PC A: Isaac Sim
cd ~/cobot3_ws/isaacpjt/M0609/color_sorting
isaac_ros                          # Isaac Sim 내부 ROS 2 브릿지 환경 설정
isaac_python pick_place_color.py

# PC B: 색상 감지 노드
ros2 run m0609 m0609_color_detector
```

## 진행 현황

| 단계 | 파일 | 내용 |
|---|---|---|
| L0 | `L0_standalone_basic.py` | Standalone 기초: SimulationApp, World, 큐브 떨어뜨리기 |
| L1 | `L1_app.py`, `L1_mission.py`, `L1_mission_wrong.py` | 앱 골격과 import 순서, 프레임 세기 |
| L2 | `L2_world_time.py`, `L2_mission.py` | World와 프레임 시간, `world.reset()` |
| L3 | `L3_scene_objects.py`, `L3_mission.py`, `L3_scene_objects_answer.py` | 장면 객체 추가, 조회, 이동과 자세(쿼터니언/오일러) |
| L4 | `L4_robot_joints.py` | M0609 로봇 불러오기와 관절 제어 |
| L5 | `L5_main_loop.py`, `L5_main_loop_quiz.py` | 메인 루프와 Play / Stop 처리 |
| L6 | `L6_task_controller.py`, `L6_task_controller_quiz.py` | Task와 Controller로 코드 나누기, 큐브를 바라보는 로봇 |
| L7 | `L7_gripper_steps.py`, `L7_gripper_steps_quiz.py` | RG2 그리퍼와 pick and place: 큐브를 집어 -45도 방향으로 옮겨 놓기 |
| 색상 분류 | `color_sorting/pick_place_color.py`, `m0609_color_detector.py` | 손목 카메라 + HSV 색 감지로 파랑/초록 큐브를 같은 색 마커 위에 분류 |
| ROS 2 | `src/m0609/` | Topic 발행/구독, Service 서버/클라이언트, 색상 감지 노드 |

## 저장소 구조

```text
cobot3_ws/
├── isaacpjt/
│   └── M0609/
│       ├── basic/          # Isaac Sim 단계별 예제와 연습 문제 (L0 ~ L7)
│       ├── color_sorting/  # 색상 분류 pick and place (Isaac Sim 쪽)
│       ├── descriptor/     # 로봇 description, RRT 설정 (yaml)
│       ├── props/          # 블록 소품 에셋 (텍스처)
│       └── robot/          # M0609, RG2, D455 모델 (URDF, xacro, mesh)
└── src/
    └── m0609/              # ROS 2 패키지 (topic_pub, topic_sub, service_server, service_client, m0609_color_detector)
```

## 브랜치 전략

작업은 이슈를 먼저 만들고 `<type>/<이슈번호>-<설명>` 브랜치(예: `feature/123-login-api`)에서 진행한 뒤 `main`으로 PR을 보냅니다. `main`에는 직접 push하지 않고, PR은 Squash and merge로 머지한 뒤 브랜치를 삭제합니다.

커밋 메시지는 `type(scope): 요약` 형식을 따르고, type은 `feat`, `fix`, `refactor`, `test`, `docs`, `chore` 중에서 고릅니다.

## 팀 구성

| 역할 | 담당 |
|---|---|
| (역할) | (이름) |
| (역할) | (이름) |
| (역할) | (이름) |
