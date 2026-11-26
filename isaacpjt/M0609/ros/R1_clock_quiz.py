"""[R1 연습] 두 개의 시계: /clock(시뮬레이션 시간) 옆에 /system_clock(실제 시간) 발행

실행: isaac_python R1_clock_quiz.py
      창에서 Play(▶) 를 누르면 /clock 과 /system_clock 이 함께 발행된다
확인: 새 터미널 두 개에서
      ros2 topic echo /clock --field clock.sec
      ros2 topic echo /system_clock --field clock.sec
      Pause 를 잠깐 했다가 Play 하면 두 시계의 차이가 Pause 한 시간만큼 커진다
"""

import time
from pathlib import Path

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
import omni.graph.core as og
from isaacsim.core.api import World
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

SCENE_USD = Path(__file__).resolve().parents[1] / "scenes" / "pick_place_cube.usd"  # GUI 강의에서 만든 기본 씬
GRAPH_PATH = "/World/Graph/ROS_Clock"  # GUI 강의와 같은 위치

# (가) ROS2 Bridge 확장 켜기: ROS2 노드(ROS2Context, ROS2PublishClock 등)를 쓰기 전에 켠다
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# (나) 작업 씬 열기: 로봇, 바닥, 큐브가 들어 있는 기본 씬
open_stage(str(SCENE_USD))
world = World(stage_units_in_meters=1.0)

# (다) Clock Graph 만들기: R1 의 노드 4개 + 실제 시간용 노드 2개
#      두 Publish 노드 모두 OnPlaybackTick 에 연결 → Play 중에만 발행
og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},  # execution = Action Graph
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),              # Play 중 매 프레임 실행 신호
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),                     # ROS2 통신 설정 (Domain ID)
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),      # 시뮬레이션 시간 읽기
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),           # 시뮬레이션 시간 → /clock
            ("ReadSystemTime", "isaacsim.core.nodes.IsaacReadSystemTime"),       # [추가] 실제 시간 읽기 (Unix epoch 초)
            ("PublishSystemClock", "isaacsim.ros2.bridge.ROS2PublishClock"),     # [추가] 실제 시간 → /system_clock
        ],
        og.Controller.Keys.CONNECT: [
            # /clock
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),       # 실행 신호
            ("Context.outputs:context", "PublishClock.inputs:context"),          # 통신 설정
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),  # 시뮬레이션 시간
            # /system_clock
            ("OnPlaybackTick.outputs:tick", "PublishSystemClock.inputs:execIn"),
            ("Context.outputs:context", "PublishSystemClock.inputs:context"),
            ("ReadSystemTime.outputs:systemTime", "PublishSystemClock.inputs:timeStamp"),  # 실제 시간
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublishClock.inputs:topicName", "clock"),                          # 토픽 이름: /clock
            ("PublishSystemClock.inputs:topicName", "system_clock"),             # 토픽 이름: /system_clock
        ],
    },
)
print("Clock Graph 생성:", GRAPH_PATH)

# (라) 멈춘 상태로 기다린다
world.reset()
world.pause()  # 화면 왼쪽의 Play(▶) 를 누르면 두 시계 발행 시작
print("Play 를 누르면 /clock 과 /system_clock 을 발행한다 (Pause 하면 둘 다 멈춘다)")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0  # Play 중에 지난 프레임 수
while simulation_app.is_running():
    world.step(render=True)
    if not world.is_playing():
        continue
    frame += 1
    if frame % 60 == 0:  # 60 프레임마다 두 시계와 차이 출력 (Pause → Play 후 차이가 커지는지 확인)
        sim_t = world.current_time
        sys_t = time.time()
        print(f"{frame} 프레임 | /clock {sim_t:8.2f} s | /system_clock {sys_t:.2f} s | 차이 {sys_t - sim_t:.2f} s")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
