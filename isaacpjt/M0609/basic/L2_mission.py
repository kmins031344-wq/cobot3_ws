"""[L2 연습 문제] L2_world_time.py 를 바탕으로 구현

  1. 2초 뒤 큐브 높이   : 2초가 몇 프레임인지 RENDERING_DT 로 계산 → 그 프레임에 번호, 시간, 높이 출력
  2. 루프 도중 reset    : 180 프레임에 높이 출력 → world.reset() → 다시 높이 출력
  3. 물체 하나만 옮기기 : 300 프레임에 큐브만 (0.3, 0.0, 1.0) 으로 → 360 프레임에 위치 출력

실행: isaac_python L2_mission.py   (창을 닫으면 종료)
"""

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

RENDERING_DT = 1.0 / 60.0  # 한 프레임의 시간 (초)

# 장면 구성: World 생성 -> 물체 추가 -> reset
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=RENDERING_DT)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
cube = world.scene.add(
    DynamicCuboid(prim_path="/World/Cube", name="cube", position=np.array([0.0, 0.0, 1.0]), size=0.1)
)
world.reset()  # 물체를 다 넣은 뒤 한 번: 물리를 시작할 준비

# [연습 1] 프레임 수 = 시간 / rendering_dt
TWO_SEC_FRAME = round(2.0 / RENDERING_DT)
print(f"2초는 {TWO_SEC_FRAME} 프레임")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)  # 한 프레임(1/60 초) 진행
    frame += 1

    # [연습 1] 2초 뒤 큐브 높이 (예제와 같은 방법으로 읽기)
    if frame == TWO_SEC_FRAME:
        height = cube.get_world_pose()[0][2]
        print(f"{frame} 프레임 ({frame * RENDERING_DT:.1f} 초): 큐브 높이 {height:.3f} m")

    # [연습 2] 루프 도중 reset
    # reset() 은 장면의 모든 물체를 처음 상태(위치, 속도)로 되돌린다 → 큐브가 1.0 m 로 돌아가 다시 떨어진다
    if frame == 180:
        print(f"reset 전 큐브 높이 {cube.get_world_pose()[0][2]:.3f} m")
        world.reset()
        print(f"reset 후 큐브 높이 {cube.get_world_pose()[0][2]:.3f} m")

    # [연습 3] 물체 하나만 옮기기
    # set_world_pose() 는 이 큐브 하나의 위치만 바꾼다 (reset 과 달리 장면 전체를 되돌리지 않음)
    if frame == 300:
        cube.set_world_pose(position=np.array([0.3, 0.0, 1.0]))
        print("큐브를 (0.3, 0, 1.0) 으로 옮김")
    if frame == 360:
        print(f"옮긴 뒤 60 프레임: 큐브 위치 {np.round(cube.get_world_pose()[0], 3)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
