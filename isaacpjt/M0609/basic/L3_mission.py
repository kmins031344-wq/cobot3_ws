"""[L3 연습 문제] L3_scene_objects.py 를 바탕으로 구현

  1. 원뿔 두 개 추가 : red  = DynamicCone, (0.4, 0.0, 0.5), x 축으로 60 deg 기울인 자세, 반지름 0.05 m, 높이 0.15 m, 빨강
                      yellow = VisualCone, (0.4, 0.0, 0.8), 같은 크기, 노랑
                      → red 는 떨어져 넘어지고 yellow 는 공중에 그대로
  2. 위치, 자세 복사 : 매 프레임 yellow 가 red 보다 0.3 m 위에서 같은 자세를 따라 한다
                      → 실행 중 Shift + 드래그로 red 를 끌어 던져 보고 yellow 가 따라 하는지 확인
  3. 각도 출력       : 60 프레임마다 red 의 위치와 각도(deg) 출력

실행: isaac_python L3_mission.py   (창을 닫으면 종료)
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
from isaacsim.core.api.objects import DynamicCone, VisualCone
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

# [연습 1] 원뿔 두 개 추가 (원뿔은 radius, height 로 크기를 정한다. 처음엔 뾰족한 끝이 +z)
red = world.scene.add(
    DynamicCone(
        prim_path="/World/Red", name="red", position=np.array([0.4, 0.0, 0.5]),
        orientation=euler_angles_to_quat(np.array([60.0, 0.0, 0.0]), degrees=True),
        radius=0.05, height=0.15, color=np.array([1.0, 0, 0]),
    )
)
yellow = world.scene.add(
    VisualCone(
        prim_path="/World/Yellow", name="yellow", position=np.array([0.4, 0.0, 0.8]),
        radius=0.05, height=0.15, color=np.array([1.0, 1.0, 0]),
    )
)
world.reset()

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1

    # [연습 2] 위치, 자세 복사: red 의 (위치, 쿼터니언) 을 읽어 yellow 에 0.3 m 위로 넣는다
    red_position, red_orientation = red.get_world_pose()
    yellow.set_world_pose(position=red_position + np.array([0.0, 0.0, 0.3]), orientation=red_orientation)

    # [연습 3] 각도 출력
    # x, z 각도는 -180 ~ 180, y 각도는 -90 ~ 90 으로 나온다.
    # 그래서 y 가 90 을 넘는 자세는 모양이 같아도 [-180, 80, -180] 처럼 다른 숫자 조합으로 바뀐다.
    if frame % 60 == 0:
        red_angles = quat_to_euler_angles(red_orientation, degrees=True)
        print(f"{frame} 프레임 | red 위치 {np.round(red_position, 3)} | 각도(deg) {np.round(red_angles, 1)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
