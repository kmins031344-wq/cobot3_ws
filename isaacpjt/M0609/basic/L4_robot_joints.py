"""[L4 예제] M0609 로봇 불러오기와 관절 제어 

실행: isaac_python L4_robot_joints.py
"""

from pathlib import Path

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
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction

ROBOT_USD = Path(__file__).resolve().parents[1] / "robot" / "m0609_rg2_d455.usd"  # isaacpjt/M0609/robot

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

# (가) 로봇 USD 참조: 파일의 기본 prim(로봇)을 /World/m0609_rg2_d455 에 넣는다
add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path="/World/m0609_rg2_d455")
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))
world.reset()  # reset 후에 관절 정보를 읽을 수 있다

# (나) 관절 정보
print("관절 개수:", robot.num_dof)
for index, name in enumerate(robot.dof_names):
    print(f"  {index:2d} {name}")
arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]
finger = robot.get_dof_index("finger_joint")
print("arm (팔 관절 6개가 관절 배열의 몇 번째인지):", arm)
print("finger (그리퍼가 관절 배열의 몇 번째인지):", finger)

# (다) 순간이동: joint_1 45 deg, 그리퍼 열기
start = np.zeros(robot.num_dof)
start[arm] = np.radians([45, 0, 90, 0, 90, 0])  # 팔
start[finger] = 0.0                             # 그리퍼 열기 (0.0 rad)
print("순간이동 명령 start (관절 12개, rad):", np.round(start, 3).tolist())
robot.set_joint_positions(start)
q = robot.get_joint_positions()
print(f"순간이동 후 | joint_1 {np.degrees(q[arm[0]]):6.1f} deg | 그리퍼 {abs(q[finger]):.2f} rad")


# ============================================================
# 3. 메인 루프 
# ============================================================
MOVE_FRAME = 100   # 이 프레임부터 왕복 시작 (그 전까지는 순간이동한 자세 그대로)
HALF = 60          # 한쪽으로 가는 데 주는 프레임 수 (도착까지 약 37 프레임 + 여유)
PRINT_EVERY = 10   # 몇 프레임마다 상태를 출력할지

LEFT_DEG = -45     # 갈 때 joint_1 목표
RIGHT_DEG = 45     # 올 때 joint_1 목표
GRIP_CLOSE = 0.9   # 그리퍼 닫기 (rad)
GRIP_OPEN = 0.0    # 그리퍼 열기 (rad)

frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1

    # (라) HALF 프레임마다 방향을 바꿔 모터 목표를 보낸다
    if frame >= MOVE_FRAME and (frame - MOVE_FRAME) % HALF == 0:
        go_left = ((frame - MOVE_FRAME) // HALF) % 2 == 0
        target = start.copy()  # 그리퍼 미믹 관절 등 나머지 관절 값은 start 유지
        target[arm] = np.radians([LEFT_DEG if go_left else RIGHT_DEG, 0, 90, 0, 90, 0])
        target[finger] = GRIP_CLOSE if go_left else GRIP_OPEN  # 갈 때 닫고, 올 때 연다
        print(f"{frame} 프레임 → joint_1 {LEFT_DEG if go_left else RIGHT_DEG} deg 목표,"
              f" 그리퍼 {'닫기' if go_left else '열기'}")
        robot.apply_action(ArticulationAction(joint_positions=target))

    # (마) 일정 간격으로 현재 상태 출력
    if frame >= MOVE_FRAME and frame % PRINT_EVERY == 0:
        q = robot.get_joint_positions()
        print(f"{frame} 프레임 | joint_1 {np.degrees(q[arm[0]]):6.1f} deg | 그리퍼 {abs(q[finger]):.2f} rad")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()