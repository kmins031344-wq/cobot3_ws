# L0. Isaac Sim Standalone 기초
# 실행: cd ~/cobot3_ws/isaacpjt/M0609/basic && isaac_python L0_standalone_basic.py
# 결과: Isaac Sim 창이 열리고 파란 큐브가 바닥으로 떨어진다

# STEP 1. SimulationApp 초기화 (반드시 모든 import 보다 먼저)
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# STEP 2. import 모듈 (SimulationApp 이후에만 가능)
import numpy as np
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid

# STEP 3. World 생성 (씬 단위: 미터)
world = World(stage_units_in_meters=1.0)

# Stage 접근 (USD 씬 데이터 직접 접근하는 진입점)
stage = omni.usd.get_context().get_stage()

# STEP 4. 씬 구성: Prim 생성
cube_prim = DynamicCuboid(
    prim_path="/World/BlueCube",
    name="blue_cube",
    position=np.array([0.0, 0.0, 0.5]),
    scale=np.array([0.15, 0.15, 0.15]),
    color=np.array([0.0, 0.0, 1.0]),
)

# Scene 구성
world.scene.add_default_ground_plane()
world.scene.add(cube_prim)

# STEP 5. 물리 초기화 + 씬 확정
world.reset()

# STEP 6. 메인 루프
while simulation_app.is_running():
    world.step(render=True)

# STEP 7. 종료
simulation_app.close()
