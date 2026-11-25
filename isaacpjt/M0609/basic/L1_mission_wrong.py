"""[L1 연습 1] import 순서가 틀린 버전 (에러 확인용)

World 를 SimulationApp 보다 먼저 import 해서 일부러 에러를 낸다.
실행: isaac_python L1_mission_wrong.py
결과: ModuleNotFoundError: No module named 'isaacsim.core'
"""

# ============================================================
# 틀린 순서: Isaac 모듈을 앱보다 먼저 import
# ============================================================
from isaacsim.core.api import World  

# ============================================================
# 1. 앱 시작
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

world = World()
print("World 생성 성공")

# ============================================================
# 3. 메인 루프
# ============================================================
while simulation_app.is_running():
    simulation_app.update()

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
