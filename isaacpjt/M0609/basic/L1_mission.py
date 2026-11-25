"""[L1 연습 문제] L1_app.py 를 바탕으로 구현

  1. import 순서 고치기   : World 를 SimulationApp 보다 위에 두면 에러 → 아래로 옮겨 해결
  2. 프레임 세기          : 60 프레임마다 "60 프레임", "120 프레임" ... 출력
  3. 정해진 프레임 후 종료: 300 프레임이 되면 메시지 출력 후 루프 탈출

실행: isaac_python L1_mission.py   (300 프레임 뒤 스스로 종료)
"""

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
# [연습 1] 아래 줄을 SimulationApp 보다 위에 두고 실행하면
#   ModuleNotFoundError: No module named 'isaacsim.core'
# 가 난다. Isaac 모듈은 앱이 켜질 때 불러오는 확장 기능 안에 있기 때문이다.
#   from isaacsim.core.api import World   ← 여기 있으면 에러
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
from isaacsim.core.api import World  # ← [연습 1] 앱을 켠 뒤로 옮겨서 해결

world = World()
print("World 생성 성공")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
MAX_FRAME = 300
frame = 0
while simulation_app.is_running():
    simulation_app.update()
    frame += 1                          # [연습 2] 루프 한 번 = 1 프레임

    if frame % 60 == 0:                 # [연습 2] 나머지 연산자 % 로 60 프레임마다
        print(f"{frame} 프레임")

    if frame >= MAX_FRAME:              # [연습 3] 300 프레임이 되면
        print(f"{MAX_FRAME} 프레임이 지나 종료합니다")
        break                           #          루프를 빠져나와 아래 close() 로

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
