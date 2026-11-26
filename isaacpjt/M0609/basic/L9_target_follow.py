"""[L9 미션] Target Following: 빨간 타겟을 끌고 돌리면 그리퍼가 따라간다

뷰포트에서 빨간 타겟(VisualCuboid)을 끌거나(Move) 돌리면(Rotate),
매 프레임 IK 로 팔 관절 6 개를 풀어 그리퍼(TCP)가 타겟 위치와 자세를 따라간다.
- 닿지 않는 곳이면 마지막 명령을 유지하고 그 자리에서 기다린다.
- 팔 형태(팔꿈치 위, 손목 아래)는 홈 자세와 같게 유지한다.

실행: isaac_python L9_target_follow.py
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
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.rotations import rot_matrix_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

# 파일 경로
M0609_DIR = Path(__file__).resolve().parents[1]                            # isaacpjt/M0609
ROBOT_USD = f"{M0609_DIR}/robot/m0609_rg2_d455.usd"                        # 장면에 넣을 로봇 (M0609 + RG2 + D455)
URDF_PATH = f"{M0609_DIR}/robot/m0609_rg2.urdf"                            # IK 용 URDF
DESCRIPTION_PATH = f"{M0609_DIR}/descriptor/m0609_robot_description.yaml"  # IK 용 로봇 설명 파일

# 로봇 관절
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]
TCP_FRAME = "gripper_center"  # IK 로 위치를 맞출 기준점: 그리퍼 손가락 사이

TARGET_SIZE = 0.04  # 빨간 타겟 한 변 (m)

# IK 계산기: 파일 두 개로 만든다 (장면의 로봇과 상관없이 계산만 한다)
ik_solver = LulaKinematicsSolver(
    robot_description_path=DESCRIPTION_PATH,          # 로봇 설명 파일
    urdf_path=URDF_PATH,                              # URDF 파일
)

# 타겟 시작 위치와 자세 = 홈 자세의 TCP 위치와 자세 (FK 로 구한다. 그리퍼가 아래를 향한다)
_home_pos, _home_rot = ik_solver.compute_forward_kinematics(TCP_FRAME, np.radians(HOME_DEG))
TARGET_START_POS = np.array(_home_pos)               # 약 [0.37, 0.01, 0.24] (m)
TARGET_START_ORIENT = rot_matrix_to_quat(_home_rot)  # 회전 행렬 -> 쿼터니언 [w, x, y, z]


# ------------------------------------------------------------
# (가) Task: 로봇(팔 + 그리퍼)과 빨간 타겟이 놓인 환경
# ------------------------------------------------------------
class FollowTargetTask(BaseTask):
    """장면(바닥 + M0609 + RG2 그리퍼 + 빨간 타겟)을 만들고, 시작 자세를 맞추고, 관측값을 돌려준다"""

    # 언제: 첫 world.reset() 때 한 번
    # 할 일: 장면에 물체 추가
    def set_up_scene(self, scene):
        super().set_up_scene(scene)                   # 부모 것을 먼저 (self.scene 을 저장한다)
        scene.add_default_ground_plane()              # 바닥

        # 로봇 USD 참조
        add_reference_to_stage(
            usd_path=ROBOT_USD,                       # 참조할 USD 파일 경로
            prim_path="/World/m0609_rg2_d455",        # 장면에 만들 prim 경로
        )

        # 그리퍼 객체: finger_joint 하나로 열고 닫는다 (이번 미션에서는 열어 두기만 한다)
        gripper = ParallelGripper(
            end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",  # 그리퍼 본체 prim
            joint_prim_names=["finger_joint"],        # 구동 관절 이름 (RG2 는 하나)
            joint_opened_positions=np.array([0.0]),   # 열림 각도 (rad)
            joint_closed_positions=np.array([1.0]),   # 닫힘 각도 (rad)
            use_mimic_joints=True,                    # 나머지 손가락은 USD 의 mimic 관절이 따라 움직임
        )

        # 로봇 객체: 팔과 그리퍼를 한 객체로
        self._robot = scene.add(
            SingleManipulator(
                prim_path="/World/m0609_rg2_d455",    # 로봇 prim 경로
                name="m0609",                         # 장면에서 찾을 때 쓰는 이름
                end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",  # 그리퍼 본체 prim
                gripper=gripper,                      # 위에서 만든 그리퍼 객체
            )
        )

        # 빨간 타겟: 물리가 없는 상자 (뷰포트에서 끌고 돌린다)
        self._target = scene.add(
            VisualCuboid(
                prim_path="/World/Target",            # Stage 에 만들 prim 경로
                name="target",                        # 장면에서 찾을 때 쓰는 이름
                position=TARGET_START_POS,            # 홈 자세의 TCP 위치
                orientation=TARGET_START_ORIENT,      # 홈 자세의 TCP 자세 (그리퍼가 아래를 향함)
                scale=np.array([TARGET_SIZE] * 3),    # 한 변 4 cm
                size=1.0,                             # 기본 한 변 1 m. 실제 크기 = size x scale
                color=np.array([1.0, 0, 0]),          # 빨간색
            )
        )

        # 이 Task 에 속한 물체로 등록
        self._task_objects[self._robot.name] = self._robot
        self._task_objects[self._target.name] = self._target

    # 언제: 매 world.reset() 끝에 (Stop -> Play 때도)
    # 할 일: 팔은 홈 자세, 그리퍼는 열림으로 순간이동 (타겟은 world.reset() 이 시작 위치로 되돌린다)
    def post_reset(self):
        home = np.zeros(self._robot.num_dof)          # 관절 12개 값을 담을 배열 (팔 6 + 그리퍼 6)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle
        self._robot.set_joint_positions(home)         # 팔을 홈 자세로 순간이동
        self._robot.gripper.set_joint_positions(self._robot.gripper.joint_opened_positions)  # 그리퍼 열기

    # 언제: world.get_observations() 를 부를 때
    def get_observations(self):
        target_pos, target_orient = self._target.get_world_pose()  # 뷰포트에서 옮긴 지금 위치와 자세
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),  # 관절 12개의 지금 각도 (rad)
            },
            self._target.name: {
                "position": target_pos,                                # [x, y, z] (m)
                "orientation": target_orient,                          # 쿼터니언 [w, x, y, z]
            },
        }

    # 언제: 메인 루프가 물체 이름을 알아낼 때
    def get_params(self):
        return {
            "robot_name": {"value": self._robot.name},    # "m0609"
            "target_name": {"value": self._target.name},  # "target"
        }


# ------------------------------------------------------------
# (나) Controller: 매 프레임 타겟 위치/자세로 IK 를 풀어 팔 관절 6개 명령을 만든다
# ------------------------------------------------------------
class FollowTargetController(BaseController):
    """타겟을 따라가는 IK 컨트롤러. 답이 없으면 마지막 명령을 유지한다 (기다림)"""

    def __init__(self, name, ik_solver, arm_indices):
        super().__init__(name)                        # 컨트롤러 이름
        self._ik = ik_solver                          # IK 계산기
        self._arm_indices = arm_indices               # 팔 관절 6개가 관절 배열의 몇 번째인지
        self.reset()

    # 언제: 처음 만들 때, 그리고 메인 루프가 world.reset() 할 때
    # 할 일: 기억하는 상태를 처음 값으로
    def reset(self):
        self.following = True     # 지금 따라가는 중인지 (마지막 IK 가 성공했는지)
        self._last_action = None  # 마지막으로 보낸 명령 (답이 없을 때 이것을 유지한다)

    # 팔 형태가 홈 자세와 같은지: 팔꿈치(joint_3)와 손목(joint_5)이 홈처럼 양수 쪽으로 굽어 있어야 한다
    #   이 검사가 없으면 멀리 끌었다가 돌아올 때 [-1, 76, -67, 0, 172, -1] 처럼 뒤집힌 형태로 남는다
    @staticmethod
    def _same_shape_as_home(joints):
        return joints[2] > 0 and joints[4] > 0

    # IK 한 번: 성공하고 팔 형태도 홈과 같을 때만 답으로 인정한다
    def _solve(self, target_pos, target_orient, warm_start):
        joints, success = self._ik.compute_inverse_kinematics(
            TCP_FRAME,                                # 목표에 맞출 프레임
            target_pos,                               # 목표 위치 [x, y, z] (m)
            target_orient,                            # 목표 자세 (쿼터니언 [w, x, y, z])
            warm_start=warm_start,                    # 이 관절값 근처에서 답을 찾는다 (rad)
        )
        if success and self._same_shape_as_home(joints):
            return joints
        return None

    # 언제: 매 프레임 메인 루프가 부른다
    # 받는 값: 타겟 위치, 타겟 자세, 팔 관절 6개의 지금 각도 (rad)
    # 돌려줄 값: ArticulationAction (답도 없고 보낸 적도 없으면 None)
    def forward(self, target_pos, target_orient, current_arm):
        # 1) 지금 자세 근처에서 푼다 (프레임마다 조금씩 움직이므로 가까운 답이 자연스럽다)
        joints = self._solve(target_pos, target_orient, warm_start=current_arm)
        # 2) 안 되면 (해가 없거나 팔 형태가 뒤집힌 답) 홈 자세 근처에서 한 번 더 푼다
        if joints is None:
            joints = self._solve(target_pos, target_orient, warm_start=np.radians(HOME_DEG))

        self.following = joints is not None
        if self.following:
            self._last_action = ArticulationAction(
                joint_positions=joints,               # 팔 관절 6개 목표 (rad)
                joint_indices=self._arm_indices,      # 6개 값을 넣을 자리 (그리퍼는 그대로)
            )
        # 3) 답이 없으면 마지막 명령을 그대로 보낸다 -> 그 자리에서 기다린다
        return self._last_action


# ------------------------------------------------------------
# (다) 등록과 준비
# ------------------------------------------------------------
world = World(
    stage_units_in_meters=1.0,                        # 길이 단위: 1 = 1 m
    physics_dt=1.0 / 120.0,                           # 물리 계산 주기 (s)
    rendering_dt=1.0 / 60.0,                          # 한 프레임 시간 (s)
)
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
task = FollowTargetTask(name="follow_target")         # Task 이름
world.add_task(task)                                  # World 에 Task 등록
world.reset()  # 여기서 task.set_up_scene() -> task.post_reset() 이 불린다

params = task.get_params()
robot_name = params["robot_name"]["value"]            # "m0609"
target_name = params["target_name"]["value"]          # "target"
robot = world.scene.get_object(robot_name)
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]  # 팔 관절 6개가 관절 배열(12개)의 몇 번째인지
controller = FollowTargetController(
    "follow_target_controller",                       # 컨트롤러 이름
    ik_solver=ik_solver,                              # IK 계산기
    arm_indices=arm,                                  # 팔 관절 6개가 몇 번째인지
)

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
reset_needed = False
last_following = None  # 바로 전 프레임의 상태 (따라감 / 기다림 이 바뀌면 출력)
while simulation_app.is_running():
    world.step(render=True)                           # 한 프레임 진행

    # 1) Stop 을 감지해 둔다
    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if world.is_playing():
        # 2) Stop 다음 첫 Play 프레임: 장면과 컨트롤러를 함께 처음으로
        if reset_needed:
            world.reset()
            controller.reset()
            last_following = None
            reset_needed = False
            print("reset 완료: 처음부터 다시 시작")
        # 3) 관측 (Task)
        observations = world.get_observations()
        current_arm = observations[robot_name]["joint_positions"][arm]  # 12개 중 팔 6개만
        target_pos = observations[target_name]["position"]
        target_orient = observations[target_name]["orientation"]
        # 4) 계산 (Controller)
        action = controller.forward(target_pos, target_orient, current_arm)
        # 5) 명령
        if action is not None:
            robot.apply_action(action)

        # 따라감 / 기다림 이 바뀔 때 한 줄 출력
        if controller.following != last_following:
            x, y, z = target_pos
            state = "따라감" if controller.following else "기다림 (IK 해 없음)"
            print(f"시각 {world.current_time:5.2f} s | {state} | 타겟 x {x:5.2f} y {y:5.2f} z {z:.2f} m"
                  f" | joint_1 ~ 6 {np.round(np.degrees(current_arm)).astype(int)} deg")
            last_following = controller.following

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
