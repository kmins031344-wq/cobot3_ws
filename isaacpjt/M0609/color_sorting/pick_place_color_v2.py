"""[색상 분류 v2] 파랑/초록 큐브를 집어 같은 색 마커 위에 놓기 (PC A: Isaac Sim)

v1 과 다른 점: 액션 그래프(OmniGraph)를 쓰지 않는다
  v1: 카메라 영상을 OmniGraph(RenderProduct -> ROS2CameraHelper)가 /rgb 로 발행
  v2: 카메라 영상을 Camera 클래스로 직접 읽어 rclpy 노드가 /rgb 로 발행
      -> ROS2 발행(/rgb)과 구독(/color_id, /rosout)을 모두 rclpy 노드 하나가 메인 루프에서 처리한다

흐름
  1. Play(리셋) 때마다 큐브 1개의 색(파랑/초록)과 위치(pick 영역)를 랜덤으로 정한다
  2. 손목 카메라(D455) 영상을 읽어 rclpy 로 /rgb (sensor_msgs/Image) 를 보낸다
  3. 큐브 위로 가서 PC B 의 color_detector 가 보내는 /color_id (std_msgs/Int32) 를 기다린다
       1 = 파랑 -> 파란 마커 위치, 2 = 초록 -> 초록 마커 위치
  4. 집어서 같은 색 마커 위에 놓는다

랜덤 위치를 고르는 방법 (IK 없이 L7 의 관절 자세를 그대로 쓰기 위해)
  로봇에서의 거리는 0.45 m 로 고정하고 방향(yaw)만 랜덤으로 고른다.
  -> joint_1 만 그 방향으로 돌리면 L7 의 ABOVE/PICK 자세를 그대로 쓸 수 있다.
  큐브도 같은 각도만큼 돌려 놓아 그리퍼 손가락과 면이 나란하게 한다.

실행:
  isaac_ros                      # Isaac Sim 내부 ROS2 브릿지 환경 설정
  isaac_python pick_place_color_v2.py
"""

import random
from pathlib import Path

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
from isaacsim.core.utils.extensions import enable_extension

enable_extension("isaacsim.ros2.bridge")  # ROS2 브릿지 켜기 (그래프는 안 쓰지만, Isaac 내부 rclpy 를 쓰려면 필요하다)
simulation_app.update()

import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import DynamicCuboid, GroundPlane, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.sensors.camera import Camera

import rclpy  # 브릿지 확장을 켠 뒤에 import 해야 Isaac 내부 rclpy 를 찾는다
from builtin_interfaces.msg import Time
from rcl_interfaces.msg import Log
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

# ------------------------------------------------------------
# 설정값
# ------------------------------------------------------------
ROBOT_USD = Path(__file__).resolve().parents[1] / "robot" / "m0609_rg2_d455.usd"
ROBOT_PRIM = "/World/m0609_rg2_d455"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

# L7 과 같은 자세 (joint_1 = 0 일 때). 실제로 쓸 때는 joint_1 자리에 방향(yaw)을 넣는다
HOME_DEG = [0, 0, 90, 0, 90, 0]
ABOVE_DEG = [0, 12, 83, 0, 85, 0]   # 그리퍼 끝이 바닥에서 약 0.2 m (카메라가 큐브를 내려다보는 자세)
PICK_DEG = [0, 18, 101, 0, 61, 0]   # 그리퍼 끝이 큐브 높이

REACH = 0.45                        # 로봇에서 큐브/마커까지 거리 (m). 위 자세들이 이 거리에 맞춰져 있다
CUBE_SIZE = 0.0515                  # 큐브 한 변 (m)
CUBE_Z = CUBE_SIZE / 2              # 바닥에 놓인 큐브 중심 높이
PICK_YAW_RANGE = (-25.0, 25.0)      # pick 영역: 정면 기준 이 각도 범위에서 랜덤 (deg)

# 색 정보: color_id -> (이름, RGB, 마커 방향 yaw(deg))
#   마커는 좌우 90 deg 에 둔다: pick 위치에서 카메라에 마커가 같이 찍히지 않을 만큼 멀리
COLORS = {
    1: {"name": "파랑", "rgb": [0.0, 0.0, 1.0], "marker_yaw": 90.0},
    2: {"name": "초록", "rgb": [0.0, 1.0, 0.0], "marker_yaw": -90.0},
}

# 손목 카메라 -> ROS2
CAMERA_PRIM = f"{ROBOT_PRIM}/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"  # R2 와 같은 컬러 카메라
RGB_TOPIC = "rgb"                   # 토픽 이름 (/rgb)
CAM_FRAME_ID = "wrist_camera"       # 메시지 header 의 frame_id
CAM_WIDTH, CAM_HEIGHT = 320, 240    # 다른 PC 로 보내므로 작게 (네트워크 부담 줄이기)
CAM_PUBLISH_EVERY = 6               # Play 중 6 프레임마다 1 장 발행 -> 60 fps 기준 약 10 Hz
COLOR_TOPIC = "color_id"            # 결과 토픽 이름 (/color_id)
DETECTOR_NODE = "color_detector_node"  # PC B 색 감지 노드 이름. 이 노드의 로그를 PC A 터미널에도 그대로 찍는다

TOLERANCE_DEG = 1.0                 # 팔 관절이 모두 목표와 이만큼 안으로 들어오면 "도착"
GRIPPER_FRAMES = 60                 # 그리퍼 단계 대기 프레임 (1 초)
DETECT_SETTLE_FRAMES = 30           # 큐브 위 도착 후 카메라 영상이 안정될 때까지 기다리는 프레임
DETECT_CONFIRM = 5                  # 같은 color_id 가 이만큼 연속으로 오면 확정
DETECT_WARN_FRAMES = 600            # 이만큼(10 초) 지나도 결과가 없으면 안내 메시지

# 단계 목록: (이름, 종류, 값)
#   "arm"     : 값 = (방향, 자세)  방향은 "pick"/"place"/"home" -> joint_1 에 들어갈 yaw 를 고른다
#   "gripper" : 값 = "open" / "close"
#   "detect"  : /color_id 를 기다렸다가 놓을 방향(place yaw)을 정한다
STEPS = [
    ("큐브 위로", "arm", ("pick", ABOVE_DEG)),
    ("색 확인", "detect", None),
    ("내려가기", "arm", ("pick", PICK_DEG)),
    ("그리퍼 닫기", "gripper", "close"),
    ("들어 올리기", "arm", ("pick", ABOVE_DEG)),
    ("마커 위로", "arm", ("place", ABOVE_DEG)),
    ("내려놓기", "arm", ("place", PICK_DEG)),
    ("그리퍼 열기", "gripper", "open"),
    ("올라가기", "arm", ("place", ABOVE_DEG)),
    ("홈으로", "arm", ("home", HOME_DEG)),
]


def polar(yaw_deg, r=REACH, z=0.0):
    """로봇 기준 (거리, 방향) -> [x, y, z]"""
    yaw = np.radians(yaw_deg)
    return np.array([r * np.cos(yaw), r * np.sin(yaw), z])


# ------------------------------------------------------------
# (가) Task: 로봇 + 큐브 1개 + 색 마커 2개
# ------------------------------------------------------------
class ColorSortTask(BaseTask):
    """장면을 만들고, 리셋마다 큐브의 색과 위치를 랜덤으로 정한다"""

    def __init__(self, name):
        super().__init__(name)
        self.target_id = None   # 이번 판에 집을 큐브 색 (1 or 2). 정답 확인용
        self.pick_yaw = 0.0     # 이번 판 큐브 방향 (deg)

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        # 바닥: 흰색 (색 감지에 방해되지 않도록 파랑/초록이 아닌 무채색)
        scene.add(
            GroundPlane(
                prim_path="/World/GroundPlane",
                name="ground",
                size=10.0,                         # 한 변 10 m
                color=np.array([1.0, 1.0, 1.0]),   # 흰색
            )
        )

        add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path=ROBOT_PRIM)
        gripper = ParallelGripper(
            end_effector_prim_path=f"{ROBOT_PRIM}/gripper_body",
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([0.0]),
            joint_closed_positions=np.array([1.0]),
            use_mimic_joints=True,
        )
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM,
                name="m0609",
                end_effector_prim_path=f"{ROBOT_PRIM}/gripper_body",
                gripper=gripper,
            )
        )
        self._task_objects[self._robot.name] = self._robot

        # 큐브 1개 (색과 위치는 post_reset 에서 매번 랜덤으로 바꾼다)
        self._cube = scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",
                name="cube",
                position=polar(0.0, z=CUBE_Z),
                scale=np.array([CUBE_SIZE] * 3),
                size=1.0,
                color=np.array(COLORS[1]["rgb"]),  # 처음 색. 색을 줘야 바꿀 수 있는 재질(material)이 만들어진다
            )
        )
        self._task_objects[self._cube.name] = self._cube

        # 색 마커 2개: 바닥에 붙은 얇은 판 (충돌 없음 -> 큐브를 그 위에 내려놓는다)
        for cid, info in COLORS.items():
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/Marker_{cid}",
                    name=f"marker_{cid}",
                    position=polar(info["marker_yaw"], z=0.001),
                    orientation=euler_angles_to_quat(np.array([0, 0, info["marker_yaw"]]), degrees=True),
                    scale=np.array([0.09, 0.09, 0.002]),
                    size=1.0,
                    color=np.array(info["rgb"]),
                )
            )

    def post_reset(self):
        # 팔: 홈 자세, 그리퍼: 열림
        home = np.zeros(self._robot.num_dof)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle
        self._robot.set_joint_positions(home)
        self._robot.gripper.set_joint_positions(self._robot.gripper.joint_opened_positions)

        # 큐브 랜덤: 색(파랑/초록)과 pick 영역 안의 방향
        self.target_id = random.choice(list(COLORS))
        self.pick_yaw = random.uniform(*PICK_YAW_RANGE)
        self._cube.get_applied_visual_material().set_color(np.array(COLORS[self.target_id]["rgb"]))
        self._cube.set_world_pose(
            position=polar(self.pick_yaw, z=CUBE_Z),
            # 그리퍼와 면이 나란하도록 큐브도 같은 각도로 돌린다
            orientation=euler_angles_to_quat(np.array([0, 0, self.pick_yaw]), degrees=True),
        )
        self._cube.set_linear_velocity(np.zeros(3))
        self._cube.set_angular_velocity(np.zeros(3))
        print(f"[스폰] {COLORS[self.target_id]['name']} 큐브 -> 방향 {self.pick_yaw:+.1f} deg (거리 {REACH} m)")

    def get_observations(self):
        return {
            self._robot.name: {"joint_positions": self._robot.get_joint_positions()},
            self._cube.name: {"position": self._cube.get_world_pose()[0]},
        }

    def get_params(self):
        return {"robot_name": {"value": self._robot.name}}


# ------------------------------------------------------------
# (나) ROS2: rclpy 노드 하나로 /rgb 발행 + /color_id 구독 (메인 루프에서 spin_once 로 처리)
# ------------------------------------------------------------
class IsaacRosNode:
    """손목 카메라 영상을 /rgb 로 보내고, /color_id 로 받은 값을 쌓아 두고, PC B 색 감지 노드의 로그를 찍는다"""

    LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("isaac_color_sort")
        self.rgb_pub = self.node.create_publisher(Image, RGB_TOPIC, 10)  # v1 의 ROS2CameraHelper 역할
        self.node.create_subscription(Int32, COLOR_TOPIC, self._callback, 10)
        # 모든 노드의 로그는 /rosout 토픽으로도 나간다 -> PC B 감지 노드 로그만 골라서 그대로 출력
        self.node.create_subscription(Log, "/rosout", self._log_callback, 100)
        self.history = []  # 받은 color_id 들 (detect 단계 시작 때 비운다)

    def _log_callback(self, msg):
        if msg.name != DETECTOR_NODE:
            return
        level = self.LEVELS.get(msg.level, str(msg.level))
        stamp = f"{msg.stamp.sec}.{msg.stamp.nanosec:09d}"
        print(f"[{level}] [{stamp}] [{msg.name}]: {msg.msg}")

    def _callback(self, msg):
        self.history.append(int(msg.data))
        if len(self.history) > 100:
            self.history = self.history[-100:]

    def publish_rgb(self, rgb, sim_time):
        """(H, W, 3) uint8 영상 -> sensor_msgs/Image 로 /rgb 발행. 시각은 시뮬레이션 시간 (/clock 과 같은 기준)"""
        msg = Image()
        msg.header.stamp = Time(sec=int(sim_time), nanosec=int((sim_time % 1.0) * 1e9))
        msg.header.frame_id = CAM_FRAME_ID
        msg.height, msg.width = rgb.shape[:2]
        msg.encoding = "rgb8"           # PC B 의 color_detector 가 rgb8 을 BGR 로 바꿔 쓴다
        msg.is_bigendian = 0
        msg.step = msg.width * 3        # 한 줄 바이트 수
        msg.data = np.ascontiguousarray(rgb, dtype=np.uint8).tobytes()
        self.rgb_pub.publish(msg)

    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)  # 기다리지 않고 도착한 메시지만 처리

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


# ------------------------------------------------------------
# (다) 손목 카메라: Camera 클래스로 영상 읽기 (v1 의 RenderProduct 노드 역할)
# ------------------------------------------------------------
def create_wrist_camera():
    """로봇 USD 안의 D455 컬러 카메라 prim 을 감싸 영상을 읽을 수 있게 한다. world.reset() 뒤에 부른다"""
    camera = Camera(prim_path=CAMERA_PRIM, name="wrist_camera", resolution=(CAM_WIDTH, CAM_HEIGHT))
    camera.initialize()  # 렌더 결과(render product)와 rgb annotator 를 붙인다
    print(f"[카메라] {CAMERA_PRIM} -> /{RGB_TOPIC} ({CAM_WIDTH}x{CAM_HEIGHT}, {CAM_PUBLISH_EVERY} 프레임마다)")
    return camera


# ------------------------------------------------------------
# (라) Controller: 단계 목록을 하나씩 진행 (상태 머신)
# ------------------------------------------------------------
class ColorSortController(BaseController):
    def __init__(self, name, gripper, arm_indices, listener):
        super().__init__(name)
        self._gripper = gripper
        self._arm_indices = arm_indices
        self._listener = listener
        self.reset(pick_yaw=0.0)

    def reset(self, pick_yaw):
        self.step = 0
        self._step_frames = 0
        self.pick_yaw = pick_yaw   # 큐브 방향 (Task 가 정한 값)
        self.place_yaw = None      # 놓을 방향 (색 확인 후 정해진다)
        self.color_id = None       # 감지된 색

    def is_done(self):
        return self.step >= len(STEPS)

    def _yaw(self, where):
        return {"pick": self.pick_yaw, "place": self.place_yaw, "home": HOME_DEG[0]}[where]

    def forward(self, current_arm):
        if self.is_done():
            return None
        name, kind, value = STEPS[self.step]
        self._step_frames += 1

        if kind == "arm":
            where, pose = value
            target = [self._yaw(where)] + list(pose[1:])  # joint_1 자리에 방향을 넣는다
            action = ArticulationAction(joint_positions=np.radians(target), joint_indices=self._arm_indices)
            finished = np.max(np.abs(np.degrees(current_arm) - target)) < TOLERANCE_DEG
        elif kind == "gripper":
            action = self._gripper.forward(action=value)
            finished = self._step_frames >= GRIPPER_FRAMES
        else:  # detect: 팔은 그대로 두고 /color_id 를 기다린다
            action = None
            finished = self._detect()

        if finished:
            self.step += 1
            self._step_frames = 0
        return action

    def _detect(self):
        if self._step_frames <= DETECT_SETTLE_FRAMES:
            self._listener.history.clear()  # 이동 중에 들어온 값은 버린다
            return False
        recent = self._listener.history[-DETECT_CONFIRM:]
        if len(recent) == DETECT_CONFIRM and len(set(recent)) == 1 and recent[0] in COLORS:
            self.color_id = recent[0]
            self.place_yaw = COLORS[self.color_id]["marker_yaw"]
            return True
        if self._step_frames % DETECT_WARN_FRAMES == 0:
            print(f"[대기] /{COLOR_TOPIC} 결과 없음 (최근 값 {recent}). PC B 의 color_detector 와 ROS_DOMAIN_ID 확인")
        return False


# ------------------------------------------------------------
# (마) 등록과 준비
# ------------------------------------------------------------
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
task = ColorSortTask(name="color_sort")
world.add_task(task)
world.reset()  # set_up_scene() -> post_reset() (여기서 첫 랜덤 배치)

camera = create_wrist_camera()
listener = IsaacRosNode()

robot = world.scene.get_object(task.get_params()["robot_name"]["value"])
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]
controller = ColorSortController("color_sort_controller", gripper=robot.gripper, arm_indices=arm, listener=listener)
controller.reset(pick_yaw=task.pick_yaw)

# ============================================================
# 3. 메인 루프
# ============================================================
reset_needed = False
last_step = 0
play_frame = 0  # Play 중에 지난 프레임 수 (카메라 발행 주기 계산용)
while simulation_app.is_running():
    world.step(render=True)
    listener.spin()

    # 카메라 -> /rgb: Play 중에만, CAM_PUBLISH_EVERY 프레임마다 (v1 의 OnPlaybackTick + frameSkipCount 역할)
    if world.is_playing():
        play_frame += 1
        if play_frame % CAM_PUBLISH_EVERY == 0:
            rgb = camera.get_rgb()
            if rgb is not None and rgb.size > 0:  # 처음 몇 프레임은 렌더 결과가 아직 없을 수 있다
                listener.publish_rgb(rgb, world.current_time)

    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if world.is_playing():
        if reset_needed:  # Stop 다음 첫 Play: 새로 랜덤 배치하고 처음부터
            world.reset()
            controller.reset(pick_yaw=task.pick_yaw)
            last_step = 0
            reset_needed = False
            print("reset 완료: 새 큐브로 다시 시작")

        observations = world.get_observations()
        current_arm = observations[robot.name]["joint_positions"][arm]
        action = controller.forward(current_arm)
        if action is not None:
            robot.apply_action(action)

        if controller.step != last_step:
            done_name = STEPS[last_step][0]
            if STEPS[last_step][1] == "detect":
                got = COLORS[controller.color_id]["name"]
                answer = COLORS[task.target_id]["name"]
                mark = "정답" if controller.color_id == task.target_id else "오답!"
                print(f"시각 {world.current_time:5.2f} s | 색 확인 끝 | color_id={controller.color_id} ({got}) "
                      f"| 실제 {answer} -> {mark} | 놓을 방향 {controller.place_yaw:+.0f} deg")
            else:
                print(f"시각 {world.current_time:5.2f} s | {done_name} 끝")
            last_step = controller.step
            if controller.is_done():
                cube_pos = observations["cube"]["position"]
                marker_pos = polar(COLORS[controller.color_id]["marker_yaw"])
                dist = np.linalg.norm(cube_pos[:2] - marker_pos[:2])
                print(f"모든 단계 완료 | 큐브-마커 거리 {dist * 100:.1f} cm | Stop -> Play 로 다시")

# ============================================================
# 4. 종료
# ============================================================
listener.shutdown()
simulation_app.close()
