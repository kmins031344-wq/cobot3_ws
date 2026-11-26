"""[색상 분류] 파랑/초록 큐브를 집어 같은 색 마커 위에 놓기 (PC A: Isaac Sim)

흐름
  1. Play(리셋) 때마다 큐브 1개의 색(파랑/초록)과 위치(pick 영역)를 랜덤으로 정한다
  2. 손목 카메라(D455) 영상을 ROS2 토픽 /rgb (sensor_msgs/Image) 로 보낸다
  3. 큐브 위로 가서 PC B 의 color_detector 가 보내는 /color_id (std_msgs/Int32) 를 기다린다
       1 = 파랑 -> 파란 마커 위치, 2 = 초록 -> 초록 마커 위치
  4. 집어서 같은 색 마커 위에 놓는다

ROS2 발행 토픽 (R1 + R2 적용, Play 중)
  /clock        rosgraph_msgs/Clock     시뮬레이션 시간 (R1 Clock Graph)
  /rgb          sensor_msgs/Image       손목 D455 컬러 카메라 -> PC B 색 감지에 쓰임 (R2)
  /depth        sensor_msgs/Image       같은 카메라의 깊이 영상 (R2)
  /camera_info  sensor_msgs/CameraInfo  카메라 내부 파라미터 (R2)
  /rgb_left     sensor_msgs/Image       D455 왼쪽 카메라 (R2 연습 문제: 두 눈으로 보기)

랜덤 위치를 고르는 방법 (IK 없이 L7 의 관절 자세를 그대로 쓰기 위해)
  로봇에서의 거리는 0.45 m 로 고정하고 방향(yaw)만 랜덤으로 고른다.
  -> joint_1 만 그 방향으로 돌리면 L7 의 ABOVE/PICK 자세를 그대로 쓸 수 있다.
  큐브도 같은 각도만큼 돌려 놓아 그리퍼 손가락과 면이 나란하게 한다.

실행:
  isaac_ros                      # Isaac Sim 내부 ROS2 브릿지 환경 설정
  isaac_python pick_place_color.py
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

enable_extension("isaacsim.ros2.bridge")  # ROS2 브릿지 켜기 (rclpy, ROS2 OmniGraph 노드가 이 뒤에 쓸 수 있게 된다)
simulation_app.update()

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.usd
from isaacsim.core.api import World
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import DynamicCuboid, GroundPlane, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from pxr import Gf, Sdf, UsdGeom

import rclpy  # 브릿지 확장을 켠 뒤에 import 해야 Isaac 내부 rclpy 를 찾는다
from rcl_interfaces.msg import Log
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
D455_PRIM = f"{ROBOT_PRIM}/angle_bracket/realsense_d455/RSD455"
CAMERA_LEFT_PRIM = f"{D455_PRIM}/Camera_OmniVision_OV9782_Left"  # 왼쪽 카메라 -> /rgb_left (R2 연습 문제)
CLOCK_GRAPH_PATH = "/World/Graph/ROS_Clock"      # R1 과 같은 위치
CAMERA_GRAPH_PATH = "/World/Graph/camera_graph"  # R2 와 같은 위치
RGB_TOPIC = "rgb"                   # 토픽 이름 (/rgb)
DEPTH_TOPIC = "depth"               # /depth
CAMERA_INFO_TOPIC = "camera_info"   # /camera_info
RGB_LEFT_TOPIC = "rgb_left"         # /rgb_left
CAM_WIDTH, CAM_HEIGHT = 320, 240    # 다른 PC 로 보내므로 작게 (네트워크 부담 줄이기)
CAM_FRAME_SKIP = 5                  # 60 fps 중 5 프레임 건너뛰고 1 장 -> 약 10 Hz
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
# (나) ROS2: /color_id 구독 (rclpy, 메인 루프에서 spin_once 로 처리)
# ------------------------------------------------------------
class ColorIdListener:
    """/color_id 로 받은 값을 차례로 쌓아 두고, PC B 색 감지 노드의 로그를 이 터미널에 찍는다"""

    LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("isaac_color_listener")
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

    def spin(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)  # 기다리지 않고 도착한 메시지만 처리

    def shutdown(self):
        self.node.destroy_node()
        rclpy.shutdown()


# ------------------------------------------------------------
# (다) R1 Clock Graph + R2 Camera Graph (손목 카메라 -> /rgb, /depth, /camera_info, /rgb_left)
# ------------------------------------------------------------
def find_wrist_camera():
    """로봇 USD 안에서 카메라 prim 을 찾는다 (D455 의 컬러 카메라 우선). 없으면 그리퍼에 하나 만든다"""
    stage = omni.usd.get_context().get_stage()
    cameras = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.IsA(UsdGeom.Camera) and str(prim.GetPath()).startswith(ROBOT_PRIM)
    ]
    print(f"[카메라] 로봇 안 카메라 prim: {cameras}")
    color = [p for p in cameras if "color" in p.lower()]
    if color:
        return color[0]
    if cameras:
        return cameras[0]

    # 비상용: 그리퍼 본체에 아래(그리퍼 방향)를 보는 카메라를 붙인다
    path = f"{ROBOT_PRIM}/gripper_body/wrist_camera"
    cam = UsdGeom.Camera.Define(stage, path)
    cam.CreateFocalLengthAttr(18.0)
    xform = UsdGeom.Xformable(cam.GetPrim())
    xform.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.07, 0.05))
    # USD 카메라는 -Z 를 본다. 그리퍼 진행 방향(+Z)을 보도록 X 축으로 180 deg
    xform.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))
    print(f"[카메라] USD 에 카메라가 없어 새로 만들었다: {path} (방향이 이상하면 위 값 조정)")
    return path


def create_clock_graph():
    """[R1] Play 중 매 틱마다 시뮬레이션 시간을 /clock 으로 발행하는 Action Graph"""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": CLOCK_GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),          # Play 중 매 프레임 실행 신호
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),                 # ROS2 통신 설정 (Domain ID)
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),  # 시뮬레이션 시간 읽기
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),       # 시간을 토픽으로 발행
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("PublishClock.inputs:topicName", "clock"),                      # /clock
            ],
        },
    )
    print("[Clock] Clock Graph 생성:", CLOCK_GRAPH_PATH)


def create_camera_graph(camera_path, left_path=None):
    """[R2] Play 중 손목 카메라를 렌더해서 /rgb, /depth, /camera_info (+ /rgb_left) 로 발행하는 Action Graph

    left_path 가 있으면 연습 문제의 왼쪽 카메라도 /rgb_left 로 함께 발행한다
    """
    keys = og.Controller.Keys
    nodes = [
        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
        ("Context", "isaacsim.ros2.bridge.ROS2Context"),
        ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
    ]
    connects = [
        ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
        ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
        ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
        ("RenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
        ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
        ("RenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
        ("RenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
        ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
        ("Context.outputs:context", "RGBPublish.inputs:context"),
        ("Context.outputs:context", "DepthPublish.inputs:context"),
    ]
    values = [
        ("RenderProduct.inputs:width", CAM_WIDTH),
        ("RenderProduct.inputs:height", CAM_HEIGHT),
        ("CameraInfoPublish.inputs:topicName", CAMERA_INFO_TOPIC),
        ("CameraInfoPublish.inputs:frameId", "wrist_camera"),
        ("CameraInfoPublish.inputs:frameSkipCount", CAM_FRAME_SKIP),
        ("RGBPublish.inputs:type", "rgb"),
        ("RGBPublish.inputs:topicName", RGB_TOPIC),
        ("RGBPublish.inputs:frameId", "wrist_camera"),
        ("RGBPublish.inputs:frameSkipCount", CAM_FRAME_SKIP),
        ("DepthPublish.inputs:type", "depth"),
        ("DepthPublish.inputs:topicName", DEPTH_TOPIC),
        ("DepthPublish.inputs:frameId", "wrist_camera"),
        ("DepthPublish.inputs:frameSkipCount", CAM_FRAME_SKIP),
    ]
    if left_path:  # 연습 문제: 왼쪽 카메라용 RenderProduct 를 하나 더 만들어 /rgb_left 로 발행
        nodes += [
            ("RenderProductLeft", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RGBLeftPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ]
        connects += [
            ("OnPlaybackTick.outputs:tick", "RenderProductLeft.inputs:execIn"),
            ("RenderProductLeft.outputs:execOut", "RGBLeftPublish.inputs:execIn"),
            ("RenderProductLeft.outputs:renderProductPath", "RGBLeftPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "RGBLeftPublish.inputs:context"),
        ]
        values += [
            ("RenderProductLeft.inputs:width", CAM_WIDTH),
            ("RenderProductLeft.inputs:height", CAM_HEIGHT),
            ("RGBLeftPublish.inputs:type", "rgb"),
            ("RGBLeftPublish.inputs:topicName", RGB_LEFT_TOPIC),
            ("RGBLeftPublish.inputs:frameId", "wrist_camera_left"),
            ("RGBLeftPublish.inputs:frameSkipCount", CAM_FRAME_SKIP),
        ]

    og.Controller.edit(
        {"graph_path": CAMERA_GRAPH_PATH, "evaluator_name": "execution"},
        {keys.CREATE_NODES: nodes, keys.CONNECT: connects, keys.SET_VALUES: values},
    )
    # 어떤 카메라로 렌더할지 (relationship 이라 SET_VALUES 대신 직접 지정)
    stage = omni.usd.get_context().get_stage()
    targets = [("RenderProduct", camera_path)] + ([("RenderProductLeft", left_path)] if left_path else [])
    for node_name, cam in targets:
        rel = stage.GetPrimAtPath(f"{CAMERA_GRAPH_PATH}/{node_name}").GetRelationship("inputs:cameraPrim")
        rel.SetTargets([Sdf.Path(cam)])

    topics = [RGB_TOPIC, DEPTH_TOPIC, CAMERA_INFO_TOPIC] + ([RGB_LEFT_TOPIC] if left_path else [])
    print(f"[카메라] {camera_path} -> " + ", ".join(f"/{t}" for t in topics) + f" ({CAM_WIDTH}x{CAM_HEIGHT})")
    if left_path:
        print(f"[카메라] 왼쪽 {left_path} -> /{RGB_LEFT_TOPIC}")


def find_left_camera():
    """연습 문제의 D455 왼쪽 카메라. USD 에 없으면 None (그러면 /rgb_left 는 발행하지 않는다)"""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(CAMERA_LEFT_PRIM).IsValid():
        return CAMERA_LEFT_PRIM
    print(f"[카메라] 왼쪽 카메라가 없다: {CAMERA_LEFT_PRIM} -> /{RGB_LEFT_TOPIC} 는 건너뛴다")
    return None


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

create_clock_graph()                                      # R1: /clock
create_camera_graph(find_wrist_camera(), find_left_camera())  # R2: /rgb, /depth, /camera_info, /rgb_left
listener = ColorIdListener()

robot = world.scene.get_object(task.get_params()["robot_name"]["value"])
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]
controller = ColorSortController("color_sort_controller", gripper=robot.gripper, arm_indices=arm, listener=listener)
controller.reset(pick_yaw=task.pick_yaw)

# ============================================================
# 3. 메인 루프
# ============================================================
reset_needed = False
last_step = 0
while simulation_app.is_running():
    world.step(render=True)
    listener.spin()

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
