#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import time
import py_trees
import py_trees_ros
import rospy
import actionlib
from py_trees.common import Status
from py_trees.blackboard import Blackboard

# ROS Messages
# [修改] 移除了 BatteryState，导入 Float32MultiArray 用于接收我们解包的 [电压, 电流, 电量]
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String, Bool
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# ==========================================
# 1) 条件/动作节点
# ==========================================

class AutochargeEnabled(py_trees.behaviour.Behaviour):
    """
    [Condition] 自动充电总开关
    - 订阅 /commander/autocharge_enable
    - True -> SUCCESS (允许进入自动充电流程)
    - False -> FAILURE (禁止自动充电，直接跳过)
    """
    def __init__(self, name="AutochargeEnabled", topic="/commander/autocharge_enable", default=False):
        super().__init__(name)
        self.topic = topic
        self.enabled = bool(default)
        self.sub = None

    def setup(self, timeout=15):
        self.sub = rospy.Subscriber(self.topic, Bool, self._cb, queue_size=1)
        return True

    def _cb(self, msg: Bool):
        self.enabled = bool(msg.data)
        rospy.loginfo(f"[{self.name}] Switch Changed: {self.enabled}")

    def update(self):
        return Status.SUCCESS if self.enabled else Status.FAILURE


class BatteryLow(py_trees.behaviour.Behaviour):
    """
    [Condition] 电池低电（百分比稳定判定版）
    - 电量 pct < low_pct 持续 confirm_s 秒 -> SUCCESS
    - 否则 -> FAILURE
    """
    def __init__(self,
                 name="BatteryLow",
                 low_pct=20.0,     # [修改] 改为 20%
                 confirm_s=3.0,
                 topic="/rover/custom_hex_240",  # [修改] 订阅自定义解包话题
                 timeout_s=2.0):
        super().__init__(name)
        self.low_pct = float(low_pct)
        self.confirm_s = float(confirm_s)
        self.topic = topic
        self.timeout_s = float(timeout_s)

        self.pct = None
        self.last_stamp = rospy.Time(0)
        self.sub = None
        self._t_low = None 

    def setup(self, timeout=15):
        self.sub = rospy.Subscriber(self.topic, Float32MultiArray, self._cb, queue_size=1)
        return True

    def _cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.pct = float(msg.data[2]) # 第3个元素是电量百分比
            self.last_stamp = rospy.Time.now()

    def initialise(self):
        self._t_low = None

    def update(self):
        if self.pct is None or self.last_stamp == rospy.Time(0):
            return Status.FAILURE

        if (rospy.Time.now() - self.last_stamp).to_sec() > self.timeout_s:
            rospy.logwarn_throttle(2.0, f"[{self.name}] battery msg timeout")
            return Status.FAILURE

        p = self.pct

        if p < self.low_pct:
            if self._t_low is None:
                self._t_low = rospy.Time.now()

            if (rospy.Time.now() - self._t_low).to_sec() >= self.confirm_s:
                rospy.logwarn(
                    f"[{self.name}] LOW BATTERY stable: {p:.0f}% < {self.low_pct:.0f}%"
                )
                return Status.SUCCESS

            return Status.RUNNING

        self._t_low = None
        return Status.FAILURE


class IsCharging(py_trees.behaviour.Behaviour):
    def __init__(self, name="IsCharging", topic="/charging_state", timeout_s=1.0):
        super().__init__(name)
        self.topic = topic
        self.timeout_s = float(timeout_s)
        self.charging = False
        self.last_stamp = rospy.Time(0)
        self.sub = None

    def setup(self, timeout=15):
        self.sub = rospy.Subscriber(self.topic, Bool, self._cb, queue_size=1)
        return True

    def _cb(self, msg: Bool):
        self.charging = bool(msg.data)
        self.last_stamp = rospy.Time.now()

    def update(self):
        if self.last_stamp == rospy.Time(0):
            return Status.FAILURE
        if (rospy.Time.now() - self.last_stamp).to_sec() > self.timeout_s:
            rospy.logwarn_throttle(2.0, f"[{self.name}] /charging_state timeout")
            return Status.FAILURE
        return Status.SUCCESS if self.charging else Status.FAILURE


class ChargingStableFor(py_trees.behaviour.Behaviour):
    def __init__(self, name="ChargingStableFor", confirm_s=2.0, topic="/charging_state"):
        super().__init__(name)
        self.confirm_s = float(confirm_s)
        self.is_charging = IsCharging(topic=topic)
        self.t0 = None

    def setup(self, timeout=15):
        return self.is_charging.setup(timeout)

    def initialise(self):
        self.t0 = None

    def update(self):
        st = self.is_charging.update()
        if st == Status.SUCCESS:
            if self.t0 is None:
                self.t0 = rospy.Time.now()
            if (rospy.Time.now() - self.t0).to_sec() >= self.confirm_s:
                rospy.loginfo(f"[{self.name}] charging stable >= {self.confirm_s}s -> SUCCESS")
                return Status.SUCCESS
            return Status.RUNNING
        else:
            self.t0 = None
            return Status.RUNNING


class BatteryCharged(py_trees.behaviour.Behaviour):
    """
    [Condition] 充满判定：电量 > charged_pct 且 电流 < charged_cur，并持续 confirm_s 秒 -> SUCCESS
    """
    def __init__(self, name="BatteryCharged",
                 charged_pct=98.0,   # [修改] 目标电量 > 98%
                 charged_cur=1.2,    # [新增] 目标电流 < 1.2A
                 confirm_s=20.0,      # [修改] 满足条件后防抖 20 秒即判定充满
                 topic="/rover/custom_hex_240",
                 timeout_s=2.0):
        super().__init__(name)
        self.charged_pct = float(charged_pct)
        self.charged_cur = float(charged_cur)
        self.confirm_s = float(confirm_s)
        self.topic = topic
        self.timeout_s = float(timeout_s)

        self.pct = None
        self.cur = None  # [新增] 存放电流值
        self.last_stamp = rospy.Time(0)
        self.sub = None

        self._t0 = None 

    def setup(self, timeout=15):
        self.sub = rospy.Subscriber(self.topic, Float32MultiArray, self._cb, queue_size=1)
        return True

    def initialise(self):
        self._t0 = None

    def _cb(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.cur = float(msg.data[1]) # [新增] 提取数组第 2 位的电流值
            self.pct = float(msg.data[2]) # 提取数组第 3 位的电量值
            self.last_stamp = rospy.Time.now()

    def update(self):

        # 向黑板汇报当前状态
        p = self.pct if self.pct is not None else 0
        Blackboard().current_state = f" 充电中 ({p:.0f}%)"
        # 1. 检查是否收到数据
        if self.pct is None or self.cur is None or self.last_stamp == rospy.Time(0):
            rospy.logwarn_throttle(2.0, f"[{self.name}] 等待电池数据中...")
            return Status.RUNNING  
        if (rospy.Time.now() - self.last_stamp).to_sec() > self.timeout_s:
            rospy.logwarn_throttle(2.0, f"[{self.name}] 电池数据超时中断")
            return Status.RUNNING 

        p = self.pct
        c = self.cur

        # 2. [核心修改] 联合判断：电量 > 98 且 电流 < 1.2
        if p > self.charged_pct and c < self.charged_cur:
            if self._t0 is None:
                self._t0 = rospy.Time.now()
                rospy.loginfo(f"[{self.name}] 达到满电特征 (电量:{p:.0f}% > {self.charged_pct:.0f}%, 电流:{c:.1f}A < {self.charged_cur:.1f}A)，开始 5 秒确认...")

            dt = (rospy.Time.now() - self._t0).to_sec()
            if dt >= self.confirm_s:
                rospy.loginfo(f"[{self.name}] ✅ 充电彻底完成！最终状态: 电量={p:.0f}%, 电流={c:.1f}A")
                return Status.SUCCESS
            else:
                rospy.loginfo_throttle(1.0, f"[{self.name}] 满电防抖确认中... 还需 {self.confirm_s - dt:.0f} 秒")
                return Status.RUNNING
        else:
            # 3. 不满足条件，重置计时器，继续死等
            if self._t0 is not None:
                rospy.loginfo(f"[{self.name}] 满电条件中断 -> 当前电量:{p:.0f}%, 电流:{c:.1f}A")
            self._t0 = None
            rospy.loginfo_throttle(5.0, f"[{self.name}] 正在充电中... 当前电量:{p:.0f}%, 电流:{c:.1f}A")
            return Status.RUNNING


class AutonomousNavigation(py_trees.behaviour.Behaviour):
    def __init__(self, name, locations_db=None, target_key=None, action_name="move_base"):
        super().__init__(name)
        self.locations_db = locations_db or {}
        self.target_key = target_key
        self.action_name = action_name
        self.client = None
        self._sent = False
        self._active_cmd_seq = None
        self._active_target = None

    def setup(self, timeout=15):
        self.client = actionlib.SimpleActionClient(self.action_name, MoveBaseAction)
        rospy.loginfo(f"[{self.name}] connecting {self.action_name}...")
        self.client.wait_for_server(timeout=rospy.Duration(timeout))
        return True

    def initialise(self):
        self._sent = False
        pose_data = self._resolve_target_pose()
        if not pose_data:
            rospy.logerr(f"[{self.name}] no valid target pose")
            return

        goal = self._create_move_base_goal(pose_data)
        self.client.send_goal(goal)
        self._sent = True

        bb = Blackboard()
        self._active_cmd_seq = getattr(bb, "cmd_seq", None) if self.target_key is None else None
        self._active_target = getattr(bb, "cmd_target", None) if self.target_key is None else self.target_key
        rospy.loginfo(f"[{self.name}] goal sent target={self._active_target} cmd_seq={self._active_cmd_seq}")

    def update(self):

        # [新增] 向黑板汇报当前状态
        Blackboard().current_state = f" 导航中 -> {self._active_target or '用户目标'}"
        if not self.client or not self._sent:
            return Status.FAILURE

        if self.target_key is None:
            bb = Blackboard()
            latest_type = getattr(bb, "cmd_type", None)
            latest_seq = getattr(bb, "cmd_seq", None)
            latest_tgt = getattr(bb, "cmd_target", None)

            if latest_type == "NAV" and latest_seq is not None and latest_seq != self._active_cmd_seq:
                rospy.logwarn(f"[{self.name}] preempt -> new NAV seq={latest_seq}, target={latest_tgt}")
                try:
                    self.client.cancel_goal()
                except Exception as e:
                    rospy.logwarn(f"[{self.name}] cancel_goal failed: {e}")

                pose_data = self.locations_db.get(latest_tgt, None)
                if not pose_data:
                    rospy.logerr(f"[{self.name}] target not found: {latest_tgt}")
                    self._active_cmd_seq = latest_seq
                    return Status.FAILURE

                goal = self._create_move_base_goal(pose_data)
                self.client.send_goal(goal)

                self._active_cmd_seq = latest_seq
                self._active_target = latest_tgt
                return Status.RUNNING

        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"[{self.name}] arrived target={self._active_target}")
            return Status.SUCCESS

        if state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
            rospy.logwarn(f"[{self.name}] nav failed state={state} target={self._active_target}")
            return Status.FAILURE

        return Status.RUNNING

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.client and self._sent:
            if self.client.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                rospy.logwarn(f"[{self.name}] preempted by higher priority -> cancel goal")
                self.client.cancel_goal()

    def _resolve_target_pose(self):
        if self.target_key:
            return self.locations_db.get(self.target_key)
        bb = Blackboard()
        dynamic_key = getattr(bb, 'cmd_target', None)
        if dynamic_key and dynamic_key in self.locations_db:
            return self.locations_db[dynamic_key]
        return None

    @staticmethod
    def _create_move_base_goal(pose_data):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        p = goal.target_pose.pose
        p.position.x = pose_data['x']
        p.position.y = pose_data['y']
        p.position.z = pose_data.get('z', 0.0)
        p.orientation.x = pose_data['ox']
        p.orientation.y = pose_data['oy']
        p.orientation.z = pose_data['oz']
        p.orientation.w = pose_data['ow']
        return goal


class AutoDockAction(py_trees.behaviour.Behaviour):
    def __init__(self, name="AutoDockAction", action_name="/autodock_action", max_duration=60.0):
        super().__init__(name)
        self.action_name = action_name
        self.max_duration = max_duration
        self.client = None
        self.sent_goal = False
        self.start_time = None
        self.server_online = False 

    def setup(self, timeout=15):
        try:
            from autodock_core.msg import AutoDockingAction, AutoDockingGoal
            self.ActionType = AutoDockingAction
            self.GoalType = AutoDockingGoal
        except ImportError:
            rospy.logerr(f"[{self.name}] 无法导入消息包！请确认代码环境。")
            return False

        self.client = actionlib.SimpleActionClient(self.action_name, self.ActionType)
        rospy.loginfo(f"[{self.name}] Client 已创建 (等待运行时连接)...")
        return True

    def initialise(self):
        self.sent_goal = False
        self.server_online = False
        self.start_time = rospy.Time.now()

        wait_timeout = 3.0 
        rospy.loginfo(f"[{self.name}] 正在寻找 AutoDock Server (超时: {wait_timeout}s)...")
        
        if not self.client.wait_for_server(timeout=rospy.Duration(wait_timeout)):
            rospy.logerr(f"[{self.name}] 启动失败：未检测到 AutoDock Server！")
            return 

        self.server_online = True
        goal = self.GoalType()
        self.client.send_goal(goal)
        self.sent_goal = True
        rospy.loginfo(f"[{self.name}] 连接成功！开始对接...")

    def update(self):
        #向黑板汇报当前状态
        Blackboard().current_state = "⚡ 精准对接充电桩..."
        if not self.server_online:
            return Status.FAILURE
        if not self.client or not self.sent_goal:
            return Status.FAILURE
        if not self.client.wait_for_server(timeout=rospy.Duration(0.01)):
            rospy.logerr_throttle(1.0, f"[{self.name}] 运行中连接丢失！")
            return Status.FAILURE
        if (rospy.Time.now() - self.start_time).to_sec() > self.max_duration:
            rospy.logwarn(f"[{self.name}] 对接超时，强制停止。")
            self.client.cancel_goal()
            return Status.FAILURE

        state = self.client.get_state()
        if state in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
            return Status.RUNNING
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo(f"[{self.name}] 对接成功！")
            return Status.SUCCESS

        rospy.logwarn(f"[{self.name}] 对接失败 (Status: {state})")
        return Status.FAILURE

    def terminate(self, new_status):
        if new_status == Status.INVALID and self.server_online and self.client:
            if self.client.wait_for_server(timeout=rospy.Duration(0.1)):
                state = self.client.get_state()
                if state in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                    self.client.cancel_goal()


class CancelAutoDock(py_trees.behaviour.Behaviour):
    def __init__(self, name="CancelAutoDock", topic="/autodock_action/cancel"):
        super().__init__(name)
        self.topic = topic
        self.pub = None
        self.GoalID = None

    def setup(self, timeout=15):
        from actionlib_msgs.msg import GoalID
        self.GoalID = GoalID
        self.pub = rospy.Publisher(self.topic, self.GoalID, queue_size=1)
        return True

    def update(self):
        if self.pub and self.GoalID:
            msg = self.GoalID()
            self.pub.publish(msg)
        return Status.SUCCESS


class JsonCommandSubscriber(py_trees.behaviour.Behaviour):
    def __init__(self, name="JsonCmdListener", topic="/commander/cmd"):
        super().__init__(name)
        self.topic = topic
        self.bb = Blackboard()
        self.sub = None
        self._seq = 0

    def setup(self, timeout=15):
        self.sub = rospy.Subscriber(self.topic, String, self._cb, queue_size=1)
        return True

    def _cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            rospy.logwarn(f"[{self.name}] invalid json: {e}, raw={msg.data}")
            return

        cmd_type = str(data.get("type", "")).upper().strip()
        if not cmd_type:
            rospy.logwarn(f"[{self.name}] missing field 'type', raw={msg.data}")
            return

        target = data.get("target", None)

        self.bb.cmd_type = cmd_type
        self.bb.cmd_target = target

        self._seq += 1
        self.bb.cmd_seq = self._seq
        rospy.loginfo(f"[{self.name}] NEW CMD seq={self._seq} type={cmd_type}, target={target}")

    def update(self):
        return Status.SUCCESS if getattr(self.bb, "cmd_type", None) else Status.FAILURE


class CmdTypeIs(py_trees.behaviour.Behaviour):
    def __init__(self, name, expected):
        super().__init__(name)
        self.expected = expected
        self.bb = Blackboard()

    def update(self):
        return Status.SUCCESS if getattr(self.bb, "cmd_type", None) == self.expected else Status.FAILURE
    
class ResetCommandState(py_trees.behaviour.Behaviour):
    def __init__(self, name="ResetState"):
        super().__init__(name)
        self.bb = Blackboard()

    def update(self):
        self.bb.cmd_type = None
        self.bb.cmd_target = None
        self.bb.cmd_seq = None
        return Status.SUCCESS

class ForceCancelNav(py_trees.behaviour.Behaviour):
    """
    [Action] 无条件强杀 move_base 所有目标 
    """
    def __init__(self, name="ForceCancelNav", action_name="move_base"):
        super().__init__(name)
        self.action_name = action_name
        self.client = None

    def setup(self, timeout=15):
        self.client = actionlib.SimpleActionClient(self.action_name, MoveBaseAction)
        # 不阻塞死等，只要节点启动就行
        return True

    def update(self):
        if self.client:
            try:
                self.client.cancel_all_goals()
                rospy.logwarn(f"[{self.name}] 已向底层发送 cancel_all_goals 强制刹车！")
            except Exception as e:
                pass
        return Status.SUCCESS


class CancelMoveBase(py_trees.behaviour.Behaviour):
    """
    [Action] 与巡航强绑定的取消move_base目标的方法
    """
    def __init__(self, name="CancelMoveBase", action_name="move_base"):
        super().__init__(name)
        self.action_name = action_name
        self.client = None
        self.bb = Blackboard()
        self._handled_seq = -1

    def setup(self, timeout=15):
        self.client = actionlib.SimpleActionClient(self.action_name, MoveBaseAction)
        self.client.wait_for_server(timeout=rospy.Duration(timeout))

        if not hasattr(self.bb, "patrol_seq"):
            self.bb.patrol_seq = 0
        if not hasattr(self.bb, "patrol_enabled"):
            self.bb.patrol_enabled = False

        return True

    def update(self):
        cur_enabled = bool(getattr(self.bb, "patrol_enabled", False))
        cur_seq = int(getattr(self.bb, "patrol_seq", 0))

        if (not cur_enabled) and (cur_seq != self._handled_seq):
            rospy.logwarn(f"[{self.name}] cancel_all_goals() seq={cur_seq}")
            try:
                self.client.cancel_all_goals()   
            except Exception as e:
                rospy.logwarn(f"[{self.name}] cancel failed: {e}")
            self._handled_seq = cur_seq

        return Status.SUCCESS


class PatrolEnabled(py_trees.behaviour.Behaviour):
    def __init__(self, name="PatrolEnabled", topic="/commander/patrol_enable", default=False):
        super().__init__(name)
        self.topic = topic
        self.enabled = bool(default)
        self.sub = None
        self.bb = Blackboard()

    def setup(self, timeout=15):
        if not hasattr(self.bb, "patrol_enabled"):
            self.bb.patrol_enabled = self.enabled
        if not hasattr(self.bb, "patrol_seq"):
            self.bb.patrol_seq = 0

        self.sub = rospy.Subscriber(self.topic, Bool, self._cb, queue_size=1)
        return True

    def _cb(self, msg: Bool):
        self.enabled = bool(msg.data)
        self.bb.patrol_enabled = self.enabled
        self.bb.patrol_seq += 1   
        rospy.logwarn(f"[{self.name}] set enabled={self.enabled}, seq={self.bb.patrol_seq}")

    def update(self):
        return Status.SUCCESS if bool(getattr(self.bb, "patrol_enabled", False)) else Status.FAILURE

class PatrolDisabled(py_trees.behaviour.Behaviour):
    def __init__(self, name="PatrolDisabled"):
        super().__init__(name)
        self.bb = Blackboard()

    def setup(self, timeout=15):
        if not hasattr(self.bb, "patrol_enabled"):
            self.bb.patrol_enabled = False
        return True

    def update(self):
        return Status.SUCCESS if (not bool(getattr(self.bb, "patrol_enabled", False))) else Status.FAILURE


def make_patrol_controller(WAYPOINTS):
    patrol_once = py_trees.composites.Sequence(name="PatrolOnce", memory=True)
    patrol_once.add_children([
        AutonomousNavigation(name="Waypoint_A", locations_db=WAYPOINTS, target_key="POSE1"),
        py_trees.timers.Timer(name="Wait_A", duration=5.0),

        AutonomousNavigation(name="Waypoint_B", locations_db=WAYPOINTS, target_key="POSE2"),
        py_trees.timers.Timer(name="Wait_B", duration=10.0),

        AutonomousNavigation(name="Waypoint_C", locations_db=WAYPOINTS, target_key="POSE3"),
        py_trees.timers.Timer(name="Wait_C", duration=15.0),
    ])

    patrol_forever = py_trees.decorators.SuccessIsRunning(
        child=patrol_once,
        name="PatrolForever"
    )

    patrol_run = py_trees.composites.Sequence(name="PatrolRun", memory=False)
    patrol_run.add_children([
        PatrolEnabled(name="PatrolEnabled"),  
        patrol_forever
    ])

    patrol_stop = py_trees.composites.Sequence(name="PatrolStopNow", memory=False)
    patrol_stop.add_children([
        PatrolDisabled(name="PatrolDisabled"),
        CancelMoveBase(name="CancelMoveBase")  
    ])

    patrol_controller = py_trees.composites.Selector(name="PatrolController", memory=False)
    patrol_controller.add_children([patrol_stop, patrol_run])

    return patrol_controller

def pre_tick_handler(behaviour_tree):
    """在每一帧扫描前，先将状态默认重置为空闲"""
    Blackboard().current_state = " 待机空闲 (Idle)"


def post_tick_handler(behaviour_tree):
    """在每一帧扫描后，提取黑板中被执行节点覆盖的最新状态，并发布"""
    global state_pub
    current_state = getattr(Blackboard(), "current_state", " 待机空闲 (Idle)")
    if state_pub:
        state_pub.publish(String(data=current_state))


# ==========================================
# 2) 坐标数据库
# ==========================================

WAYPOINTS = {
    'POSE1':   {'x': -2.767, 'y': -2.63,  'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -0.7096,   'ow':  0.7045},
    'POSE2':   {'x': -5.25,  'y': -9.12,  'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -0.9999,   'ow':  0.006096},
    'POSE3':   {'x': -5.91,  'y': -2.71,  'z': 0.0, 'ox': 0.0, 'oy': 0.0, 'oz': -0.99999,  'ow':  0.002342},
    'CHARGER': {'x':-2.95,   'y': -3.56,  'z': 0.0, 'ox': 0.0, 'oy': 0.01, 'oz': 0.035,   'ow':  0.999}
}

# ==========================================
# 3) 行为树构建
# ==========================================

def build_behavior_tree():
    root = py_trees.composites.Selector(name="MainLogic", memory=False)
    
    # --- A: 用户指令（最高优先级） ---
    command_root = py_trees.composites.Sequence(name="UserCommandRoot", memory=False)
    dispatch = py_trees.composites.Selector(name="UserCommandDispatch", memory=False)

    # 1) NAV 分支
    user_nav_guard = py_trees.composites.Sequence(name="UserNavGuard", memory=False)
    user_nav_worker = py_trees.composites.Sequence(name="UserNavWorker", memory=True)
    user_nav_worker.add_children([
        CancelAutoDock(name="EnsureDockStopped", topic="/autodock_action/cancel"),
        AutonomousNavigation(name="NavToUserTarget", locations_db=WAYPOINTS, target_key=None),
        ResetCommandState() 
    ])
    
    user_nav_guard.add_children([
        CmdTypeIs(name="CmdIsNAV", expected="NAV"), 
        user_nav_worker
    ])

    # 2) CHARGE 分支
    user_charge_guard = py_trees.composites.Sequence(name="UserChargeGuard", memory=False)
    charge_logic_selector = py_trees.composites.Selector(name="ChargeTryCatch", memory=False)

    user_charge_worker = py_trees.composites.Sequence(name="UserChargeWorker", memory=True)
    user_charge_worker.add_children([
        AutonomousNavigation(name="NavToCharger_User", locations_db=WAYPOINTS, target_key="CHARGER"),
        AutoDockAction(name="AutoDockWithFeedback"),      
        ChargingStableFor(confirm_s=2.0),
        
        # [修改] 用户强制充电模式下，充满98%且电流小于1.2，稳定 20 秒就判定成功
        BatteryCharged(name="UserBatteryCharged", charged_pct=98.0, charged_cur=1.2, confirm_s=20.0),

        ResetCommandState()
    ])

    failure_handler = py_trees.composites.Sequence(name="ChargeFailureHandler")
    failure_handler.add_children([
        ResetCommandState() 
    ])

    charge_logic_selector.add_children([user_charge_worker, failure_handler])

    user_charge_guard.add_children([
        CmdTypeIs(name="CmdIsCHARGE", expected="CHARGE"), 
        charge_logic_selector  
    ])
    #用户急停指令
    user_stop_guard = py_trees.composites.Sequence(name="UserStopGuard", memory=False)
    user_stop_guard.add_children([
        CmdTypeIs(name="CmdIsSTOP", expected="STOP"),       # 1. 拦截到 STOP 指令
        ForceCancelNav(name="ForceStopNav"),                # 2. 强杀导航 (刹车)
        CancelAutoDock(name="ForceStopDock", topic="/autodock_action/cancel"), # 3. 强杀对接
        ResetCommandState()                                 # 4. 彻底擦除黑板指令
    ])
    dispatch.add_children([
        user_stop_guard,
        user_nav_guard,
        user_charge_guard 
    ])
    command_root.add_children([
        JsonCommandSubscriber(),
        dispatch
    ])

    # --- B: 自动充电（中优先级） ---
    charging_seq = py_trees.composites.Sequence(name="AutoChargeSequence", memory=True)
    charging_seq.add_children([
        AutochargeEnabled(default=False),     
        #  电量低于 20% 持续 5 秒则触发自动回充
        BatteryLow(low_pct=37.0, confirm_s=5.0),  
        AutonomousNavigation(name="NavToCharger", locations_db=WAYPOINTS, target_key='CHARGER'),
        AutoDockAction(name="AutoDockWithFeedback"),             
        ChargingStableFor(confirm_s=2.0),
        
        #  自动充电模式下，电量大于98%并且电流小于1.2A且至少持续20s，即充满 ，
        BatteryCharged(charged_pct=98.0, charged_cur=1.2, confirm_s=20.0),
        AutonomousNavigation(name="NavToCharger", locations_db=WAYPOINTS, target_key='CHARGER'),
    ])

    # --- C: 自动巡航（低优先级，默认） ---
    patrol_controller = make_patrol_controller(WAYPOINTS)

    root.add_children([ command_root, charging_seq, patrol_controller])
    return root


if __name__ == '__main__':
    rospy.init_node("robot_behavior_manager")
    
    # 注册 /robot/current_state 话题
    state_pub = rospy.Publisher('/robot/current_state', String, queue_size=1)

    root_node = build_behavior_tree()
    tree = py_trees_ros.trees.BehaviourTree(root_node)
    
    # 将钩子函数挂载到行为树上
    tree.add_pre_tick_handler(pre_tick_handler)
    tree.add_post_tick_handler(post_tick_handler)

    rospy.loginfo("initializing behavior tree...")
    tree.setup(timeout=15)

    rospy.loginfo("ready: listening /commander/cmd, /rover/custom_hex_240, /charging_state")

    try:
        tree.tick_tock(500)  # 2Hz
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("exit.")