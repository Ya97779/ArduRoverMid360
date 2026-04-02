#!/usr/bin/env python3

# Copyright 2021 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import math
import argparse

import rospy
from std_srvs.srv import Trigger
from sensor_msgs.msg import BatteryState

from autodock_core.autodock_utils import DockState
import autodock_core.autodock_utils as utils
from autodock_core.autodock_server import AutoDockServer, AutoDockConfig

##############################################################################


class DefaultAutoDockConfig(AutoDockConfig):
    """
    Default config is used in simulation
    # Note: all these tf computations are reference to robot base_link. Hence,
    """
    # General Configs
    cam_offset = 0.05           # camera to base link, for edge2edge distance
    front_dock = False
    tf_expiry: float = 1.0      # sec
    dock_timeout = 220          # sec
    controller_rate = 20.0       # hz

    # goal thresh
    stop_yaw_diff = 0.03        # radian
    stop_trans_diff = 0.02      # meters

    # frames and topics
    base_link = "body_foot"
    left_marker = "fiducial_10"
    right_marker = "fiducial_11"
    centre_marker = "fiducial_20"

    # velocity profiles
    linear_vel_range = (-0.3, 0.3)
    angular_vel_range = (-0.2, 0.2)

    max_linear_vel = 0.2         # m/s, for parallel.c and steer
    min_linear_vel = 0.1        # m/s, for lastmile
    max_angular_vel = 0.2       # rad/s
    min_angular_vel = 0.1       # rad/s

    # predock state
    max_parallel_offset = 0.16     # m, will move to parallel.c if exceeded
    predock_tf_samples = 5         # tf samples to avg, parallel.c validation

    # steer dock state
    offset_to_angular_vel = 2   # factor to convert y-offset to ang vel
    to_last_mile_dis = 0.3     # edge2edge distance where transition to LM
    to_last_mile_tol = 0.25     # transition tolerance from SD to LM

    # last mile
    stop_distance = 0.10        # edge2edge distance to stop from charger
    max_last_mile_odom = 0.20   # max last mile odom move without using marker

    # activate charger state
    enable_charger_srv = True
    check_battery_status = False
    check_battery_timeout = 1.0
    charger_srv_name = "trigger_charger_srv"
    battery_status_topic = "battery_state"

    # retry state
    retry_count = 1             # how many times to retry
    retry_retreat_dis = 0.4     # meters, distance retreat during retry

    # debug state
    debug_mode = True           # when False selectively turns on aruco detections only
                                # a valid action is in progress.


class AutoDockStateMachine(AutoDockServer):
    """
    Implementation of the AutoDock Server with a Simple State Machine. Also
    this describes the logic of which the control loop of each state.
    """

    def __init__(self,
                 config: DefaultAutoDockConfig,
                 run_server=False,
                 load_rosparam=False,
                 fake_clock=False):
        # /Note: This ugly fix is to solve time sync issue when nodes are
        # running on a different PC, which time in not sync. As we will only
        # wish to enable sim time on 'AutoDockServer Node', please ensures
        # this node is not starting along with other nodes.
        if fake_clock:
            rospy.logwarn("WARNING!!!! fake clock is in used. "
                          "Temporary set use_sim_time to true")
            rospy.set_param("/use_sim_time", True)

        rospy.init_node('auto_dock_node')

        if fake_clock:
            rospy.logwarn(
                "WARNING!!!! fake clock enabled! now disable use_sim_time")
            rospy.set_param("/use_sim_time", False)

        self.cfg = config
        if load_rosparam:
            self.init_params()

        super().__init__(self.cfg, run_server)
        self.dock_state = DockState.IDLE

    def init_params(self):
        rospy.loginfo("Getting AutoDockServer params from rosparams server")
        param_names = [attr for attr in dir(self.cfg) if not callable(
            getattr(self.cfg, attr)) and not attr.startswith("__")]

        # get private rosparam, if none use default
        for param_name in param_names:
            param_val = rospy.get_param(
                "~" + param_name, getattr(self.cfg, param_name))
            print(f" set param [{param_name}] to [{param_val}]")
            setattr(self.cfg, param_name, param_val)

            # brute check to ensure numerical config is positive
            if isinstance(param_val, (int, float)):
                assert param_val >= 0, f"[{param_name}] param should be +ve"

    def start(self) -> bool:
        """
        Start Docking Sequence
        """
        rospy.loginfo("Start Docking Action Now")
        self.init_params()
        self.set_aruco_detections(detection_state=True)
        rospy.sleep(self.sleep_period)

        current_retry_count = 0
        rospy.loginfo(f"Will attempt with {self.cfg.retry_count} retry")

        while(True):

            if(
                # self.do_predock()
                self.do_steer_dock()
                and self.do_last_mile()
                and self.do_activate_charger()
            ):
                self.publish_cmd()
                self.set_state(DockState.IDLE, "All Success!")
                self.set_aruco_detections(detection_state=False)
                return True

            # If Dock failed
            self.publish_cmd()
            rospy.logwarn("Failed to Dock attempt")

            # Break from a retry
            if(current_retry_count >= self.cfg.retry_count):
                self.set_aruco_detections(detection_state=False)
                break

            # check again if it failed because of canceled
            if self.check_cancel():
                self.set_aruco_detections(detection_state=False)
                break

            # Attempt a Retry
            current_retry_count += 1
            rospy.logwarn("Attempting retry: "
                          f"{current_retry_count}/{self.cfg.retry_count}")

            if not self.do_retry():
                rospy.logwarn("Not able to retry")
                self.set_aruco_detections(detection_state=False)
                break

        # Note: will not set_state IDLE here since we will wish to capture
        # the state which failed in the action result status
        self.publish_cmd()
        return False

    def do_retry(self) -> bool:
        """
        Attempt to retry the docking again, only retry by retreating if it
        failed in activate charger, last mile. if Failed with steerDock,
        will straightaway attempt predock without retreating.
        """
        if self.check_cancel():
            return False

        if self.dock_state in [DockState.ACTIVATE_CHARGER,
                               DockState.LAST_MILE]:
            self.set_state(DockState.RETRY, "Retry by retreating")
            _dis = self.cfg.retry_retreat_dis
            if self.cfg.front_dock:
                _dis *= -1
            return self.move_with_odom(_dis)
        elif self.dock_state == DockState.STEER_DOCK:
            self.set_state(DockState.RETRY, "skip retreat and try predock")
            return True
        else:
            rospy.loginfo("State is not retry-able")
            return False

    def do_parallel_correction(self, offset: float) -> bool:
        """
        A parallel "parking" correction will be executed if the y offset
        exceeds the allowable threshold. Note this is an open loop operation.
        :return: success
        """
        self.set_state(DockState.PARALLEL_CORRECTION,
                       f"activate parallel correction with {offset:.2f}m")
        return (
            # Step 1: turn -90 degrees respective to odom
            self.rotate_with_odom(-math.pi/2)
            # Step 2: parallel correction respective to odom
            and self.move_with_odom(offset)
            # Step 3: turn 90 degrees respective to odom
            and self.rotate_with_odom(math.pi/2)
        )

    def do_single_side_marker_rotate(self) -> bool:
        """
        This predock's substate which handles single side marker detection,
        adjust the yaw according to the pose estimation of one single marker
        :return : success
        """
        self.set_state(DockState.PREDOCK, "Try single marker yaw adjustment")

        left_tf = self.get_tf(self.cfg.left_marker)
        if left_tf is not None:
            rospy.logwarn(f"Rotate with left marker: {self.cfg.left_marker}")
            yaw = utils.get_2d_pose(left_tf)[2]
            if self.cfg.front_dock:
                yaw = utils.flip_yaw(yaw)
            return self.rotate_with_odom(yaw - math.pi/2)

        right_tf = self.get_tf(self.cfg.right_marker)
        if right_tf is not None:
            rospy.logwarn(f"Rotate with right marker: {self.cfg.right_marker}")
            yaw = utils.get_2d_pose(right_tf)[2]
            if self.cfg.front_dock:
                yaw = utils.flip_yaw(yaw)
            return self.rotate_with_odom(yaw - math.pi/2)

        rospy.logerr("Not detecting two side markers, exit state")
        return False

    def do_predock(self) -> bool:
        """
        This is the predock phase, which will simply check if the the robot
        is located in the allowable zone, and rotate the robot base as such as
        the robot orientation is normal to the side markers.
        @return: success
        """
        self.set_state(DockState.PREDOCK, "start docking")

        # initial check if both markers are detected,
        # if only one single side marker detected, will purely depends to that
        # marker to rotate till facing the charger. then will check if both
        # markers exist again.
        if self.get_centre_of_side_markers() is None:
            if not self.do_single_side_marker_rotate():
                return False

        # start predock loop
        _pose_list = []
        rospy.loginfo("Both Markers are detected, running predock loop")
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            if self.is_pause:
                if not self.do_pause():
                    return False

            centre_tf = self.get_centre_of_side_markers()

            if centre_tf is None:
                rospy.logerr("Not detecting two side markers, exit state")
                return False

            if self.cfg.front_dock:
                centre_tf = utils.flip_base_frame(centre_tf)
            _, _, yaw = centre_tf
            print(f"current yaw diff: {yaw:.3f}")

            # Check if the robot is now normal to the charging station
            if abs(yaw) < self.cfg.stop_yaw_diff:
                rospy.logwarn(f"Done with yaw correction, tf: {centre_tf}")
                self.publish_cmd()

                # switch the reference frame to charging station.
                marker_to_base_link_tf = utils.get_2d_inverse(centre_tf)
                if len(_pose_list) < self.cfg.predock_tf_samples:
                    remainings = self.cfg.predock_tf_samples - len(_pose_list)
                    print(f"Getting {remainings} more samples for averaging")
                    _pose_list.append(marker_to_base_link_tf)
                    continue

                y_offset = utils.avg_2d_poses(_pose_list)[1]
                if self.cfg.front_dock:  # TODO
                    y_offset *= -1
                _pose_list = []

                # if robot y axis is way off, we will do parallel correction
                # after this, will repeat predock
                if abs(y_offset) > self.cfg.max_parallel_offset:
                    if not self.do_parallel_correction(y_offset):
                        return False

                    self.publish_cmd()
                    rospy.sleep(rospy.Duration(self.cfg.tf_expiry))
                    self.set_state(DockState.PREDOCK, "try predock again")
                    continue

                return True

            # publish rotate
            ang_vel = utils.bin_filter(yaw, self.cfg.min_angular_vel)
            self.publish_cmd(angular_vel=ang_vel)

            rospy.sleep(self.sleep_period)
        exit(0)

    def do_steer_dock(self) -> bool:
        """
        Utilize side markers to steer the robot closer to the charger.
        Transition to last_mile only after persistent side-marker loss
        and centre-marker validation.
        """
        self.set_state(DockState.STEER_DOCK)

        offset_from_charger = self.cfg.cam_offset + self.cfg.to_last_mile_dis
        transition_dis_with_tol = offset_from_charger + self.cfg.to_last_mile_tol
        _dir = 1 if self.cfg.front_dock else -1

        # side marker loss tolerance
        max_missing_count = 3
        side_marker_missing_count = 0
        
        # 历史数据记录
        from collections import deque
        HISTORY_SIZE = 20  # 记录最近20帧数据
        offset_history = deque(maxlen=HISTORY_SIZE)  # 存储offset历史
        distance_history = deque(maxlen=HISTORY_SIZE)  # 存储距离历史
        yaw_history = deque(maxlen=HISTORY_SIZE)  # 存储yaw历史
        control_history = deque(maxlen=HISTORY_SIZE)  # 存储控制历史
        
        # 控制状态
        recovery_mode = False  # 恢复模式标记
        recovery_start_time = 0.0
        MAX_RECOVERY_TIME = 2.0  # 最大恢复时间
        
        # 当前有效值
        current_offset = 0.0
        current_distance = 0.0
        current_yaw = 0.0
        is_offset_valid = False

        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            if self.is_pause:
                if not self.do_pause():
                    return False

            # get centre of two side markers
            centre_tf = self.get_centre_of_side_markers(offset_from_charger)

            # ------------------------------------------------------------
            # Case 1: side markers NOT detected
            # ------------------------------------------------------------
            if centre_tf is None:
                side_marker_missing_count += 1
                
                # 检查是否足够靠近充电桩
                if distance_history:
                    # 使用最近的有效距离判断
                    recent_distances = list(distance_history)[-3:]  # 取最近3个距离
                    if recent_distances:
                        avg_distance = sum(recent_distances) / len(recent_distances)
                        rospy.loginfo(f"侧边标记丢失，最近平均距离: {avg_distance:.3f}m, 过渡距离阈值: -0.05m")
                        
                        # 关键判断：如果已经足够靠近，直接检查中心标记
                        if avg_distance > -0.05:
                            rospy.logwarn(f"已足够靠近充电桩(距离: {avg_distance:.3f}m)，检查中心标记...")
                            
                            # persistent loss: check centre marker
                            centre_tf_mat = self.get_tf(self.cfg.centre_marker)
                            if centre_tf_mat is None:
                                rospy.logerr("Centre marker not detected")
                                return False

                            dis, _, _ = utils.get_2d_pose(centre_tf_mat)

                            if abs(dis) < -0.05:
                                rospy.logerr(
                                    f"Centre marker {dis:.3f} m away, too far for last_mile"
                                )
                                return False

                            rospy.logwarn(
                                f"Centre marker {dis:.3f} m away, transition to last_mile"
                            )
                            self.publish_cmd(0.0, 0.0)
                            return True
                
                rospy.logwarn(
                    f"Side markers lost "
                    f"({side_marker_missing_count}/{max_missing_count})"
                )

                # 标记丢失，进入恢复模式
                if not recovery_mode and is_offset_valid:
                    recovery_mode = True
                    recovery_start_time = rospy.get_time()
                    rospy.loginfo("进入恢复模式，尝试回溯历史数据")
                
                # 在恢复模式下，使用历史数据进行控制
                if recovery_mode:
                    current_time = rospy.get_time()
                    
                    # 检查恢复时间是否过长
                    if current_time - recovery_start_time > MAX_RECOVERY_TIME:
                        rospy.logwarn("恢复模式超时，检查中心标记")
                        
                        # 恢复模式超时，检查中心标记
                        centre_tf_mat = self.get_tf(self.cfg.centre_marker)
                        if centre_tf_mat is None:
                            rospy.logerr("恢复模式超时且中心标记未检测到，对接失败")
                            return False

                        dis, _, _ = utils.get_2d_pose(centre_tf_mat)

                        if abs(dis) > transition_dis_with_tol:
                            rospy.logerr(
                                f"Centre marker {dis:.3f} m away, too far for last_mile"
                            )
                            return False

                        rospy.logwarn(
                            f"Centre marker {dis:.3f} m away, transition to last_mile"
                        )
                        self.publish_cmd(0.0, 0.0)
                        return True
                    
                    # 使用历史数据恢复
                    if offset_history:
                        # 使用最近的有效offset
                        recent_offsets = list(offset_history)[-5:]  # 取最近5个
                        if recent_offsets:
                            # 计算历史offset的平均值
                            avg_offset = sum(recent_offsets) / len(recent_offsets)
                            
                            # 检查历史数据的一致性
                            offset_std = (max(recent_offsets) - min(recent_offsets)) / 2
                            if offset_std < 0.1:  # 历史数据相对一致
                                current_offset = avg_offset
                                rospy.loginfo(f"恢复模式: 使用历史offset: {current_offset:.3f}m,反向恢复")
                                
                                # 计算控制量
                                ang_vel = utils.sat_proportional_filter(
                                    current_offset, 
                                    factor=self.cfg.offset_to_angular_vel * 0.7
                                )
                                
                                # 缓慢前进
                                self.publish_cmd(
                                    linear_vel=_dir * self.cfg.max_linear_vel * 0.3,
                                    angular_vel=ang_vel
                                )
                            else:
                                rospy.logwarn(f"恢复模式: 历史数据不一致，停止等待")
                                self.publish_cmd(0.0, 0.0)
                        else:
                            rospy.logwarn("恢复模式: 无历史数据，停止")
                            self.publish_cmd(0.0, 0.0)
                    else:
                        rospy.logwarn("恢复模式: 无历史数据，停止")
                        self.publish_cmd(0.0, 0.0)
                    
                    # 恢复模式下继续循环，等待重新检测到标签
                    rospy.sleep(self.sleep_period)
                    continue
                
                # short loss: stop and wait
                if side_marker_missing_count <= max_missing_count:
                    rospy.sleep(self.sleep_period)
                    continue

                # persistent loss: check centre marker
                rospy.logwarn("Side markers persistently lost, checking centre marker")

                centre_tf_mat = self.get_tf(self.cfg.centre_marker)
                if centre_tf_mat is None:
                    rospy.logerr("Centre marker not detected")
                    return False

                dis, _, _ = utils.get_2d_pose(centre_tf_mat)

                if abs(dis) > transition_dis_with_tol:
                    rospy.logerr(
                        f"Centre marker {dis:.3f} m away, too far for last_mile"
                    )
                    return False

                rospy.logwarn(
                    f"Centre marker {dis:.3f} m away, transition to last_mile"
                )
                self.publish_cmd(0.0, 0.0)
                return True

            # ------------------------------------------------------------
            # Case 2: side markers detected
            # ------------------------------------------------------------
            side_marker_missing_count = 0
            
            # 退出恢复模式
            if recovery_mode:
                recovery_mode = False
                rospy.loginfo("退出恢复模式，重新检测到两个标签")

            if self.cfg.front_dock:
                centre_tf = utils.flip_base_frame(centre_tf)

            dis, offset, yaw = centre_tf

            print(
                f"DETECTED side markers "
                f"[d: {dis:.2f}, offset: {offset:.2f}, yaw: {yaw:.2f}]"
            )
            
            # ------------------------------------------------------------
            # 首先检查过渡条件
            # ------------------------------------------------------------
            if dis > 0:
                rospy.logwarn("Transition to last_mile state")
                self.publish_cmd(0.0, 0.0)
                return True
            
            # ------------------------------------------------------------
            # 数据验证和记录
            # ------------------------------------------------------------
            
            # 检查数据有效性
            is_current_data_valid = True
            
            # 1. 检查offset变化是否合理
            if offset_history:
                last_offset = offset_history[-1]
                offset_change = abs(offset - last_offset)
                if offset_change > 0.2:  # 单帧变化超过0.2m认为不合理
                    rospy.logwarn(f"Offset变化过大: {last_offset:.3f} -> {offset:.3f} (变化: {offset_change:.3f})")
                    is_current_data_valid = False
            
            # 2. 检查距离变化是否合理
            if distance_history:
                last_distance = distance_history[-1]
                distance_change = abs(dis - last_distance)
                if distance_change > 0.3:  # 单帧距离变化超过0.3m认为不合理
                    rospy.logwarn(f"距离变化过大: {last_distance:.3f} -> {dis:.3f} (变化: {distance_change:.3f})")
                    is_current_data_valid = False
            
            # 如果数据有效，更新当前值并记录历史
            if is_current_data_valid:
                current_offset = offset
                current_distance = dis
                current_yaw = yaw
                is_offset_valid = True
                
                # 记录历史
                offset_history.append(offset)
                distance_history.append(dis)
                yaw_history.append(yaw)
                
                rospy.logdebug(f"数据有效，已记录历史: offset={offset:.3f}, distance={dis:.3f}")
            else:
                rospy.logwarn("当前帧数据异常，使用历史数据")
                # 使用最近的有效历史数据
                if offset_history:
                    recent_offset = offset_history[-1]
                    current_offset = recent_offset
                    rospy.loginfo(f"使用历史offset: {recent_offset:.3f}")
            
            # ------------------------------------------------------------
            # 正常控制
            # ------------------------------------------------------------
            
            # 使用当前有效值进行控制
            if is_offset_valid:
                # 计算角速度
                ang_vel = utils.sat_proportional_filter(
                    -current_offset, 
                    factor=self.cfg.offset_to_angular_vel
                )
                
                # 线速度控制
                linear_vel = _dir * self.cfg.max_linear_vel
                
                # 根据历史数据稳定性调整速度
                if offset_history and len(offset_history) >= 5:
                    recent_offsets = list(offset_history)[-5:]
                    offset_range = max(recent_offsets) - min(recent_offsets)
                    
                    if offset_range < 0.05:  # 历史数据很稳定
                        linear_vel *= 0.8  # 正常速度
                    elif offset_range < 0.1:  # 相对稳定
                        linear_vel *= 0.5
                    else:  # 不稳定
                        linear_vel *= 0.3
                        rospy.loginfo(f"历史数据不稳定(range={offset_range:.3f})，降低速度")
                
                # 记录控制历史
                control_history.append((linear_vel, ang_vel))
                
                # 发布控制命令
                self.publish_cmd(linear_vel=linear_vel, angular_vel=ang_vel)
                
                rospy.logdebug(f"控制输出: v={linear_vel:.3f}, ω={ang_vel:.3f}")
            else:
                # 没有有效数据，停止
                rospy.logwarn("没有有效的offset数据，停止")
                self.publish_cmd(0.0, 0.0)
            
            rospy.sleep(self.sleep_period)

        exit(0)

    def do_last_mile(self) -> bool:
        """
        Final docking using the centre marker. 
        Added early charging check: if robot is already charging, return True immediately.
        """
        self.set_state(DockState.LAST_MILE)

        remaining_dis = self.cfg.to_last_mile_dis
        _dir = 1 if self.cfg.front_dock else -1

        # 检查充电状态，如果已经充电就直接完成
        if self.cfg.check_battery_status:
            from std_msgs.msg import Bool
            try:
                rospy.loginfo("检查是否已经在充电...")
                charge_msg = rospy.wait_for_message("/charging_state", Bool, timeout=2.0)
                if charge_msg.data:
                    rospy.loginfo("✓ 机器人已经在充电，直接结束最后一段")
                    return True
            except rospy.exceptions.ROSException:
                rospy.logwarn("获取充电状态超时，继续执行最后一段对接")

        # 添加标记丢失计数器
        marker_missing_count = 0
        max_missing_attempts = 3

        rospy.loginfo("开始最后一段对接")
        while not rospy.is_shutdown():
            if self.check_cancel():
                return False

            if self.is_pause:
                if not self.do_pause():
                    return False

            # 先检查充电状态（实时检测）
            if self.cfg.check_battery_status:
                try:
                    from std_msgs.msg import Bool
                    charge_msg = rospy.wait_for_message("/charging_state", Bool, timeout=0.5)
                    if charge_msg.data:
                        rospy.loginfo("✓ 充电已开始，直接完成最后一段")
                        self.publish_cmd(linear_vel=0.0, angular_vel=0.0)
                        return True
                except rospy.exceptions.ROSException:
                    pass  # 超时则继续执行最后一段

            centre_tf_mat = self.get_tf(self.cfg.centre_marker)

            # Final Dock based on odom if centre marker is lost
            if centre_tf_mat is None:
                marker_missing_count += 1
                rospy.logwarn(f"中心标记丢失检测 (第{marker_missing_count}次)")

                if remaining_dis < 0.1:
                    rospy.loginfo(f"剩余距离 {remaining_dis:.3f}m，使用里程计前进完成对接")
                    return self.move_with_odom( _dir * self.cfg.min_linear_vel)

                if marker_missing_count <= max_missing_attempts:
                    rospy.loginfo(f"等待标记重新出现 ({marker_missing_count}/{max_missing_attempts})...")
                    self.publish_cmd(linear_vel=0.0, angular_vel=0.0)
                    rospy.sleep(0.1)
                    continue
                else:
                    rospy.logwarn("中心标记持续丢失，使用里程计前进")
                    return self.move_with_odom(_dir * 0.5 * min(remaining_dis, self.cfg.max_last_mile_odom))
            else:
                marker_missing_count = 0

            centre_tf = utils.get_2d_pose(centre_tf_mat)
            if self.cfg.front_dock:
                centre_tf = utils.flip_base_frame(centre_tf)

            dis, offset, yaw = centre_tf
            yaw -= math.pi/2
            print(
                f"检测中心标记 "
                f"[d: {dis:.2f}, offset: {offset:.2f},yaw: {yaw:.2f}]"
            )
            remaining_dis = -dis - self.cfg.stop_distance - self.cfg.cam_offset

            if remaining_dis <= 0:
                rospy.loginfo("✓ 到达目标位置，最后一段完成")
                self.publish_cmd(linear_vel=0.0, angular_vel=0.0)
                return True

            # 自适应速度控制
            current_linear_vel = _dir * self.cfg.min_linear_vel

            ang_vel = utils.sat_proportional_filter(
                yaw - 5 * offset, abs_min=0.0, abs_max=self.cfg.min_angular_vel, factor=0.3
            )

            self.publish_cmd(linear_vel=current_linear_vel, angular_vel=ang_vel)
            rospy.sleep(self.sleep_period)

    def do_activate_charger(self) -> bool:
        self.set_state(DockState.ACTIVATE_CHARGER, "Check charging state")
        rospy.sleep(0.2)

        rospy.loginfo("等待充电状态确认...")

        if self.wait_for_charging_state(timeout=5.0):
            self.set_state(DockState.ACTIVATE_CHARGER, "Charging confirmed!")
            return True

        rospy.logerr("超时未检测到充电，认为对接失败")
        return False

    def wait_for_charging_state(self, timeout=30.0) -> bool:
        """
        等待充电状态变为真
        @param timeout: 超时时间(秒)
        @return: 是否成功检测到充电
        """
        from std_msgs.msg import Bool
        import rospy
        
        rospy.loginfo(f"等待充电开始，超时: {timeout}秒")
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < timeout:
            try:
                charge_msg = rospy.wait_for_message(
                    "/charging_state", Bool, timeout=1.0)
                
                if charge_msg.data:
                    rospy.loginfo("✓ 检测到充电开始")
                    return True
                    
                rospy.loginfo(f"等待充电... 已等待 {(rospy.Time.now() - start_time).to_sec():.1f}秒")
                
            except rospy.exceptions.ROSException:
                rospy.logwarn("获取充电状态超时，继续尝试...")
                continue
            
            rospy.sleep(1.0)
        
        rospy.logerr(f"充电状态检查超时 ({timeout}秒)")
        return False


##############################################################################
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Simple AutoDockServer')
    parser.add_argument('--server', dest='run_server',
                        action='store_true', help="run action server")
    parser.add_argument('--fake_clock', dest='fake_clock',
                        action='store_true',
                        help="Danger! use this to solve time sync issue")
    parser.add_argument('--rosparam', dest='load_rosparam',
                        action='store_true', help="load rosparam for configs")
    args, unknown = parser.parse_known_args()

    # Default Default Configuration
    config = DefaultAutoDockConfig()

    if args.run_server:
        node = AutoDockStateMachine(
            config,
            run_server=True,
            load_rosparam=args.load_rosparam,
            fake_clock=args.fake_clock)
        rospy.spin()
    else:
        node = AutoDockStateMachine(
            config,
            run_server=False,
            load_rosparam=args.load_rosparam,
            fake_clock=args.fake_clock)
        node.start()
