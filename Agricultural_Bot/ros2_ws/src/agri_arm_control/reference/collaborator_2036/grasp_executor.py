#!/usr/bin/env python3
"""抓取动作链：预抓取 -> 接近 -> 夹紧 -> 撤离 -> 入篮 -> 释放 -> 回零"""
import time
from cr5_controller import compute_grasp_orientation


class GraspExecutor:
    def __init__(self, node, controller, params):
        self.node = node
        self.arm = controller
        self.params = params

    def _log(self, msg):
        self.node.get_logger().info(msg)

    def execute(self, tomato_pos, normal):
        grasp = self.params["grasp"]
        poses = self.params["poses"]

        self._log(f"番茄位置: {tomato_pos}")
        self._log(f"法向量: {normal}")

        quat = self.arm.try_grasp_orientations(tomato_pos, normal)
        if quat is None:
            return False, "所有姿态 IK 失败"
        self._log(f"选定姿态: {quat}")

        pre_pos = [
            tomato_pos[i] + normal[i] * grasp["pre_offset"]
            for i in range(3)
        ]
        self._log(f"[1/7] 预抓取: {pre_pos}")
        if not self.arm.move_to_pose(pre_pos, quat):
            return False, "预抓取失败"

        self._log(f"[2/7] 接近: {tomato_pos}")
        if not self.arm.move_to_pose(tomato_pos, quat):
            return False, "接近失败"

        self._log(f"[3/7] 夹爪闭合: {grasp['gripper_close']}")
        self.arm.set_gripper(grasp["gripper_close"])
        time.sleep(1.0)

        retreat_pos = [
            tomato_pos[i] + normal[i] * grasp["retreat_offset"]
            for i in range(3)
        ]
        self._log(f"[4/7] 撤离: {retreat_pos}")
        if not self.arm.move_to_pose(retreat_pos, quat):
            return False, "撤离失败"

        basket = poses["basket_drop"]
        basket_pos = basket["position"]
        basket_quat = basket["quaternion"]
        above_basket = [basket_pos[0], basket_pos[1], basket_pos[2] + 0.15]

        self._log(f"[5/7] 篮子上方: {above_basket}")
        if not self.arm.move_to_pose(above_basket, basket_quat):
            return False, "移动到篮子上方失败"

        self._log(f"[5/7] 下降到篮子口: {basket_pos}")
        if not self.arm.move_to_pose(basket_pos, basket_quat):
            return False, "下降到篮子口失败"

        self._log(f"[6/7] 夹爪释放: {grasp['gripper_open']}")
        self.arm.set_gripper(grasp["gripper_open"])
        time.sleep(1.0)

        self._log(f"[6/7] 抬起")
        if not self.arm.move_to_pose(above_basket, basket_quat):
            return False, "抬起失败"

        stow = poses.get("arm_stow", {}).get("joints", [0, 0, 0, 0, 0, 0])
        self._log(f"[7/7] 回零位: {stow}")
        if not self.arm.move_to_joints(stow):
            self._log("  回零失败（不影响抓取结果）")

        return True, ""
