#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import sys
import os

# 添加路径
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from fleximind_robot.huayan_robot import CPScontrol
from fleximind_robot.CPS import RbtFSM

def debug_blackout():
    """专门调试断电功能的脚本"""
    
    # 初始化节点（如果还没初始化）
    try:
        rospy.init_node('debug_power_off', anonymous=True)
    except:
        pass
    
    print("=== 开始断电调试 ===")
    
    # 读取配置参数（请根据您的实际配置修改）
    config = {
        'boxid_left': 0,
        'boxid_right': 1,
        'ip_left': "192.168.1.20",  # 请替换为实际IP
        'ip_right': "192.168.1.30", # 请替换为实际IP
        'port': 10003
    }
    
    # 创建机械臂实例
    try:
        print("创建左臂控制器...")
        arm_left = CPScontrol("left_robot", config['boxid_left'], config['ip_left'], config['port'])
        
        print("创建右臂控制器...")
        arm_right = CPScontrol("right_robot", config['boxid_right'], config['ip_right'], config['port'])
        
        # 测试连接
        print("测试连接...")
        left_connected = arm_left.connect()
        right_connected = arm_right.connect()
        
        print(f"连接状态 - 左臂: {left_connected}, 右臂: {right_connected}")
        
        if not left_connected or not right_connected:
            print("连接失败，退出调试")
            return
        
        # 读取当前状态
        print("\n=== 读取当前状态 ===")
        
        # 读取FSM状态
        left_fsm = []
        right_fsm = []
        arm_left.sdk.HRIF_ReadCurFSM(arm_left.box_id, 0, left_fsm)
        arm_right.sdk.HRIF_ReadCurFSM(arm_right.box_id, 0, right_fsm)
        
        print(f"左臂FSM状态: {left_fsm}")
        print(f"右臂FSM状态: {right_fsm}")
        
        # 读取机器人状态
        left_state = arm_left.read_robot_state()
        right_state = arm_right.read_robot_state()
        
        print(f"左臂详细状态: {left_state}")
        print(f"右臂详细状态: {right_state}")
        
        # 测试断电功能
        print("\n=== 测试断电功能 ===")
        
        # 方法1: 直接断电
        print("方法1: 直接调用 blackout()")
        left_result1 = arm_left.blackout()
        right_result1 = arm_right.blackout()
        
        print(f"直接断电结果 - 左臂: {left_result1}, 右臂: {right_result1}")
        
        # 等待并检查状态变化
        time.sleep(3)
        
        # 重新读取状态
        arm_left.sdk.HRIF_ReadCurFSM(arm_left.box_id, 0, left_fsm)
        arm_right.sdk.HRIF_ReadCurFSM(arm_right.box_id, 0, right_fsm)
        
        print(f"断电后FSM状态 - 左臂: {left_fsm}, 右臂: {right_fsm}")
        
        # 如果方法1失败，尝试方法2
        if not left_result1 or not right_result1:
            print("\n方法1失败，尝试方法2: 先禁用再断电")
            
            # 重新连接（如果需要）
            arm_left.connect()
            arm_right.connect()
            time.sleep(2)
            
            # 先禁用
            arm_left.disable()
            arm_right.disable()
            time.sleep(1)
            
            # 再断电
            left_result2 = arm_left.blackout()
            right_result2 = arm_right.blackout()
            
            print(f"先禁用再断电结果 - 左臂: {left_result2}, 右臂: {right_result2}")
            
        # 如果还失败，尝试方法3
        if (not left_result1 and not left_result2) or (not right_result1 and not right_result2):
            print("\n方法2失败，尝试方法3: 使用底层API")
            
            # 直接调用底层API并获取错误码
            left_error = []
            right_error = []
            
            left_ret = arm_left.sdk.HRIF_BlackOut(arm_left.box_id)
            right_ret = arm_right.sdk.HRIF_BlackOut(arm_right.box_id)
            
            print(f"底层API返回 - 左臂错误码: {left_ret}, 右臂错误码: {right_ret}")
            
            # 获取错误描述
            if left_ret != 0:
                left_desc = []
                arm_left.sdk.HRIF_GetErrorCodeStr(arm_left.box_id, left_ret, left_desc)
                print(f"左臂错误描述: {left_desc}")
                
            if right_ret != 0:
                right_desc = []
                arm_right.sdk.HRIF_GetErrorCodeStr(arm_right.box_id, right_ret, right_desc)
                print(f"右臂错误描述: {right_desc}")
        
        print("\n=== 调试完成 ===")
        
    except Exception as e:
        print(f"调试过程中发生异常: {str(e)}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    debug_blackout()