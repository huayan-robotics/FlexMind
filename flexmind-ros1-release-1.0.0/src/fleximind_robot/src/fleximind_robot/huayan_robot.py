import logging

import numpy as np
from fleximind_robot.CPS import CPSClient, RbtFSM
import rospy

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] [%(threadName)s] %(message)s",
    handlers=[logging.StreamHandler()],
)


class CPScontrol:
    def __init__(self, name, box_id, ip, port):
        self.logger = logging.getLogger("huayan")
        self.box_id = box_id
        self.ip = ip
        self.port = port
        self.name = name

        try:
            self.sdk = CPSClient()
            self.logger.info(f"{self.name} CPS SDK initialized")
        except ImportError as e:
            self.logger.error(f"{self.name} CPS SDK not found: {str(e)}")
            raise RuntimeError(f"{self.name} Missing CPS SDK dependency")

    def connect(self) -> bool:
        """连接"""
        try:
            self.logger.info(
                f"{self.name} Connecting... box_id:{self.box_id} {self.ip}:{self.port}"
            )
            nRet = self.sdk.HRIF_Connect(self.box_id, self.ip, self.port)
            if nRet != 0:
                self.logger.error(f"{self.name} Connection failed: {nRet}")
                return False
            self.logger.info(f"{self.name} Connection successful")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} Connection error: {str(e)}")
            return False

    def disconnect(self) -> bool:
        """断开连接"""
        try:
            nRet = self.sdk.HRIF_DisConnect(self.box_id)
            if nRet != 0:
                self.logger.error(f"{self.name} DisConnect failed (Error code: {nRet})")
                return False
            self.logger.info(f"{self.name} Disconnection successful")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} Disconnection error: {str(e)}")
            return False

    def isconnect(self) -> bool:
        """连接状态"""
        try:
            nRet = self.sdk.HRIF_IsConnected(self.box_id)
            if nRet != 0:
                self.logger.error(
                    f"{self.name} IsConnected failed (Error code: {nRet})"
                )
                return False
            self.logger.info(f"{self.name} IsConnected successful")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} IsConnected error: {str(e)}")
            return False

    def read_io(self, bit=0) -> list:
        """持续监控电箱状态"""
        result = []  # 必须初始化为空列表

        # 调用接口读取状态
        nRet = self.sdk.HRIF_ReadBoxCI(self.box_id, bit, result)

        if nRet != 0:
            self.logger.error(
                f"{self.name} 读取电箱{self.box_id}位{bit}失败，错误码: {nRet}"
            )
            result = []
        return result

    def reset(self, robot_id=0) -> bool:
        """机器人复位"""
        try:
            nRet = self.sdk.HRIF_GrpReset(self.box_id, robot_id)
            if nRet != 0:
                self.logger.error(f"{self.name} GrpReset failed (Error code: {nRet})")
                return False
            self.logger.info(f"{self.name} Group reseted successful")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} Group reseted error: {str(e)}")
            return False

    def enable(self, robot_id=0) -> bool:
        """机器人使能"""
        try:
            nRet = self.sdk.HRIF_GrpEnable(self.box_id, robot_id)
            if nRet != 0:
                self.logger.error(f"{self.name} GrpEnable failed (Error code: {nRet})")
                return False
            self.logger.info(f"{self.name} Group enabled successfully")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} Group enabled error: {str(e)}")
            return False

    def electrify(self) -> bool:
        """机器人上电"""
        try:
            nRet = self.sdk.HRIF_Electrify(self.box_id)
            if nRet != 0:
                self.logger.error(f"{self.name} Electrify failed (Error code: {nRet})")
                return False
            self.logger.info(f"{self.name} Electrified successfully")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} Electrified error: {str(e)}")
            return False

    def blackout(self) -> bool:
        """机器人断电"""
        try:
            nRet = self.sdk.HRIF_BlackOut(self.box_id)
            if nRet != 0:
                self.logger.error(f"{self.name} BlackOut failed (Error code: {nRet})")
                return False
            self.logger.info(f"{self.name} Blackout successful")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} Blackout error: {str(e)}")
            return False

    def stop(self, robot_id=0) -> bool:
        """机器人停止"""
        try:
            nRet = self.sdk.HRIF_GrpStop(self.box_id, robot_id)
            if nRet != 0:
                self.logger.error(f"{self.name} GrpStop failed (Error code: {nRet})")
                return False
            self.logger.info(f"{self.name} Group stopped successfully")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} Group stopped error: {str(e)}")
            return False

    def connect_controller(self) -> bool:
        """连接控制器,仅在机器人断电状态下使用"""
        try:
            nRet = self.sdk.HRIF_Connect2Controller(self.box_id)
            if nRet != 0:
                self.logger.error(
                    f"{self.name} Connect2Controller failed (Error code: {nRet})"
                )
                return False
            self.logger.info(f"{self.name} Connect Controller successful")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} Connect Controller error: {str(e)}")
            return False

    def disable(self, robot_id=0) -> bool:
        """机器人去使能"""
        try:
            nRet = self.sdk.HRIF_GrpDisable(self.box_id, robot_id)
            if nRet != 0:
                self.logger.error(f"{self.name} GrpDisable failed (Error code: {nRet})")
                return False
            self.logger.info(f"{self.name} Group disabled successfully")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} Group disabled error: {str(e)}")
            return False

    def read_robot_state(self, robot_id=0):
        """
        调用 HRIF_ReadRobotState 接口读取机器人状态并详细打印结果

        参数:
            box_id: 电箱ID (0~5, 默认0)
            robot_id: 机器人ID (0~5, 默认0)
        """
        # 定义返回值空列表
        result = []

        # 调用接口
        try:
            nRet = self.sdk.HRIF_ReadRobotState(self.box_id, robot_id, result)
        except Exception as e:
            self.logger.error(f"{self.name} 调用接口失败: {str(e)}")
            return None

        if nRet != 0:
            self.logger.error(f"{self.name} 警告: 接口调用失败，错误码: {nRet}")
            return None

        if len(result) < 13:
            self.logger.error(
                f"{self.name} 错误: 返回结果长度不足,期望13,实际得到{len(result)},得到的结果是 {result}"
            )
            return None
        return result

    def longJogJ(self, boxID, rbtID, points):
        """
        长距离移动
        """
        try:
            nRet = self.sdk.HRIF_MoveJTo(boxID, rbtID, points)
            if nRet != 0:
                self.logger.error(f"{self.name} LongJogJ failed (Error code: {nRet})")
                return False
            self.logger.info(f"{self.name} LongJogJ successful")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} LongJogJ error: {str(e)}")
            return False

    def longMoveEvent(self, boxID, rbtID):
        """
        长点动继续指令，当开始长点动之后，要按 500 毫秒或更短时间为时间周期发送一次该指令，否则长点动会停止
        """
        try:
            nRet = self.sdk.HRIF_LongMoveEvent(boxID, rbtID)
            if nRet != 0:
                self.logger.error(
                    f"{self.name} LongMoveEvent failed (Error code: {nRet})"
                )
                rospy.logerr(f"{self.name} LongMoveEvent failed (Error code: {nRet})")
                return False
            self.logger.info(f"{self.name} LongMoveEvent successful")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} LongMoveEvent error: {str(e)}")
            rospy.logerr(f"{self.name} LongMoveEvent error: {str(e)}")
            return False

    def isMotionDone(self, boxID, rbtID, result):
        """
        检查机器人运动是否完成
        result[0]: 1:运动完成,0：运动未完成
        """
        try:
            nRet = self.sdk.HRIF_IsMotionDone(boxID, rbtID, result)
            if nRet != 0:
                self.logger.error(
                    f"{self.name} IsMotionDone failed (Error code: {nRet})"
                )
                return False
            self.logger.info(f"{self.name} IsMotionDone successful result: {result[0]}")
            return True
        except Exception as e:
            self.logger.error(f"{self.name} IsMotionDone error: {str(e)}")
            return False

    def isArrivedJoints(self, boxID, rbtID, target_joints, result, tolerance=0.5):
        """
        检查机器人运动是否到点
        """
        # 定义返回值空列表
        result = []

        # 调用接口
        try:
            nRet = self.sdk.HRIF_ReadActPos(boxID, rbtID, result)
        except Exception as e:
            self.logger.error(f"{self.name} 调用接口失败: {str(e)}")
            return False

        if nRet != 0:
            self.logger.error(f"{self.name} 警告: 接口调用失败，错误码: {nRet}")
            return False

        if len(result) < 24:
            self.logger.error(
                f"{self.name} 错误: 返回结果长度不足,期望13,实际得到{len(result)},得到的结果是 {result}"
            )
            return False

        current_joints = [float(joint) for joint in result[5:11]]
        diff1 = np.abs(np.array(target_joints) - np.array(current_joints))
        return np.all(diff1 <= tolerance)


    def start_servo(self, boxID, rbtID, servoTime, lookaheadTime):
        """
        检查机器人运动是否完成
        result[0]: 1:运动完成,0：运动未完成
        """
        try:
            nRet = self.sdk.HRIF_StartServo(boxID, rbtID, servoTime, lookaheadTime)
            if nRet != 0:
                rospy.logerr(
                    f"{self.ip} start_servo failed (Error code: {nRet})"
                )
                return False
            rospy.loginfo(f"{self.ip} start_servo Successfully")
            return True
        except Exception as e:
            rospy.loginfo(f"{self.ip} start_servo error: {str(e)}")
            return False
        
    def push_servo(self, boxID, rbtID, dACS):
        """
        检查机器人运动是否完成
        result[0]: 1:运动完成,0：运动未完成
        """
        try:
            nRet = self.sdk.HRIF_PushServoJ(boxID, rbtID, dACS)
            if nRet != 0:
                rospy.logerr(
                    f"{self.ip} push_servo failed (Error code: {nRet})"
                )
                return False
            # rospy.loginfo(f"{self.ip} push_servo Successfully")
            return True
        except Exception as e:
            rospy.loginfo(f"{self.ip} push_servo error: {str(e)}")
            return False
        
    def setspeed(self, robot_id = 0, vel = 0.15):
        """
        检查机器人运动是否完成
        result[0]: 1:运动完成,0：运动未完成
        """
        try:
            nRet = self.sdk.HRIF_SetOverride(self.box_id, robot_id, vel)
            if nRet != 0:
                rospy.logerr(
                    f"{self.ip} set speed failed (Error code: {nRet})"
                )
                return False
            return True
        except Exception as e:
            rospy.loginfo(f"{self.ip} set speed error: {str(e)}")
            return False

    def move_j(self, boxID=0, rbtID=0, dX_dRz=None, dJ1_dJ6=None, sTcpName="TCP", 
               sUcsName="Base", dVelocity=50.0, dAcc=50.0, dRadius=50.0, 
               nIsUseJoint=1, nIsSeek=0, nIOBit=0, nIOState=0, strCmdID="0"):
        """
        关节运动接口 - HRIF_MoveJ
        
        参数:
            boxID: 电箱ID (0~5, 默认0)
            rbtID: 机器人ID (0~5, 默认0)
            dX_dRz: 空间目标位置 [X, Y, Z, Rx, Ry, Rz]，单位mm/°
            dJ1_dJ6: 关节目标位置 [关节1~关节6]，单位°
            sTcpName: 工具坐标名称 (默认"TCP")
            sUcsName: 用户坐标名称 (默认"Base")
            dVelocity: 运动最大速度，单位°/s (默认50.0)
            dAcc: 运动最大加速度，单位°/s² (默认50.0)
            dRadius: 过渡半径，单位mm (默认50.0)
            nIsUseJoint: 是否使用关节坐标 0:不使用 1:使用 (默认1)
            nIsSeek: 是否检测DI停止 0:不检测 1:检测 (默认0)
            nIOBit: 检测的DI索引 (0~7, 默认0)
            nIOState: 检测的DI状态 (0/1, 默认0)
            strCmdID: 命令ID (默认"0")
        
        返回:
            bool: 成功返回True，失败返回False
        """
        try:
            # 参数验证
            if boxID < 0 or boxID > 5:
                self.logger.error(f"{self.name} boxID参数超出有效范围(0~5): {boxID}")
                return False
                
            if rbtID < 0 or rbtID > 5:
                self.logger.error(f"{self.name} rbtID参数超出有效范围(0~5): {rbtID}")
                return False
                
            if nIOBit < 0 or nIOBit > 7:
                self.logger.error(f"{self.name} nIOBit参数超出有效范围(0~7): {nIOBit}")
                return False
                
            if nIOState not in [0, 1]:
                self.logger.error(f"{self.name} nIOState参数必须为0或1: {nIOState}")
                return False
                
            if nIsUseJoint not in [0, 1]:
                self.logger.error(f"{self.name} nIsUseJoint参数必须为0或1: {nIsUseJoint}")
                return False
                
            if nIsSeek not in [0, 1]:
                self.logger.error(f"{self.name} nIsSeek参数必须为0或1: {nIsSeek}")
                return False
            
            # 设置默认值
            if dX_dRz is None:
                dX_dRz = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
                
            if dJ1_dJ6 is None:
                dJ1_dJ6 = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
            
            # 调用HRIF_MoveJ接口
            nRet = self.sdk.HRIF_MoveJ(boxID, rbtID, dX_dRz, dJ1_dJ6, sTcpName, 
                                     sUcsName, dVelocity, dAcc, dRadius, nIsUseJoint, 
                                     nIsSeek, nIOBit, nIOState, strCmdID)
            
            if nRet != 0:
                self.logger.error(f"{self.name} MoveJ failed (Error code: {nRet})")
                return False
                
            self.logger.info(f"{self.name} MoveJ successful")
            return True
            
        except Exception as e:
            self.logger.error(f"{self.name} MoveJ error: {str(e)}")
            return False
    
    def set_override(self, boxID=0, rbtID=0, dOverride=1) -> bool:
        """
        设置速度比 - HRIF_SetOverride
        
        参数:
            boxID: 电箱ID (0~5, 默认0)
            rbtID: 机器人ID (0~5, 默认0)
            dOverride: 速度比 (0.01~1, 默认0.5)
        
        返回:
            bool: 成功返回True，失败返回False
        """
        try:
            # 参数验证
            if boxID < 0 or boxID > 5:
                self.logger.error(f"{self.name} boxID参数超出有效范围(0~5): {boxID}")
                return False
                
            if rbtID < 0 or rbtID > 5:
                self.logger.error(f"{self.name} rbtID参数超出有效范围(0~5): {rbtID}")
                return False
                
            if dOverride < 0.01 or dOverride > 1:
                self.logger.error(f"{self.name} dOverride参数超出有效范围(0.01~1): {dOverride}")
                return False
            
            # 调用HRIF_SetOverride接口
            nRet = self.sdk.HRIF_SetOverride(boxID, rbtID, dOverride)
            
            if nRet != 0:
                self.logger.error(f"{self.name} SetOverride failed (Error code: {nRet})")
                return False
                
            self.logger.info(f"{self.name} SetOverride successful, speed ratio: {dOverride}")
            return True
            
        except Exception as e:
            self.logger.error(f"{self.name} SetOverride error: {str(e)}")
            return False