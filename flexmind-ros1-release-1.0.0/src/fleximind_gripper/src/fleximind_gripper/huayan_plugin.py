import rospy
from fleximind_robot.CPS import CPSClient


class HuayanPluginGripper:
    def __init__(self, box_id=0, ip="10.20.200.85", port=10003):
        self.name = "hr_gri_plugins"
        self.boxID = box_id

        try:
            self.sdk = CPSClient()
            self.sdk.HRIF_Connect(self.boxID, ip, port)
            rospy.loginfo(f"CPS SDK initialized for gripper {box_id} at {ip}:{port}")
        except ImportError as e:
            rospy.logerr(f"CPS SDK not found: {str(e)}")
            raise RuntimeError("Missing CPS SDK dependency")

    def _call_command(self, cmd, param, result) -> bool:
        try:
            nRet = self.sdk.HRIF_HRApp(self.boxID, self.name, cmd, param, result)
            if nRet != 0:
                rospy.logerr_throttle(
                    3,
                    f"HRIF_HRApp failed (boxID: {self.boxID}, command: {cmd}, params: {param}, Error code: {nRet})",
                )
                return False
            return True
        except Exception as e:
            rospy.logerr(
                f"HRIF_HRApp error [boxID: {self.boxID}, command: {cmd}, params: {param}, error: {str(e)}"
            )
            return False
