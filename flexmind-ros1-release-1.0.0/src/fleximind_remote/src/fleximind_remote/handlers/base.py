from abc import abstractmethod
from typing import Optional, Tuple

import numpy as np

from fleximind_remote import ArmType


class BaseRemoteHandler:
    def __init__(self):
        self._is_connected: bool = False
        self._arm_type: ArmType = ArmType.BIMANUAL

    @property
    def arm_type(self):
        return self._arm_type

    @arm_type.setter
    def arm_type(self, _arm_type: ArmType):
        if type(_arm_type) == type(ArmType):
            self._arm_type = _arm_type

    @property
    def is_connected(self):
        return self._is_connected

    @abstractmethod
    def connect(self):
        raise NotImplementedError

    @abstractmethod
    def disconnect(self):
        raise NotImplementedError

    @abstractmethod
    def calibrate(self):
        raise NotImplementedError

    @abstractmethod
    def get_joint_angles(self) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        raise NotImplementedError
