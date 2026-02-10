from typing import Any, Dict, Protocol

import numpy as np


class Agent(Protocol):
    def act(self, obs: Dict[str, Any]) -> np.ndarray:
        """Returns an action given an observation.

        Args:
            obs: observation from the environment.

        Returns:
            action: action to take on the environment.
        """
        raise NotImplementedError

    def reset_offset(self):
        raise NotImplementedError

    def reset_gripper(self):
        raise NotImplementedError


class DummyAgent(Agent):
    def __init__(self, num_dofs: int):
        self.num_dofs = num_dofs

    def act(self) -> np.ndarray:
        return np.zeros(self.num_dofs)
        # return np.full(shape=self.num_dofs, fill_value=-7)


class BimanualAgent(Agent):
    def __init__(self, agent_left: Agent, agent_right: Agent):
        self.agent_left = agent_left
        self.agent_right = agent_right

    def act(self) -> np.ndarray:
        try:
            act_l = self.agent_left.act()
            act_r = self.agent_right.act()
            if (act_l is not None) and (act_r is not None):
                return np.concatenate([self.agent_left.act(), self.agent_right.act()])
            else:
                return None
        except Exception as e:
            print(f"双臂手柄读取角度出现错误{e}")
            return None

    def reset_offset(self):
        try:
            offset_l = self.agent_left.reset_offset()
            offset_r = self.agent_right.reset_offset()
            return (offset_l, offset_r)
        except Exception as e:
            print(f"双臂手柄矫正角度出现错误{e}")
            return None

    def reset_gripper(self):
        try:
            offset_l = self.agent_left.reset_gripper()
            offset_r = self.agent_right.reset_gripper()
            return (offset_l, offset_r)
        except Exception as e:
            print(f"双臂手柄矫正角度出现错误{e}")
            return None
