from dataclasses import dataclass
from enum import Enum


class BridgeDirection(Enum):
    BIDIRECTIONAL = 0
    IGN_TO_ROS = 1
    ROS_TO_IGN = 2


DIRECTION_SYMS = {
    BridgeDirection.BIDIRECTIONAL: '@',
    BridgeDirection.IGN_TO_ROS: '[',
    BridgeDirection.ROS_TO_IGN: ']',
}


@dataclass
class Bridge:
    ign_topic: str
    ros_topic: str
    ign_type: str
    ros_type: str
    direction: BridgeDirection

    def argument(self):
        out = f'{self.ign_topic}@{self.ros_type}{DIRECTION_SYMS[self.direction]}{self.ign_type}'
        return out

    def remapping(self):
        return (self.ign_topic, self.ros_topic)
