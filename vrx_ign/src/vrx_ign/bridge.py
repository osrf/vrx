from dataclasses import dataclass
from enum import Enum


class BridgeDirection(Enum):
    BIDIRECTIONAL = 0
    GZ_TO_ROS = 1
    ROS_TO_GZ = 2


DIRECTION_SYMS = {
    BridgeDirection.BIDIRECTIONAL: '@',
    BridgeDirection.GZ_TO_ROS: '[',
    BridgeDirection.ROS_TO_GZ: ']',
}


@dataclass
class Bridge:
    gz_topic: str
    ros_topic: str
    gz_type: str
    ros_type: str
    direction: BridgeDirection

    def argument(self):
        out = f'{self.gz_topic}@{self.ros_type}{DIRECTION_SYMS[self.direction]}{self.gz_type}'
        return out

    def remapping(self):
        return (self.gz_topic, self.ros_topic)
