#!/usr/bin/env python3
from ur5_robot.robot_control import UR_robot

def main():
    ur_robot = UR_robot()
    ur_robot.run()

if __name__ == "__main__":
    main()