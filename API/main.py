import numpy as np
import time

from apto_sdk import *


def main():
    apto_arm = AptoSDK('/dev/ttyACM0', 1000000)
    apto_arm.connect()

    while True:
        apto_arm.set_pose(np.zeros(len(apto_arm.motor_ids)))
        print("Position: " + str(apto_arm.read_pos()))
        time.sleep(0.05)

if __name__ == "__main__":
    main()