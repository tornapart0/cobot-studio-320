# Start near zero; preview and check the physical workspace first.
robot.move([0, 0, 0, 0, 0, 0], 5)
robot.move([5, 0, 0, 0, 0, 0], 5)
robot.wait(1)
robot.move([0, 0, 0, 0, 0, 0], 5)
