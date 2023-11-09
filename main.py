
import sys
import numpy as np
sys.path.append("/bluebot/data/Pyrocon")
from CRS_commander import Commander
from robCRSgripper import robCRSgripper
from robCRSdkt import robCRSdkt
from robCRSikt import robCRSikt
from robotCRS import robCRS97

robot = robCRS97()
cmd = Commander(robot)
cmd.open_comm("/dev/ttyUSB0", speed=19200)
cmd.init()

p_deg = np.array([0, 0, 0, 0, 0, 0])
#coords = robCRSdkt(robot, p_deg)
#angles_back = robCRSikt(robot, coords)
#print(angles_back, np.shape(angles_back), "coords: ", coords)
p_irc = cmd.anglestoirc(p_deg)
cmd.coordmv(p_irc)
"""p_irc = cmd.anglestoirc(p_deg)

cmd.coordmv(p_irc)

t, a = cmd.axis_get_pos()
print(a, cmd.irctoangles(a))
cmd.soft_home()
cmd.rcon.close()
"""
"""{
import argparse
import numpy as np

from CRS_commander import Commander
# from demo.im_proc import *
from graph import Graph
from interpolation import *
from robCRSgripper import robCRSgripper
from robotBosch import robotBosch
from robotCRS import robCRS93, robCRS97


def move_point_to_point(trajectory, commander):
    commander.move_to_pos(trajectory[0])
    for i in range(1, len(trajectory)):
        commander.move_to_pos(trajectory[i])


def move_spline(trajectory, commander, spline, order):
    spline_params = []

    if spline == 'poly':
        order = 3
        spline_params = poly.interpolate(trajectory)
    if spline == 'b-spline':
        spline_params = b_spline.interpolate(trajectory, order=order)
    if spline == 'p-spline':
        num_segments = int(len(trajectory) / 3)
        poly_deg = order
        penalty_order = 2
        lambda_ = 0.1
        spline_params = p_spline.interpolate(trajectory, num_segments, poly_deg, penalty_order, lambda_)

    commander.move_to_pos(trajectory[0])
    commander.wait_ready(sync=True)
    for i in range(len(spline_params)):
        commander.splinemv(spline_params[i], order=order)
    commander.wait_ready(sync=True)


def circle_trajectory(commander, x=500, y0=250, z0=500, r=50, step=10):
    
    Circle trajectory for CRS robot.
    :param commander: Robot commander
    :param x: X coordinate of circle_trajectory plane
    :param y0: Y coordinate of starting point
    :param z0: Z coordinate of starting point
    :param r: radius of circle_trajectory
    :param step: angle of trajectory discretisation in degrees
    :return: points of trajectory
    
    pos = [x, y0 + r, z0, 0, 0, 0]
    sol = [commander.find_closest_ikt(pos)]
    rng = int(360 / step)
    for i in range(rng + 1):
        y = y0 + r * np.cos((i * step) / 180.0 * np.pi)
        z = z0 + r * np.sin((i * step) / 180.0 * np.pi)
        pos = [x, y, z, 0, 0, 0]
        prev_a = commander.find_closest_ikt(pos, sol[-1])
        sol.append(prev_a)

    return np.array(sol)


def line_trajectory(commander, x0, x1, step=5):
    
    Line trajectory for CRS robot.
    :param commander: Robot commander
    :param x0: starting point of trajectory
    :param x1: end point of trajectory
    :param step: step of trajectory discretisation in mm
    :return: points of trajectory
    
    rng = int(np.linalg.norm(np.array(x0) - np.array(x1)) / step)
    normal = (np.array(x1) - np.array(x0)) / np.linalg.norm(np.array(x0) - np.array(x1))
    x = x0
    sol = [commander.find_closest_ikt(x)]
    for i in range(rng):
        x = x + normal * step
        prev_x = commander.find_closest_ikt(x, sol[-1])
        sol.append(prev_x)
    return np.array(sol)


if __name__ == '__main__':

    help_msg = 'SYNOPSIS: CRS_commander.py [-l /dev/ttyXXX]'

    parser = argparse.ArgumentParser(description='CRS robot commander')
    parser.add_argument('-s', '--skip-setup', dest='skip_setup', action='store_true',
                        default=False, help='skip hard-home initialization of robot')
    parser.add_argument('-d', '--tty-device', dest='tty_dev', type=str,
                        default='/dev/ttyUSB0', help='tty line_trajectory/device to robot')
    parser.add_argument('-a', '--action', dest='action', type=str,
                        default='home', help='action to run, possible actions:\n \
                                            {home - homing of the robot,\n \
                                             graph - draw graph of interpolated circle trajectory,\n \
                                             circle_spline - move along interpolated circle trajectory,\n \
                                             circle_ptp - move along circle trajectory point to point,\n \
                                             grip - close gripper,\n \
                                             purge - purge errors on motors}')
    parser.add_argument('-r', '--robot', dest='robot', type=str,
                        help='type of robot\n{CRS97, CRS93, Bosch}', required=True)
    parser.add_argument('-m', '--max-speed', dest='max_speed', type=int,
                        default=None, help='maximal motion speed')
    parser.add_argument('-t', '--reg-type', dest='reg_type', type=int, default=None, help='controller type selection')
    parser.add_argument('-sp', '--spline', dest='spline', type=str, default='poly',
                        help='type of spline to use for interpolation\n{poly, b-spline, p-spline}')
    parser.add_argument('-o', '--order', dest='order', type=int, default=2, help='order of splines')

    args = parser.parse_args()

    tty_dev = args.tty_dev
    skip_setup = args.skip_setup
    max_speed = args.max_speed
    reg_type = args.reg_type
    action = args.action
    spline = args.spline
    order = args.order
    rob = args.robot

    robot = None
    if rob == 'CRS97':
        robot = robCRS97()
    if rob == 'CRS93':
        robot = robCRS93()
    if rob == 'Bosch':
        robot = robotBosch()

    commander = Commander(robot)  # initialize commander
    commander.open_comm(tty_dev, speed=19200)  # connect to control unit

    if not skip_setup or action == 'home':
        commander.init(reg_type=reg_type, max_speed=max_speed, hard_home=True)

    if rob in ['CRS97', 'CRS93']:

        if action in ['graph', 'circle_ptp', 'circle_spline']:
            sol = circle_trajectory(commander)

            if action == 'graph':
                e = Graph(sol)
                e.show_gui()

            if action == 'circle_spline':
                move_spline(sol, commander, spline, order)

            if action == 'circle_ptp':
                move_point_to_point(sol, commander)

        if action == 'grip':
            robCRSgripper(commander, 0.9)
            commander.wait_ready()

        if action == 'purge':
            commander.send_cmd("PURGE:\n")}"""
