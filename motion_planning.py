
import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv
import re

from planning_utils import a_star, heuristic, create_grid, \
    bres_prune, plot_route
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, waypoints=[]):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = waypoints
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION,
                               self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] -
                              self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1],
                          self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        t0 = time.time()
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = args.altitude
        SAFETY_DISTANCE = args.safety

        self.target_position[2] = TARGET_ALTITUDE

        # work around for solving the time issue with the drone class
        # AttributeError: 'int' object has no attribute 'time'

        if len(self.waypoints) > 0:
            self.send_waypoints()
            time.sleep(1)
        else:

            # TODO: read lat0, lon0 from colliders into floating point values
            with open('colliders.csv', newline='') as f:
                r = csv.reader(f)
                r1 = next(r)
            f.close()
            
            lat0 = float((r1[0].strip('lat0')))      
            lon0 = float((r1[1].strip(' lon0')))

            # TODO: set home position to (lon0, lat0, 0)
            self.set_home_position(lon0, lat0, 0.0)
            
            # TODO: retrieve current global position
            global_position = [self._longitude, self._latitude, self._altitude]
            
            # TODO: convert to current local position using global_to_local()
            local_position = global_to_local(global_position,
                                             self.global_home)
            
            print('global home {0}, global position {1}, local position {2}'.format(
                self.global_home, self.global_position, self.local_position))

            # Close connection open again after finding a path
            self.disarming_transition()
            self.manual_transition()

            # Read in obstacle map
            data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64',
                              skiprows=2)

            # Define a grid for a particular altitude and safety margin around
            # obstacles
            grid, north_offset, east_offset = create_grid(
                data, TARGET_ALTITUDE, SAFETY_DISTANCE)
            print("North offset = {0}, east offset = {1}".format(north_offset,
                                                                 east_offset))
            
            # GRID CENTER : Define starting point on the grid (this is just grid center)
            # grid_start = (-north_offset, -east_offset)
            
            # TODO: convert start position to current position rather than map
            # center
            grid_start = (int(np.ceil(local_position[0] - north_offset)),
                          int(np.ceil(local_position[1] - east_offset)))
            
            # Set goal as some arbitrary position on the grid
            # grid_goal = (-north_offset + 10, -east_offset + 10)
            # TODO: adapt to set goal as latitude / longitude position and convert
            
            grid_goal = (-north_offset + 483, -east_offset -150)

            
            print('Local Start and Goal: ', grid_start, grid_goal)

            # Run A* to find a path from start to goal
            path, cost = a_star(grid, heuristic, grid_start, grid_goal)
            print('length of plan and cost: ',len(path), cost)

            plot_route(grid, path, grid_start, grid_goal, False)
            print('Plotted chart of initial path')
            print('Original length of path: {}'.format(len(path)))

            # TODO: prune path to minimize number of waypoints
            pruned_path = bres_prune(grid, path)
            print('Pruned path length: {}'.format(len(pruned_path)))
            
            plot_route(grid, pruned_path, grid_start, grid_goal, True)
            print('Plotted chart of pruned path')

            if pruned_path:
                path = pruned_path
            # Convert path to waypoints
            waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]

            """
            Moved this to the beginning to work around the attribute error issue with time
            # Set self.waypoints
            #self.waypoints = waypoints
            # self.send_waypoints()
            """

            print('Time to complete planning: {:.2f}'.format(time.time() - t0))

            # Workaround for the attribute error....
            conn = MavlinkConnection('tcp:{0}:{1}'.format('127.0.0.1', 5760),
                                             timeout=600)
            drone = MotionPlanning(conn, waypoints=waypoints)
            time.sleep(1)
            drone.start()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1',
                        help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--altitude', '-a', type=int, default=10,
                        help='Target flight altitude. Default is 5m')
    parser.add_argument('--safety', '-s', type=int, default=7,
                        help='Margin of safety to maintain around '
                        'obstacles in m.')
    parser.add_argument('--goal_lat', '-glat', type=float,
                        help='Goal position latitude (float)')
    parser.add_argument('--goal_lon', '-glon', type=float,
                        help='Goal position longitude (float)')
    parser.add_argument('--goal_alt', '-galt', type=float,
                        help='Goal position altitude (float)')
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port),
                             timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()

