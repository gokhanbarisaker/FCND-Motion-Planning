import argparse
from functools import reduce
import re
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid
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

    def __init__(self, connection, global_goal, flight_altitude):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.global_goal = global_goal
        self.flight_altitude = flight_altitude

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            self.waypoint_transition()
            # if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
            #     self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
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
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        SAFETY_DISTANCE = 5

        self.target_position[2] = self.flight_altitude

        # TODO: read lat0, lon0 from colliders into floating point values
        lat0, lon0 = self.read_lat_lon('colliders.csv')

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(
            lon0,
            lat0,
            0
        )

        # TODO: retrieve current global position

        # TODO: convert to current local position using global_to_local()
        local_north, local_east, local_alt = global_to_local(self.global_position, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, self.flight_altitude, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid (this is just grid center)
        grid_start = (
            int(np.rint(local_north - north_offset)), 
            int(np.rint(local_east - east_offset))
        )
        # TODO: convert start position to current position rather than map center

        # Set goal as some arbitrary position on the grid
        if(self.global_goal):
            print("Setting goal with global parameters: {}".format(self.global_goal))
            local_goal = global_to_local(self.global_goal, self.global_home)
            grid_goal = (
                int(np.rint(local_goal[0])),
                int(np.rint(local_goal[1]))
            )
        else:
            print("Setting a random goal")
            empty_grid_idx = np.argwhere(grid == 0)
            grid_goal = tuple(empty_grid_idx[np.random.randint(0, len(empty_grid_idx))])

        # TODO: adapt to set goal as latitude / longitude position and convert

        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        print("Found path")
        # TODO: prune path to minimize number of waypoints
        path = self.prune_path(path)
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        print("Prunned path")

        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, self.flight_altitude, 0] for p in path]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()

    def read_lat_lon(self, filename: str):
        with open(filename) as fp:
            line = fp.readline()

        pattern = re.compile(r"lat0 (?P<lat0>-?\d+(?:.\d+)?)|lon0 (?P<lon0>-?\d+(?:.\d+)?)")
        matches = pattern.finditer(line)

        matched_groups = map(lambda x: x.groupdict(), matches)
        found_matched_groups = map(lambda x: dict((key, value)
                                                  for key, value in x.items() if value is not None), matched_groups)
        latlng = reduce(lambda x, y: {**x, **y}, found_matched_groups, {})

        return (float(latlng['lat0']), float(latlng['lon0']))

    def point(self, p):
        return np.array([p[0], p[1], 1.]).reshape(1, -1)

    def collinearity_check(self, p1, p2, p3, epsilon=1e-6):   
        m = np.concatenate((p1, p2, p3), 0)
        det = np.linalg.det(m)
        return abs(det) < epsilon

    # We're using collinearity here, but you could use Bresenham as well!
    def prune_path(self, path):
        pruned_path = [p for p in path]
        # TODO: prune the path!

        i = 0
        while i < len(pruned_path) - 2:
            p1 = self.point(pruned_path[i])
            p2 = self.point(pruned_path[i + 1])
            p3 = self.point(pruned_path[i + 2])

            # If the 3 points are in a line remove
            # the 2nd point.
            # The 3rd point now becomes and 2nd point
            # and the check is redone with a new third point
            # on the next iteration.
            if self.collinearity_check(p1, p2, p3):
                # Something subtle here but we can mutate
                # `pruned_path` freely because the length
                # of the list is check on every iteration.
                pruned_path.remove(pruned_path[i + 1])
            else:
                i += 1
        return pruned_path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    parser.add_argument('--tlon', type=float, default=None,
                        help="Global goal longitude. (Both `tlon` and `tlat` arguments needs to be defined. Otherwise, the defined value will be ignored and a random coordinate will be used)")
    parser.add_argument('--tlat', type=float, default=None,
                        help="Global goal latitude. (Both `tlon` and `tlat` arguments needs to be defined. Otherwise, the defined value will be ignored and a random coordinate will be used)")
    parser.add_argument('--falt', type=float, default=5, help="Flight altitude")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)

    global_goal = None if (args.tlon is None or args.tlat is None) else (args.tlon, args.tlat, 0)
    drone = MotionPlanning(conn, global_goal, args.falt)
    time.sleep(1)

    drone.start()
