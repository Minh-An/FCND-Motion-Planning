import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import numpy.linalg as LA

from planning_utils import create_grid_and_edges, crosses, heuristic, a_star
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
import networkx as nx

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
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
        print('takeoff transition to ({0}, {1}, {2})'.format(self.local_position[0], self.local_position[1], self.target_position[2]))
        print('Target position ({0}, {1}, {2})'.format(self.target_position[0], self.target_position[1], self.target_position[2]))
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

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


    def global_to_grid(self, lat, lon, offset):
        local = global_to_local((lon, lat, 0.0), self.global_home)
        return (int(local[0] - offset[0]), int(local[1]-offset[1]))

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE

        # read lat0, lon0 from colliders into floating point values
        # first line format: lat0 _______, lon0 __________
        lines = open('colliders.csv').readline().split(", ")
        lat0 = float(lines[0].split(" ")[1])
        lon0 = float(lines[1].split(" ")[1])

        # set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0.0)

        # retrieve current global position
        current_global = self.global_position
 
        # convert to current local position using global_to_local()
        current_local = global_to_local(current_global, self.global_home)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        grid, north_offset, east_offset, edges  = create_grid_and_edges(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # Define starting point on the grid to current position
        grid_start = (int(current_local[0] - north_offset), int(current_local[1]-east_offset))
        
        # Set goal using latitude / longitude position and convert to grid
        # goal latitude and longitude can be changed for testing
        # test case 1
        goal_lat = 37.793793
        goal_long = -122.396803
        # test case 2
        #goal_lat = 37.797071
        #goal_long = -122.401214
        grid_goal = self.global_to_grid(goal_lat, goal_long, (north_offset, east_offset))
        print(grid_goal)
        #grid_goal = (grid_start[0]+20, grid_start[1]+20)


        g = nx.Graph()
        for e in edges:
            p1 = e[0]
            p2 = e[1]
            
            g.add_edge(p1, p2)

            if crosses(grid, np.array(grid_start), np.array(p1)) == False:
                g.add_edge(p1, grid_start)
            if crosses(grid, np.array(grid_start), np.array(p2)) == False:
                g.add_edge(p2, grid_start)

            if crosses(grid, np.array(p1), np.array(grid_goal)) == False:
                g.add_edge(p1, grid_goal) 
            if crosses(grid, np.array(p2), np.array(grid_goal)) == False:
                g.add_edge(p2, grid_goal)

        
        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', grid_start, grid_goal)
        path, _ = a_star(g, heuristic, grid_start, grid_goal)

        for p in path:
            print(p)

        # Convert path to waypoints
        waypoints = [[int(p[0] + north_offset), int(p[1] + east_offset), TARGET_ALTITUDE, 0] for p in path]

        for wp in waypoints:
            print(wp)

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


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()