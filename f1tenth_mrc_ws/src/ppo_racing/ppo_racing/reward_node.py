import numpy as np
import csv
import scipy.interpolate as sp_int



class RewardNode():
    def __init__(self):
        
        self.start_x = None
        self.start_y = None
        self.start_wp_idx = None
        self.prev_action = np.array([0.0, 0.0])

        self.K1 = 1.0  #speed
        self.K2 = 1.0  #raceline
        self.K3 = 3.0  #crash
        self.K4 = 0.2  #distance
        self.K5 = 0.2  #overtake
        self.K6 = 0.0  #laptime
        self.K7 = 0.3  #smoothnes

        self.waypoints = []
        '''with open('/home/mrc/sim_ws/src/f1tenth_lab6_template/waypoints.csv', mode='r') as file:
            reader = csv.reader(file)
            for row in reader:
                self.waypoints.append([float(i) for i in row])

        self.original_waypoints = self.waypoints
        self.waypoints = np.transpose(self.waypoints)
        
        tck, _ = sp_int.splprep(self.waypoints, s=0)
        u_fine = np.linspace(0, 1, 100)
        interpolated = sp_int.splev(u_fine, tck)
        self.waypoints = list(zip(interpolated[0], interpolated[1]))  # List of (x, y) tuples'''

        waypoints = np.loadtxt('/home/mrc/sim_ws/src/ppo_racing/ppo_racing/Spielberg_waypoints.csv', delimiter=',')

        # Keep only the first two columns
        waypoints = waypoints[:, :2]
        waypoints = waypoints[::5,:]
        self.waypoints = [tuple(row) for row in waypoints]
        #print(self.waypoints)

    def get_reward(self, obs, action):
        ego = obs['ego_idx']
        opp = 1 - ego  # assuming 2 agents
        current_x = obs['poses_x'][ego] 
        current_y = obs['poses_y'][ego]
        opp_x = obs['poses_x'][opp]
        opp_y = obs['poses_x'][opp]
        current_speed = obs['linear_vels_x'][ego] / 3.0
        raceline_error = self.raceline_error(current_x, current_y)
        laptime = self.laptime(obs['lap_times'][ego])
        crash = -obs['collisions'][ego]
        overtake = self.overtake(current_x, current_y, opp_x, opp_y) #-1/0/1
        distance_traveled = self.distance_traveled(current_x, current_y)
        smoothness_penalty = self.smoothness_penalty(action)

        reward = self.K1 * current_speed + self.K2 * raceline_error + self.K3 * crash + self.K4 * distance_traveled \
          + self.K7 * smoothness_penalty + self.K5 * overtake #+ self.K6 * laptime
        
        #print("Reward:", current_speed, raceline_error, reward)

        self.start_x = current_x
        self.start_y = current_y
        

        return reward

    def raceline_error(self, x, y):
        
    
        index = self.waypoint_index(x, y)
        wp1 = self.waypoints[(index+1+len(self.waypoints))%len(self.waypoints)]
        wp2 = self.waypoints[index]
        distance = self.point_to_line_distance(wp1, wp2, x, y)
        distance_clipped = np.clip(distance, 0.0, 2.0) / 2.0

        
                    
        return -distance_clipped

    def point_to_line_distance(self, p1, p2, x, y):
          """
          Calculate the distance from point p3 to the line defined by points p1 and p2.
          
          Parameters:
               p1, p2, p3: Tuples or lists representing points in 2D (x, y)
          
          Returns:
               float: Distance from point p3 to the line through p1 and p2
          """
          # Convert points to numpy arrays
          p1 = np.array(p1)
          p2 = np.array(p2)
          p3 = np.array([x, y])
          
          # Compute the vector from p1 to p2 and from p1 to p3
          line_vec = p2 - p1
          point_vec = p3 - p1

          # Compute the area of the parallelogram (cross product magnitude in 2D)
          area = np.abs(np.cross(line_vec, point_vec))
          
          # Compute the length of the line
          line_length = np.linalg.norm(line_vec)
          
          # Distance is area divided by base length
          distance = area / line_length if line_length != 0 else np.linalg.norm(point_vec)
          
          return distance

    def laptime(self, laptime):
         laptime_reward = 20.0 / laptime
              
         return laptime_reward
    
    def overtake(self, ego_x, ego_y, opp_x, opp_y):
          ego_idx = self.waypoint_index(ego_x, ego_y)
          opp_idx = self.waypoint_index(opp_x, opp_y)
         
          if ego_idx == opp_idx:
               return 0  # same position

          # Forward distance from ego to opponent
          forward_dist = (opp_idx - ego_idx + len(self.waypoints)) % len(self.waypoints)

          # If forward distance is less than half the loop, opponent is ahead
          if forward_dist < len(self.waypoints) / 2:
               return -1  # opponent ahead
          else:
               return 1   # ego ahead
          
    def waypoint_index(self, x, y):
         min_dist = float('inf')
         best_wp_index = 0

         for wp_i in range(len(self.waypoints)):
            dx = self.waypoints[wp_i][0] - x
            dy = self.waypoints[wp_i][1] - y
            distance = np.hypot(dx, dy)
            if distance < min_dist:
                min_dist = distance
                best_wp_index = wp_i

         return best_wp_index

    def remember_start(self, wp_i):
         self.start_wp_idx = wp_i
         self.prev_action = np.array([[0.0, 0.0], [0.0, 0.0]])

         return
    
    def distance_traveled(self, x, y):
         
         curr_way_idx = self.waypoint_index( x, y)
         dist = (curr_way_idx - self.start_wp_idx + len(self.waypoints))%len(self.waypoints)
         if dist < len(self.waypoints) * (9/10):
               return (dist/(len(self.waypoints) * (9/10)))  # opponent ahead
         else:
               return 0   # ego ahead
         
    def smoothness_penalty(self, action):
         delta_action = np.linalg.norm(action - self.prev_action)
         self.prev_action = action

         return -delta_action / 3.0
         


class OpponentWaypointLogger:
    def __init__(self, min_dist=0.5, lap_threshold=1.0, min_lap_points=10):
        self.min_dist = min_dist          # Min dist to log a new point
        self.lap_threshold = lap_threshold  # Max dist to starting point to count as lap complete
        self.min_lap_points = min_lap_points  # Prevent false early completions

        self.opponent_waypoints = []
        self.replace_index = 0
        self.lap_complete = False
        self.start_point = None

    def reset(self):
        self.opponent_waypoints = []
        self.replace_index = 0
        self.lap_complete = False
        self.start_point = None

    def add_position(self, x, y):
        point = np.array([x, y])

        if not self.opponent_waypoints:
            self.opponent_waypoints.append(point)
            self.start_point = point
            return

        # Check distance from previous point
        if not self.lap_complete:
            last_point = self.opponent_waypoints[-1]
        else:
            last_index = (self.replace_index - 1) % len(self.opponent_waypoints)
            last_point = self.opponent_waypoints[last_index]

        dist = np.linalg.norm(point - last_point)

        if dist >= self.min_dist:
            if not self.lap_complete:
                self.opponent_waypoints.append(point)

                # ✅ Check for lap completion
                if (
                    len(self.opponent_waypoints) >= self.min_lap_points and
                    np.linalg.norm(point - self.start_point) <= self.lap_threshold
                ):
                    self.lap_complete = True
                    self.replace_index = 0
            else:
                # ✅ Circular replacement mode
                self.opponent_waypoints[self.replace_index] = point
                self.replace_index = (self.replace_index + 1) % len(self.opponent_waypoints)

    def get_waypoints(self):
        return np.array(self.opponent_waypoints)

    def lap_finished(self):
        return self.lap_complete