import numpy as np
import scipy.interpolate as sp_int
import csv
from ppo_agent import Agent
from utils import plot_learning_curve
import gym
import rclpy
import os
from reward_node import RewardNode
import time
from f110_gym.envs.base_classes import Simulator, Integrator
from datetime import datetime
from custom_rendering import CarTrackRecorder

def main():
    rclpy.init()
    env = gym.make('f110_gym:f110-v0',
                            map='/home/mrc/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_map',
                            map_ext='.png',
                            num_agents=2,
                            integrator = Integrator.RK4,
                            timestep = 0.05,
                            render_mode=None)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # PPO hyperparameters
    N = 4096               # steps before update
    batch_size = 128
    n_epochs = 10
    alpha = 3e-4
    n_games = 3000
    figure_file = f'plots/ros2_agent_learning_curve_{timestamp}.png'
    beta = 0.3
    episode_length = 30.0 #seconds
    

    scan_size = 108
    odom_size = 6
    waypoint_size = 60
    collision_size = 1
    prev_action_size = 2


    input_size = scan_size + odom_size *2 + waypoint_size + collision_size + prev_action_size
    
    load_model = False

    if not load_model:
        parent_path = "/home/mrc/sim_ws/src/ppo_racing/ppo_racing/tmp"  # Change this to your target directory
        ppo_dir = create_unique_ppo_dir(parent_path)
        chkpt_dir = ppo_dir
    else:
        chkpt_dir = 'tmp/ppo'

    agent = Agent(
        n_actions=2,
        batch_size=batch_size,
        alpha=alpha,
        n_epochs=n_epochs,
        input_dims=input_size,
        chkpt_dir = chkpt_dir,
    )

    

    if load_model:
        agent.load_models()
    

    



    best_score = -np.inf
    score_history = []

    learn_iters = 0
    avg_score = 0
    n_steps = 0
    

    # Load waypoints from CSV at module level (only once)
    #WAYPOINTS = np.loadtxt('raceline.csv', delimiter=',', skiprows=1)[:, :2]  # shape (N, 2)
    '''WAYPOINTS = []
    with open('/home/mrc/sim_ws/src/f1tenth_lab6_template/waypoints.csv', mode='r') as file:
        reader = csv.reader(file)
        for row in reader:
            WAYPOINTS.append([float(i) for i in row])

    
    WAYPOINTS= np.transpose(WAYPOINTS)
    
    tck, _ = sp_int.splprep(WAYPOINTS, s=0)
    u_fine = np.linspace(0, 1, 100)
    interpolated = sp_int.splev(u_fine, tck)
    x_vals, y_vals = interpolated
    WAYPOINTS = np.vstack((x_vals, y_vals)).T.flatten()'''
    waypoints = np.loadtxt('/home/mrc/sim_ws/src/ppo_racing/ppo_racing/Spielberg_waypoints.csv', delimiter=',')

    # Keep only the first two columns
    waypoints = waypoints[::5,:]
    waypoints = waypoints[:, :2]
    WAYPOINTS = waypoints.flatten()
    render_waypoints = [tuple(row) for row in waypoints]
    #print(WAYPOINTS)

    reward_node = RewardNode()

    start_time = time.time()



    #visualizer = CarTrackRecorder(
    #track_image_path='/home/mrc/sim_ws/src/f1tenth_gym_ros/maps/Spielberg_map.png',
    #track_extent=[-84.85, 31.1, -36.3, 79.6],
    #raceline_waypoints=render_waypoints
    #)


    for i in range(n_games):
        
        frames = []
        poses, wp_start_i = reset_poses(WAYPOINTS)
        #print("poses", poses)
        #time.sleep(1.5)
        observation, _, _, _ = env.reset(poses)
        reward_node.remember_start(wp_start_i)
        prev_action = np.array([0.0, 0.0])
        opp_action = get_opp_action(observation, WAYPOINTS)
        observation = preprocess_observation(observation, WAYPOINTS, prev_action)
        done = False
        score = 0
        duration = 0.

        if (i + 1) % 1000 == 0:
            elapsed = time.time() - start_time
            elapsed_minutes = elapsed / 60
            print(f"[Episode {i+1}] Time elapsed: {elapsed_minutes:.2f} minutes")



        while not done:

            #if i % 100 == 0:
            #    env.render()
            
            action, prob, val = agent.choose_action(observation)
            action = np.array(action)
            smoothed_action = beta * prev_action + (1 - beta) * action
            prev_action = smoothed_action

            action_sized = [np.clip(smoothed_action[0], -0.34, 0.34), np.clip(smoothed_action[1], 0.0, 3.0)]
            action_full =np.array([[action_sized[0], action_sized[1]], [opp_action[0], opp_action[1]]])

            #if (i+1)%100==0: print(action_sized)
            obs_, reward, done, info = env.step(action_full)
            #print(obs_)
            opp_action = get_opp_action(obs_, WAYPOINTS)
            #if (i+1)%200==0:visualizer.draw_frame([(obs_['poses_x'][0], obs_['poses_y'][0] , obs_['poses_theta'][0]), \
            #                             (obs_['poses_x'][1], obs_['poses_y'][1] , obs_['poses_theta'][1])])
            #time.sleep(0.5)
            #print("collisions:", obs_['collisions'], "done:", done)
            
            if obs_['collisions'][0] == 1 or duration >= episode_length:
                done = True
            else:
                done = False

            reward = reward_node.get_reward(obs_, np.array(action_sized))
            if duration >= episode_length and obs_['collisions'][0] == 0:
                reward = reward + 0.5
            #done = check_done()
            #print("observation:", obs_)
            #time.sleep(0.5)
            #print("reward:", reward)
            obs_ = preprocess_observation(obs_, WAYPOINTS, prev_action)
            
            n_steps += 1
            score += reward
            agent.remember(observation, action, prob, val, reward, done)

            if n_steps % N == 0:
                agent.learn()
                learn_iters += 1

            observation = obs_
            duration = duration + 0.05

        



        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        if avg_score > best_score:
            
            best_score = avg_score
            agent.save_models()

        

        print(f'episode {i} | score: {score:.1f} | avg score: {avg_score:.1f} | '
              f'time steps: {n_steps} | learn iters: {learn_iters}')
        
    #visualizer.save_video(path=f"videos/training_run{timestamp}.mp4", fps=20)

    x = [i+1 for i in range(len(score_history))]
    os.makedirs(os.path.dirname(figure_file), exist_ok=True)
    plot_learning_curve(x, score_history, figure_file)

def preprocess_observation(obs, WAYPOINTS, prev_action, lidar_max_range=30.0, max_speed=3.0):
    ego = obs['ego_idx']
    opp = 1 - ego  # assuming 2 agents

    # Ego scan normalized
    scan = np.clip(obs['scans'][ego], 0.0, 30.0) / 30.0
    scan_short = []

    for k in range(len(scan)):
        if k%10==0:
            scan_short.append(scan[k])
    

    # Ego pose
    ego_pose = np.array([
        obs['poses_x'][ego] / 100.0,
        obs['poses_y'][ego] / 100.0,
        (obs['poses_theta'][ego]) / np.pi
    ])

    # Ego velocity
    ego_vel = np.array([
        obs['linear_vels_x'][ego] / max_speed,
        obs['linear_vels_y'][ego] / max_speed,
        obs['ang_vels_z'][ego] / 3.2
    ])

    # Opponent relative pose
    rel_pose = np.array([
        (obs['poses_x'][opp] - obs['poses_x'][ego]) / 100.0,
        (obs['poses_y'][opp] - obs['poses_y'][ego]) / 100.0,
        (obs['poses_theta'][opp] - obs['poses_theta'][ego]) / np.pi
    ])

    # Opponent velocity
    opp_vel = np.array([
        obs['linear_vels_x'][opp] / max_speed,
        obs['linear_vels_y'][opp] / max_speed,
        obs['ang_vels_z'][opp] / 3.2
    ])

    # Collision (convert to scalar float)
    collision = np.array([float(obs['collisions'][ego])])

    WAYPOINTS = WAYPOINTS / 100.0

    next_waypoints = get_next_waypoints_flat((ego_pose[0],ego_pose[1]), WAYPOINTS)

    prev_action_array = np.array(prev_action)





    # Combine all
    state_vector = np.concatenate([
        scan_short,
        ego_pose,
        ego_vel,
        rel_pose,
        opp_vel,
        collision,
        next_waypoints,
        prev_action_array
    ])

    state_vector = np.nan_to_num(state_vector, nan=0.0, posinf=1e3, neginf=-1e3)
    state_vector = np.clip(state_vector, -1e3, 1e3)


    return state_vector.astype(np.float32)

def reset_poses(flat_waypoints):
    """
    Resets the poses of the ego and opponent car using looped, flattened waypoints.

    Args:
        flat_waypoints (np.ndarray): 1D array [x0, y0, x1, y1, ..., xN, yN]

    Returns:
        poses (np.ndarray): shape (2, 3), each row is [x, y, yaw] for ego and opponent.
    """
    assert len(flat_waypoints) % 2 == 0, "Waypoint list must be even-length."
    waypoints = flat_waypoints.reshape(-1, 2)
    num_waypoints = waypoints.shape[0]

    # Choose a random index
    ego_idx = np.random.randint(0, num_waypoints)

    # Circular indexing for yaw calculation
    wp_prev = waypoints[(ego_idx - 1) % num_waypoints]
    wp_curr = waypoints[ego_idx]
    wp_next = waypoints[(ego_idx + 1) % num_waypoints]
    ego_yaw = np.arctan2(wp_next[1] - wp_prev[1], wp_next[0] - wp_prev[0])

    # Opponent car placed 2 waypoints ahead
    opp_idx = (ego_idx + 15) % num_waypoints
    wp_prev_opp = waypoints[(opp_idx - 1) % num_waypoints]
    wp_curr_opp = waypoints[opp_idx]
    wp_next_opp = waypoints[(opp_idx + 1) % num_waypoints]
    opp_yaw = np.arctan2(wp_next_opp[1] - wp_prev_opp[1], wp_next_opp[0] - wp_prev_opp[0])

    poses = np.array([
        [wp_curr[0], wp_curr[1], ego_yaw],
        [wp_curr_opp[0], wp_curr_opp[1], opp_yaw]
    ])

    #poses = np.array([[0.0, 0.0, 0.0], [2.0, 0.5, 0.0]])
    #ego_idx = 0

    return poses, ego_idx

def get_next_waypoints_flat(car_pos, flat_waypoints, num_points=30):
    """
    Find the closest waypoint to the car and return the next `num_points` as a flat array.
    
    Args:
        car_pos (tuple): (x, y) position of the car
        flat_waypoints (list or np.ndarray): Flattened list of waypoints [x0, y0, x1, y1, ...]
        num_points (int): Number of future waypoints to return
    
    Returns:
        np.ndarray: Flattened array of the next `num_points` waypoints
    """
    # Reshape into [N, 2]
    waypoints = np.array(flat_waypoints).reshape(-1, 2)

    # Find closest index
    dists = np.linalg.norm(waypoints - np.array(car_pos), axis=1)
    closest_idx = np.argmin(dists)

    # Select next waypoints with wrap-around
    total = waypoints.shape[0]
    indices = [(closest_idx + i) % total for i in range(num_points)]
    next_waypoints = waypoints[indices]

    # Flatten and return
    return next_waypoints.flatten()

def create_unique_ppo_dir(parent_dir, base_name="ppo"):
    i = 1
    while True:
        dir_name = f"{base_name}{i}"
        full_path = os.path.join(parent_dir, dir_name)
        if not os.path.exists(full_path):
            os.makedirs(full_path)
            print(f"Created directory: {full_path}")
            return full_path
        i += 1

def get_opp_action(obs, flat_waypoints, L=3.0, K=0.5):
    opp_pos = np.array([obs['poses_x'][1], obs['poses_y'][1]])
    opp_yaw = obs['poses_theta'][1]
    waypoints = np.array(flat_waypoints).reshape(-1, 2)

    # Find closest index
    dists = np.linalg.norm(waypoints - np.array(opp_pos), axis=1)
    closest_idx = np.argmin(np.abs(dists-L))

    waypoint = waypoints[closest_idx]

    # Transform waypoint into vehicle frame
    dx = waypoint[0] - opp_pos[0]
    dy = waypoint[1] - opp_pos[1]
    x_car = np.cos(-opp_yaw) * dx - np.sin(-opp_yaw) * dy
    y_car = np.sin(-opp_yaw) * dx + np.cos(-opp_yaw) * dy

    # Compute steering angle
    curvature = 2 * y_car / (L ** 2)
    steering_angle = np.clip(K * curvature, -0.34, 0.34)
    velocity = 2.5 - 10 * np.abs(steering_angle)

    return [steering_angle, velocity]

if __name__ == '__main__':
    main()
