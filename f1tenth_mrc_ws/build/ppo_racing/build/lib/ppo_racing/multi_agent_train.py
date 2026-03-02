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

def main():
    rclpy.init()
    env = gym.make('f110_gym:f110-v0',
                            map='/home/mrc/sim_ws/src/f1tenth_gym_ros/maps/levine',
                            map_ext='.png',
                            num_agents=2,
                            integrator = Integrator.RK4,
                            timestep = 0.05)

    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

    # PPO hyperparameters
    N = 2048                # steps before update
    batch_size = 64
    n_epochs = 10
    alpha = 3e-4
    n_games = 20000
    figure_file = f'plots/ros2_agent_learning_curve_multi_{timestamp}.png'
    beta = 0.3

    scan_size = 108
    odom_size = 6
    waypoint_size = 60
    collision_size = 1
    prev_action_size = 2


    input_size = scan_size + odom_size *2 + waypoint_size *2+ collision_size + prev_action_size

    load_model = False

    if not load_model:
        parent_path = "/home/mrc/sim_ws/src/ppo_racing/ppo_racing/tmp"  # Change this to your target directory
        ppo_dir = create_unique_ppo_dir(parent_path)
        chkpt_dir = ppo_dir
    else:
        chkpt_dir = 'tmp/ppo_multi'

    agent = Agent(
        n_actions=2,
        batch_size=batch_size,
        alpha=alpha,
        n_epochs=n_epochs,
        input_dims=input_size,
        chkpt_dir=chkpt_dir,
    )

    if load_model:
        agent.load_models()

    best_score = -np.inf
    score_history = []

    learn_iters = 0
    avg_score = 0
    n_steps = 0

    '''# Load waypoints from CSV at module level (only once)
    #WAYPOINTS = np.loadtxt('raceline.csv', delimiter=',', skiprows=1)[:, :2]  # shape (N, 2)
    WAYPOINTS = []
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
    WAYPOINTS = waypoints[:, :2].flatten()

    reward_node_ego = RewardNode()
    reward_node_opp = RewardNode()

    start_time = time.time()


    for i in range(n_games):
        poses, wp_start_i_ego, wp_start_i_opp = reset_poses(WAYPOINTS)
        #print("poses", poses)
        #time.sleep(1.5)
        observation, _, _, _ = env.reset(poses)
        reward_node_ego.remember_start(wp_start_i_ego)
        reward_node_opp.remember_start(wp_start_i_opp)
        prev_action_ego = np.array([0.0, 0.0])
        prev_action_opp = np.array([0.0, 0.0])
        observation_ego = preprocess_observation(observation, WAYPOINTS, prev_action_ego, ego=0)
        observation_opp = preprocess_observation(observation, WAYPOINTS, prev_action_opp, ego=1)
        done = False
        score = 0
        duration = 0.

        if (i + 1) % 1000 == 0:
            elapsed = time.time() - start_time
            elapsed_minutes = elapsed / 60
            print(f"[Episode {i+1}] Time elapsed: {elapsed_minutes:.2f} minutes")

        while not done:
            
            action_ego, prob_ego, val_ego = agent.choose_action(observation_ego)
            action_ego = np.array(action_ego)
            smoothed_action_ego = beta * prev_action_ego + (1 - beta) * action_ego
            prev_action_ego = smoothed_action_ego
            action_opp, prob_opp, val_opp = agent.choose_action(observation_opp)
            action_opp = np.array(action_opp)
            smoothed_action_opp = beta * prev_action_opp + (1 - beta) * action_opp
            prev_action_opp = smoothed_action_opp

            action_sized_ego = [smoothed_action_ego[0] * 0.34, smoothed_action_ego[1] * 3.0]
            action_sized_opp = [smoothed_action_opp[0] * 0.34, smoothed_action_opp[1] * 3.0]
            action_full =np.array([[action_sized_ego[0], action_sized_ego[1]], [action_sized_opp[0], action_sized_opp[1]]])
            
            obs_, reward, done, info = env.step(action_full)
            
            #print("collisions:", obs_['collisions'], "done:", done)
            
            if obs_['collisions'][0] == 1 or obs_['collisions'][1] == 1 or duration >= 120.0:
                done = True
            else:
                done = False

            reward_ego = reward_node_ego.get_reward(obs_, action_ego)
            reward_opp = reward_node_opp.get_reward(obs_, action_opp)
            #done = check_done()
            #print("observation:", obs_)
            #time.sleep(0.01)
            #print("reward:", reward)
            obs_ego = preprocess_observation(obs_, WAYPOINTS, prev_action_ego, ego=0)
            obs_opp = preprocess_observation(obs_, WAYPOINTS, prev_action_opp, ego=1)
            #print(obs_ego, obs_opp)
            #time.sleep(1)
            n_steps += 1
            score += reward_ego
            score += reward_opp
            agent.remember(observation_ego, action_ego, prob_ego, val_ego, reward_ego, done)
            agent.remember(observation_opp, action_opp, prob_opp, val_opp, reward_opp, done)

            if n_steps % N == 0:
                agent.learn()
                learn_iters += 1

            observation_ego = obs_ego
            observation_opp = obs_opp
            duration = duration + 0.05

        score_history.append(score)
        avg_score = np.mean(score_history[-100:])

        if avg_score > best_score:
            best_score = avg_score
            agent.save_models()

        print(f'episode {i} | score: {score:.1f} | avg score: {avg_score:.1f} | '
              f'time steps: {n_steps} | learn iters: {learn_iters}')

    x = [i+1 for i in range(len(score_history))]
    os.makedirs(os.path.dirname(figure_file), exist_ok=True)
    plot_learning_curve(x, score_history, figure_file)

def preprocess_observation(obs, WAYPOINTS, prev_action, lidar_max_range=30.0, max_speed=3.0, ego=0):
    #ego = obs['ego_idx']
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
    next_opp_waypoints = get_next_waypoints_flat((rel_pose[0]+ego_pose[0], rel_pose[1]+ego_pose[1]), WAYPOINTS)

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
        next_opp_waypoints,
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

    # Opponent car placed 15 waypoints ahead
    opp_idx = (ego_idx + 15) % num_waypoints
    wp_prev_opp = waypoints[(opp_idx - 1) % num_waypoints]
    wp_curr_opp = waypoints[opp_idx]
    wp_next_opp = waypoints[(opp_idx + 1) % num_waypoints]
    opp_yaw = np.arctan2(wp_next_opp[1] - wp_prev_opp[1], wp_next_opp[0] - wp_prev_opp[0])

    poses = np.array([
        [wp_curr[0], wp_curr[1], ego_yaw],
        [wp_curr_opp[0], wp_curr_opp[1], opp_yaw]
    ])

    '''change = np.random.randint(0,1)
    if change == 1:
        ego_idx , opp_idx = opp_idx , ego_idx
        poses = poses[::-1]'''

    #poses = np.array([[0.0, 0.0, 0.0], [2.0, 0.5, 0.0]])
    #ego_idx = 0

    return poses, ego_idx, opp_idx

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



if __name__ == '__main__':
    main()
