import gym

env = gym.make('f110_gym:f110-v0')

print("Observation space:", env.observation_space)
print("Action space:", env.action_space)

# To inspect bounds (if applicable):
if hasattr(env.action_space, 'low') and hasattr(env.action_space, 'high'):
    print("Action space low:", env.action_space.low)
    print("Action space high:", env.action_space.high)

if hasattr(env.observation_space, 'low') and hasattr(env.observation_space, 'high'):
    print("Observation space low:", env.observation_space.low)
    print("Observation space high:", env.observation_space.high)
