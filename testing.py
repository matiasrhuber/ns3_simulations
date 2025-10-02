import ns3ai_gym_env
import gymnasium as gym
import sys
import traceback
import os
import numpy as np

# compatibility shim for np >= 1.24 (ns3ai_gym_env uses deprecated aliases)
if not hasattr(np, "float"):  np.float  = np.float64
if not hasattr(np, "int"):    np.int    = np.int64
if not hasattr(np, "bool"):   np.bool   = np.bool_

print(os.path.abspath("../../../../../"))

class RoundRobinAgent:
    """
    Simple round-robin link assignment:
      STA i -> link (i + step) % numLinks
    Options:
      shift_every: change the rotation every K steps (default 1 = every step)
      offset:      initial global shift
    """
    def __init__(self, action_space, shift_every: int = 1, offset: int = 0):
        self.numStas  = action_space.shape[0]
        # action_space.high is inclusive; handle scalar or per-dim
        high = action_space.high
        self.numLinks = int(high.max() if np.ndim(high) else high) + 1
        self.shift_every = max(1, int(shift_every))
        self.offset = int(offset)
        self.t = 0

    def reset(self):
        self.t = 0

    def get_action(self, obs, reward=0.0, done=False, info=None) -> np.ndarray:
        # every 'shift_every' steps, advance the rotation by 1
        step = (self.t // self.shift_every)
        action = (np.arange(self.numStas) + step + self.offset) % self.numLinks
        self.t += 1
        return action.astype(np.uint32)
    

ns3_settings = {
    'duration':1000,
}
env = gym.make("ns3ai_gym_env/Ns3-v0", 
               targetName="scratch/ns3_simulations/testing", 
               ns3Path="/home/lifistudmlo/ns-3-allinone/ns-3.44",
               ) #ns3Settings=ns3_settings

action_log = False

ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space, ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)

try:
    obs, info = env.reset()
    reward = 0
    done = False
    step_idx = 0

    agent = RoundRobinAgent(env.action_space, shift_every=1, offset=0)

    while True:

        action = agent.get_action(obs)
        if (action_log):
            print(f"Step {step_idx}: action = {action}")

        obs, reward, done, _, info = env.step(action)

        print(f"Step {step_idx}: obs = {obs}")

        if done:
            print("Simulation Ended")
            break

        step_idx += 1

except Exception as e:
    print("Exception occurred: {}".format(e))
    print("If more information is required check ns3ai repo for debugging additions")
    exit(1)


finally:
    print("Observation space: ", ob_space, ob_space.dtype)
    print("Action space: ", ac_space, ac_space.dtype)
    print("Finally exiting...")
    env.close()