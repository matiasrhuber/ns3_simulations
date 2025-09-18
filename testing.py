import ns3ai_gym_env
import gymnasium as gym
import sys
import traceback
import os
print(os.path.abspath("../../../../../"))

class RRAgent:
    def __init__(self):
        pass

    def get_action(self, obs, reward, done, info):
        num_links = len(obs)

        return 1
print('here1')
ns3_settings = {
    'duration':1000,
}
env = gym.make("ns3ai_gym_env/Ns3-v0", 
               targetName="scratch/ns3_simulations/testing", 
               ns3Path="/home/lifistudmlo/ns-3-allinone/ns-3.44",
               ) #ns3Settings=ns3_settings
print('here2')
ob_space = env.observation_space
ac_space = env.action_space
print("Observation space: ", ob_space, ob_space.dtype)
print("Action space: ", ac_space, ac_space.dtype)

try:
    obs, info = env.reset
    reward = 0
    done = False

    agent = RRAgent()

    while True:

        action = agent.get_action(obs, reward, info, done)

        obs, reward, done, _, info = env.step(action)

        if done:
            print("Simulation Ended")
            break

except Exception as e:
    print("Exception occurred: {}".format(e))
    print("If more information is required check ns3ai repo for debugging additions")
    exit(1)


finally:
    print("Observation space: ", ob_space, ob_space.dtype)
    print("Action space: ", ac_space, ac_space.dtype)
    print("Finally exiting...")
    env.close()