from maze.core.env.maze_env import MazeEnv
from maze.core.wrappers.maze_gym_env_wrapper import GymMazeEnv
import sys
#sys.path.insert(0,'/Users/katiezelvin/PycharmProjects/traffic-light/traffic_light/envs/__init__')
#from __init__ import __init__

from traffic_light.envs.traffic_env import TrafficLightEnv

def make_env(name: str) -> MazeEnv:
    custom_gym_env = TrafficLightEnv()
    return GymMazeEnv(custom_gym_env)

