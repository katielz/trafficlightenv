from gym.envs.registration import register

register(
    id='traffic-v0',
    entry_point='traffic_light.envs:TrafficLightEnv',
)

