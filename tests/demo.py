import os

from mjc_mwe import (
    AntEnv,
    HalfCheetahEnv,
    HopperEnv,
    HumanoidEnv,
    HumanoidStandupEnv,
    InvertedDoublePendulumEnv,
    InvertedPendulumEnv,
    PusherEnv,
    ReacherEnv,
    SwimmerEnv,
    Walker2dEnv,
)


if __name__ == '__main__':
    env_clses = [
        AntEnv,
        HalfCheetahEnv,
        HopperEnv,
        HumanoidEnv,
        HumanoidStandupEnv,
        InvertedDoublePendulumEnv,
        InvertedPendulumEnv,
        PusherEnv,
        ReacherEnv,
        SwimmerEnv,
        Walker2dEnv,
    ]
    xml_files = [
        "ant.xml",
        "half_cheetah.xml",
        "hopper.xml",
        "humanoid.xml",
        "humanoidstandup.xml",
        "inverted_double_pendulum.xml",
        "inverted_pendulum.xml",
        "point.xml",
        "pusher.xml",
        "reacher.xml",
        "swimmer.xml",
        "walker2d.xml",
    ]
    for env_cls, xml in zip(env_clses, xml_files):
        print(env_cls)
        env = env_cls(xml_file=os.path.abspath(f"../mjc_mwe/assets/{xml}"))
        print(env)
        print(env.reset())
        print(env.step(env.action_space.sample()))
