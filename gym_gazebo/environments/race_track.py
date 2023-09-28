"""
Observations:
(Coordinate = latitude, longitude, altitude)
The coordinates of the next n checkpoints
The coordinates of the agent

The speed of the agent
The acceleration of the agent

The energy consumption of the agent
The wind resistance against the agent

The current throttle value of the agent

Actions:
Throttle [0 to 1000] where 1000 represents full speed and 0 is stopped
Steering direction, ranging from 0 to 2000 where 0 is full left, 2000 is full right

Signals:
Checkpoint reached signal
Course completed signal

Rewards:
TBD
(temp):
go through checkpoint + 1
every tick: - 0.001 + (energy consumption) * 0.001

"""
from typing import Any, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from gymnasium.core import ObsType
from gazebo_ai.world_control import WorldControlClient
from rclpy.node import Node

from std_msgs.msg import String

import rclpy

class Subscriber(Node):
    def __init__(self, topic: str, topicType: Any, topicConverter: function, defaultValue: Any):
        """Create a subscriber

        Args:
            topic (str): the ROS topic to subscribe to
            topicType (Any): the ROS type of the topic
            topicConverter (function): the conversion function for the ROS type to the value stored in the subscriber
        """
        super().__init__('subscriber_' + topic)
        self.topicConverter = topicConverter
        self.defaultValue = defaultValue
        self.value = defaultValue
        self.subscription = self.create_subscription(topicType, topic, self.callback, 10)

    def callback(self, msg):
        self.get_logger().info('Subscriber received message.')
        self.value = self.topicConverter(msg)

    def getValue(self):
        return self.value
    
    def reset(self):
        self.value = self.defaultValue
    
def locationListConverter(msg):
    return msg

def locationConverter(msg):
    return msg

def speedConverter(msg):
    return msg

def accelerationConverter(msg):
    return msg

def energyConsumptionConverter(msg):
    return msg 

def windResistanceConverter(msg):
    return msg

def throttleConverter(msg):
    return msg

def directionConverter(msg):
    return msg

class RaceTrackEnv(gym.Env):
    """Environment for training ai on a gazebo race track"""

    def __init__(self, controlSvc: str, checkpointTopic: str, locationTopic: str, speedTopic: str, accelerationTopic: str, energyConsumptionTopic: str, windResistanceTopic: str, throttleTopic: str, directionTopic: str, nCheckpointsLookAhead: int):
        """Initialize the environment"""
        self.checkpointSubscriber        = Subscriber(checkpointTopic, '', locationListConverter, '')
        self.locationSubscriber          = Subscriber(locationTopic, '', locationConverter, '')
        self.speedSubscriber             = Subscriber(speedTopic, '', locationConverter, 0)
        self.accelerationSubscriber      = Subscriber(accelerationTopic, '', accelerationConverter, 0)
        self.energyConsumptionSubscriber = Subscriber(energyConsumptionTopic, '', energyConsumptionConverter, 0)
        self.windResistanceSubscriber    = Subscriber(windResistanceTopic, '', windResistanceConverter, 0)
        self.throttleSubscriber          = Subscriber(throttleTopic, '', throttleConverter, 0)
        self.directionSubscriber         = Subscriber(directionTopic, '', directionConverter, 0)

        # Define all the inputs for the model
        self.observation_space = spaces.Dict(
            {
                "checkpoints": spaces.Tuple(
                    [spaces.Box(low=-2147483648, high=2147483647, shape=(1, 3), dtype=np.int32) for i in
                     range(nCheckpointsLookAhead)]
                ),
                "location": spaces.Box(low=-2147483648, high=2147483647, shape=(1, 3), dtype=np.int32),
                "speed": spaces.Box(low=0, high=65535, shape=(1, 1), dtype=np.uint16),
                "acceleration": spaces.Box(low=-32768, high=32767, shape=(1, 1), dtype=np.int16),
                "energyConsumption": spaces.Box(low=0, high=4294967295, shape=(1, 1), dtype=np.uint32),
                "windResistance": spaces.Box(low=-32768, high=32767, shape=(1, 1), dtype=np.int16),
                "throttle": spaces.Box(low=0, high=1000, shape=(1, 1), dtype=np.uint16),
                "direction": spaces.Box(low=-2147483648, high=2147483647, shape=(1, 2), dtype=np.int32) 
            }
        )

        # There are two actions to take at each step
        self.action_space = spaces.Dict(
            {
                # Choose a throttle ranging from 0 to 1000
                "throttle": spaces.Box(low=0, high=1000, shape=(1, 1), dtype=np.uint16),
                # Choose a steering direction ranging from 0 to 2000
                "steering": spaces.Box(low=-1000, high=1000, shape=(1, 1), dtype=np.int16)
            }
        )
 
        # Start ROS integration
        rclpy.init()

        self.client = WorldControlClient(controlSvc)
        self.client.get_logger().info('Constructing World Control Client in AI Gymnasium.')
    
    def __del__(self):
        self.client.destroy_node()
        rclpy.shutdown()    


    def reset(self, seed=None, options=None):
        """Called to reset the environment"""

        # Propagate passed seed
        super().reset(seed=seed)

        # Call Reset ROS service
        self.client.reset()
        self.client.get_logger().info('Called World Control Client Reset in AI Gymnasium')

        observation = self._get_obs()
        info = self._get_info()

        return observation, info

    def step(self, action):
        """Called at each step of the process"""
        # Get the chosen throttle
        targetThrottle = action["throttle"][0][0]

        # Get the chosen steering
        targetSteering = action["steering"][0][0]

        terminated = np.array_equal(self._checkpoints[0], [-1, -1, -1])

        # TODO: Add Error Margin
        reachedCheckpoint = np.array_equal(self._location, self._checkpoints[0])

        reward = 1 if terminated else 0
        reward += 0.1 if reachedCheckpoint else 0

        # Need to play with energy consumption punishment
        reward -= (self._energyConsumption / 500000)

        # Call ROS Step
        self.client.step(100)

        observation = self._get_obs()
        info = self._get_info()

        return observation, reward, terminated, False, info

    def _get_obs(self):
        """Convenience Function to convert stored data into an observation dictionary"""
        return {
            "checkpoints": self._checkpoints,
            "location": self._location,
            "speed": self._speed,
            "acceleration": self._acceleration,
            "energyConsumption": self._energyConsumption,
            "windResistance": self._windResistance,
            "throttle": self._throttle
        }

    def _listener_tf(self, msg):
        print(msg)

    def _get_info(self):
        return {}

if __name__ == '__main__':
    print("Constructing Sample Env")
    env = RaceTrackEnv('/world/buoyancy/control', (0,0,0))
    print("Reseting Env")
    env.reset()

    del env

    print("Deleted Env")