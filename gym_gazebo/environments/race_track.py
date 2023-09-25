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


class RaceTrackEnv(gym.env):
    """Environment for training ai on a gazebo race track"""
    startingPosition: Tuple[np.int32, np.int32, np.int32]
    nCheckpointsLookAhead: int

    _checkpoints: np.array
    _location: np.array
    _speed: np.uint16
    _acceleration: np.int16
    _energyConsumption: np.uint32
    _windResistance: np.int16
    _throttle: np.uint16

    def __init__(self, startingPosition: Tuple[np.int32, np.int32, np.int32], nCheckpointsLookAhead: int = 3):
        """Initialize the environment"""
        self.startingPosition = startingPosition
        self.nCheckpointsLookAhead = nCheckpointsLookAhead

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
                "throttle": spaces.Box(low=0, high=1000, shape=(1, 1), dtype=np.uint16)
            }
        )

        # There are two actions to take at each step
        self.action_space = spaces.Dict(
            {
                # Choose a throttle ranging from 0 to 1000
                "throttle": spaces.Box(low=0, high=1000, shape=(1, 1), dtype=np.uint16),
                # Choose a steering direction ranging from 0 to 2000
                "steering": spaces.Box(low=0, high=2000, shape=(1, 1), dtype=np.uint16)
            }
        )

    def reset(self, seed=None, options=None):
        """Called to reset the environment"""

        # Propagate passed seed
        super().reset(seed=seed)

        self._location = self.startingPosition
        self._checkpoints = (np.array([0, 0, 0]), np.array([0, 0, 0]), np.array([0, 0, 0]))
        self._speed = np.uint16(0)
        self._acceleration = np.int16(0)
        self._energyConsumption = np.uint32(0)
        self._windResistance = np.int16(0)
        self._throttle = np.uint16(0)

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
        reachedCheckpoint = np.array_equal(self._location, self._checkpoints[0])

        reward = 1 if terminated else 0
        reward += 0.1 if reachedCheckpoint else 0

        # Need to play with energy consumption punishment
        reward -= (self._energyConsumption / 500000)

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
