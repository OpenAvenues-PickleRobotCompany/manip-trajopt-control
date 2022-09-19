from typing import Final

# Default pybullet frequency (see documentation)
FREQUENCY: Final = 240
TIMESTEP: Final = 1. / FREQUENCY
MAX_DURATION: Final = 12  # seconds
MAX_SIMULATION_STEPS: Final = int(MAX_DURATION * FREQUENCY)
COMMAND_PERIOD = 0.1  # seconds
COMMAND_PERIOD_STEPS = int(COMMAND_PERIOD * FREQUENCY)
