""" Default configuration and hyperparameters for agent objects. """
import numpy as np
import rospkg

import roslib
try:
    # AgentROS
    config_KUKA = {
        #TODO: It might be worth putting this in JSON/yaml format so C++ can read it.
        #baisc
        'smooth_noise': True,
        'smooth_noise_var': 2.0,
        'smooth_noise_renormalize': True,

        # communication:
        'action_command_topic': 'gps_controller_position_command',
        'reset_command_topic': 'gps_controller_position_command',
        'data_request_topic': 'gps_controller_data_request',
        'sample_result_topic': 'gps_controller_report',
        'trial_timeout': 20,  # Give this many seconds for a trial.
        'reset_conditions': [],  # Defines reset modes + positions for
                                 # trial and auxiliary arms.
        'frequency': 20,

        }
except ImportError as e:
    AGENT_ROS = {}
    LOGGER.debug('No ROS enabled: %s', e)
except rospkg.common.ResourceNotFound as e:
    AGENT_ROS = {}
    LOGGER.debug('No gps_agent_pkg: %s', e)
