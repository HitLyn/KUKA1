'''this file defines Utilities for the agent'''

import numpy as np


def msg_to_sample(ros_msg, agent):
    """
    Convert a SampleResult ROS message into a Sample Python object.
    """
    sample = Sample(agent) # initiallize a sample object
    for sensor in ros_msg.sensor_data:
        sensor_id = sensor.data_type
        shape = np.array(sensor.shape)
        data = np.array(sensor.data).reshape(shape)
        sample.set(sensor_id, data) # save data to Dictionary
    return sample

def action_to_msg(action, noise):
    """
    Convert a policy object to a ROS ControllerParams message.
    """
    msg = ControllerParams()
    if isinstance(policy, LinearGaussianPolicy):
        msg.controller_to_execute = LIN_GAUSS_CONTROLLER
        msg.lingauss = LinGaussParams()
        msg.lingauss.dX = policy.dX
        msg.lingauss.dU = policy.dU
        msg.lingauss.K_t = policy.K.reshape(policy.T * policy.dX * policy.dU).tolist()
        msg.lingauss.k_t = policy.fold_k(noise).reshape(policy.T * policy.dU).tolist()
    else:
        raise NotImplementedError("Caffe not imported or Unknown policy object: %s" % policy)
    return msg

def generate_noise(T, dU, hyperparams):
    """
    Generate a T x dU gaussian-distributed noise vector. This will
    approximately have mean 0 and variance 1, ignoring smoothing.

    Args:
        T: Number of time steps.
        dU: Dimensionality of actions.
    Hyperparams:
        smooth: Whether or not to perform smoothing of noise.
        var : If smooth=True, applies a Gaussian filter with this
            variance.
        renorm : If smooth=True, renormalizes data to have variance 1
            after smoothing.
    """
    smooth, var = hyperparams['smooth_noise'], hyperparams['smooth_noise_var']
    renorm = hyperparams['smooth_noise_renormalize']
    noise = np.random.randn(T, dU)
    if smooth:
        # Smooth noise. This violates the controller assumption, but
        # might produce smoother motions.
        for i in range(dU):
            noise[:, i] = sp_ndimage.filters.gaussian_filter(noise[:, i], var)
        if renorm:
            variance = np.var(noise, axis=0)
            noise = noise / np.sqrt(variance)
    return noise
#end of generate_noise method

class Sample(object):
    """Class that handle the representation of a trajectory and store a single trajectory ."""
    def __init__(self, agent):
        self.agent = agent

        # get param from agent: all time-step , dimension of state and action
        self.T = agent.T
        self.dX = agent.dX
        self.dU = agent.dU

        # Dictionary containing the sample data from various sensors.
        self._data = {}

        # variables for saving of a trajectory
        self._X = np.empty((self.T, self.dX))
        self._X.fill(np.nan)
    #end of __init__ method

    def set(self, sensor_name, sensor_data, t=None):
        """ get data from sensor, save as self._data file.
            arg:
                t   time-step of a trajectroy
        """
        if t is None:
            # Invalidate each existing variables = nan
            self._data[sensor_name] = sensor_data
            self._X.fill(np.nan)  # Invalidate existing X.
        else:
            if sensor_name not in self._data:
                self._data[sensor_name] = np.empty((self.T,) + sensor_data.shape)
                self._data[sensor_name].fill(np.nan)
            self._data[sensor_name][t, :] = sensor_data
            self._X[t, :].fill(np.nan)
    #end of set method

    def get(self, sensor_name, t=None):
        """ Get trajectory data for a particular sensor. """
        if t is None:
            data_tmp = self._data[sensor_name] # all the data
        else:
            data_tmp = self._data[sensor_name][t, :] # data at t step
        return data_tmp
    #end of get method

    def get_X(self, t=None):
        """ Get the state. Put it together if not precomputed. """
        if t is None:
            X_tmp = self._X # all the state
        else:
            X_tmp = self._X[t, :] # state at t step

        if np.any(np.isnan(X_tmp)):
            for data_type in self._data:
                if data_type not in self.agent.x_data_types:
                    continue
                data = (self._data[data_type] if t is None
                        else self._data[data_type][t, :])
                #self.agent.pack_data_x(X_tmp, data, data_types=[data_type])
                pack_data_x(X_tmp, data, data_types=[data_type]) #yuchen
        return X_tmp
    #end of get_X method
    def get_U(self, t=None):
        """ Get the action. """
        if t is None:
            U_tmp = self._data[ACTION]
        else:
            U_tmp = self._data[ACTION][t, :]
        return U_tmp
    #end of get_U method

    def pack_data_x(self, existing_mat, data_to_insert, data_types, axes=None):
        """
        Update the state matrix with new data.
        Args:
            existing_mat: Current state matrix.
            data_to_insert: New data to insert into the existing matrix.
            data_types: Name of the sensors to insert data for.
            axes: Which axes to insert data. Defaults to the last axes.
        """
        num_sensor = len(data_types) # = 4
        if axes is None:
            # If axes not specified, assume indexing on last dimensions.
            axes = list(range(-1, -num_sensor - 1, -1))
        else:
            # Make sure number of sensors and axes are consistent.
            if num_sensor != len(axes):
                raise ValueError(
                    'Length of sensors (%d) must equal length of axes (%d)',
                    num_sensor, len(axes)
                )
        # Shape checks.
        insert_shape = list(existing_mat.shape)
        for i in range(num_sensor):
            # Make sure to slice along X.
            if existing_mat.shape[axes[i]] != self.dX:
                raise ValueError('Axes must be along an dX=%d dimensional axis', self.dX)
            insert_shape[axes[i]] = len(self.agent._x_data_idx[data_types[i]])
        if tuple(insert_shape) != data_to_insert.shape:
            raise ValueError('Data has shape %s. Expected %s', data_to_insert.shape, tuple(insert_shape))

        # Actually perform the slice.
        index = [slice(None) for _ in range(len(existing_mat.shape))]
        for i in range(num_sensor):
            index[axes[i]] = slice(self.agent._x_data_idx[data_types[i]][0],
                                   self.agent._x_data_idx[data_types[i]][-1] + 1)
        existing_mat[index] = data_to_insert
    #end of pack_data_x method

    def unpack_data_x(self, existing_mat, data_types, axes=None):
        """
        Returns the requested data from the state matrix.
        Args:
            existing_mat: State matrix to unpack from.
            data_types: Names of the sensor to unpack.
            axes: Which axes to unpack along. Defaults to the last axes.
        """
        num_sensor = len(data_types)
        if axes is None:
            # If axes not specified, assume indexing on last dimensions.
            axes = list(range(-1, -num_sensor - 1, -1))
        else:
            # Make sure number of sensors and axes are consistent.
            if num_sensor != len(axes):
                raise ValueError(
                    'Length of sensors (%d) must equal length of axes (%d)',
                    num_sensor, len(axes)
                )

        # Shape checks.
        for i in range(num_sensor):
            # Make sure to slice along X.
            if existing_mat.shape[axes[i]] != self.dX:
                raise ValueError('Axes must be along an dX=%d dimensional axis',
                                 self.dX)

        # Actually perform the slice.
        index = [slice(None) for _ in range(len(existing_mat.shape))]
        for i in range(num_sensor):
            index[axes[i]] = slice(self.agent._x_data_idx[data_types[i]][0],
                                   self.agent._x_data_idx[data_types[i]][-1] + 1)
        return existing_mat[index]
    #end of unpack_data_x method
