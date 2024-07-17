import numpy as np
from obelisk_std_msgs.msg import FloatMultiArray
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout


def multiarray_to_np(msg: FloatMultiArray) -> np.ndarray:
    """Convert FloatMultiArray message to numpy array.

    multiarray(i,j,k,...) = data[data_offset + dim_stride[0]*i + dim_stride[1]*j + dim_stride[2]*k + ...]

    Parameters:
        msg: FloatMultiArray message.

    Returns:
        Numpy array.
    """
    flat_data = np.array(msg.data, dtype=np.float64)
    if msg.layout.data_offset:
        flat_data = flat_data[msg.layout.data_offset :]

    shape = [dim.size for dim in msg.layout.dim]
    strides = [dim.stride * flat_data.itemsize for dim in msg.layout.dim]
    return np.lib.stride_tricks.as_strided(flat_data, shape=shape, strides=strides)


def np_to_multiarray(arr: np.ndarray) -> FloatMultiArray:
    """Convert numpy array to FloatMultiArray message.

    multiarray(i,j,k,...) = data[data_offset + dim_stride[0]*i + dim_stride[1]*j + dim_stride[2]*k + ...]

    Parameters:
        arr: Numpy array.

    Returns:
        FloatMultiArray message.
    """
    msg = FloatMultiArray()
    flat_data = arr.ravel()
    msg.data = flat_data.tolist()
    msg.layout = MultiArrayLayout()

    dimensions = []
    for i, (size, stride) in enumerate(zip(arr.shape, arr.strides)):
        dim = MultiArrayDimension()
        dim.size = size
        dim.stride = stride // arr.itemsize  # convert byte strides to element strides
        dim.label = f"dim_{i}"
        dimensions.append(dim)

    msg.layout.dim = dimensions
    msg.layout.data_offset = 0

    return msg
