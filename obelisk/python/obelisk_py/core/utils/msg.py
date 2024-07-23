from typing import Union

import numpy as np
from obelisk_std_msgs.msg import FloatMultiArray, UInt8MultiArray
from std_msgs.msg import MultiArrayDimension, MultiArrayLayout

MultiArray = Union[FloatMultiArray, UInt8MultiArray]


def multiarray_to_np(msg: MultiArray) -> np.ndarray:
    """Convert MultiArray message to numpy array.

    multiarray(i,j,k,...) = data[data_offset + dim_stride[0]*i + dim_stride[1]*j + dim_stride[2]*k + ...]

    Parameters:
        msg: MultiArray message.

    Returns:
        Numpy array.

    Raises:
        ValueError: If the message has an unsupported type.
    """
    if isinstance(msg, UInt8MultiArray):
        dtype = np.uint8
    elif isinstance(msg, FloatMultiArray):
        dtype = np.float64
    else:
        raise ValueError(f"Unsupported message type: {type(msg)}")

    flat_data = np.array(msg.data, dtype=dtype)
    if msg.layout.data_offset:
        flat_data = flat_data[msg.layout.data_offset :]

    shape = [dim.size for dim in msg.layout.dim]
    strides = [dim.stride * flat_data.itemsize for dim in msg.layout.dim]
    return np.lib.stride_tricks.as_strided(flat_data, shape=shape, strides=strides)


def np_to_multiarray(arr: np.ndarray) -> MultiArray:
    """Convert numpy array to MultiArray message.

    multiarray(i,j,k,...) = data[data_offset + dim_stride[0]*i + dim_stride[1]*j + dim_stride[2]*k + ...]

    Parameters:
        arr: Numpy array.

    Returns:
        MultiArray message.

    Raises:
        ValueError: If the array has an unsupported dtype.
    """
    if arr.dtype == np.uint8:
        msg = UInt8MultiArray()
    elif arr.dtype == np.float64:
        msg = FloatMultiArray()
    else:
        raise ValueError(f"Unsupported dtype: {arr.dtype}")

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
