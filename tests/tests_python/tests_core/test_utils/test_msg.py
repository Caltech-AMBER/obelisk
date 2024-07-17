import numpy as np
from obelisk_std_msgs.msg import FloatMultiArray

from obelisk_py.core.utils.msg import multiarray_to_np, np_to_multiarray


def test_conversion() -> None:
    """Test the conversion between numpy array and FloatMultiArray."""
    arr = np.random.rand(3, 4, 5)
    msg = np_to_multiarray(arr)
    assert isinstance(msg, FloatMultiArray)
    arr_recovered = multiarray_to_np(msg)
    assert np.allclose(arr, arr_recovered)
