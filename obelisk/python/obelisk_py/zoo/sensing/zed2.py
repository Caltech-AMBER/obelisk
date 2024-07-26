import os
from threading import Lock
from typing import Callable, Dict

import numpy as np
from ament_index_python.packages import get_package_share_directory
from obelisk_sensor_msgs.msg import ObkImage
from pyzed.sl import sl
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from ruamel.yaml import YAML

from obelisk_py.core.sensing import ObeliskSensor
from obelisk_py.core.utils.msg import np_to_multiarray


class ObeliskZed2Sensors(ObeliskSensor):
    """The ZED2 sensor node.

    This node can support multiple ZED2 cameras at once. All ZED cameras publish their images at the same time, but poll
    their images asynchronously.
    """

    def __init__(self, node_name: str) -> None:
        """Initialize the ZED2 sensor node.

        Raises:
            ValueError: If no serial numbers are provided for ZED2 cameras.
            ValueError: If params_path is not an absolute path and params_path_pkg is not provided.
            ValueError: If the parameters file is not a YAML file.
        """
        super().__init__(node_name)

        # set parameters for ZED cameras
        self.declare_parameter("params_path_pkg", "")
        params_path_pkg = self.get_parameter("params_path_pkg").get_parameter_value().string_value
        _params_path = self.get_parameter("params_path").get_parameter_value().string_value

        if _params_path.startswith("/"):
            params_path = _params_path
        elif not params_path_pkg:
            err_msg = "No package provided for ZED2 camera parameters!"
            self.get_logger().error(err_msg)
            raise ValueError(err_msg)
        else:
            share_directory = get_package_share_directory(params_path_pkg)
            params_path = os.path.join(share_directory, _params_path)
        self._set_camera_params(params_path)

        # register publisher for ZED2 cameras
        self.register_obk_publisher(
            "pub_img_setting",
            key="pub_img",
            msg_type=ObkImage,
        )

    def _set_camera_params(self, params_path: str) -> None:
        """Set the parameters for the ZED2 cameras from a configuration yaml file.

        The yaml file should load a dictionary of the following format:
        ```
        0:
            serial_number: "12345678"  # [REQUIRED] serial number, 8 digits
            resolution: "VGA"  # must be one of ["VGA", "720", "1080", "2K"]
            fps: 100  # in [15, 30, 60, 100] + compatible w/res (https://www.stereolabs.com/docs/video/camera-controls)
            depth: False  # whether to enable depth sensing
            side: "left"  # must be one of ["left", "right"], can only grab one image at a time
        1:
            ...
        ```

        Args:
            params_path: The path to the camera parameters.

        Raises:
            ValueError: If the parameters file is not a YAML file.
            FileNotFoundError: If the configuration file cannot be loaded.
            ValueError: If the configuration file is not in the correct format.
        """
        # if params_path doesn't end in yaml or yml, raise an error
        if not params_path.endswith(".yaml") and not params_path.endswith(".yml"):
            err_msg = (
                f"Invalid parameters file: {params_path}! "
                "ObeliskZed2Sensors expects a YAML file with extension .yaml or .yml."
            )
            self.get_logger().error(err_msg)
            raise ValueError(err_msg)

        # loading the cam param dicts
        yaml = YAML(typ="safe")
        try:
            cam_param_dicts = yaml.load(params_path)
        except Exception as e:
            raise FileNotFoundError(f"Could not load a configuration file at {params_path}!") from e

        # verifying the format of the cam param dicts
        for cam_index, cam_param_dict in cam_param_dicts.items():
            if not isinstance(cam_index, int):
                err_msg = f"Invalid camera name {cam_index} type in {params_path}! Must be an int."
                self.get_logger().error(err_msg)
                raise ValueError(err_msg)

            # each value must be a dict
            if not isinstance(cam_param_dict, dict):
                err_msg = f"Invalid camera parameters for camera {cam_index} in {params_path}!"
                self.get_logger().error(err_msg)
                raise ValueError(err_msg)

            # serial number is required and has certain format
            sn_len = 8  # length of serial number
            if "serial_number" not in cam_param_dict:
                err_msg = f"Serial number not provided for camera {cam_index} in {params_path}!"
                self.get_logger().error(err_msg)
                raise ValueError(err_msg)
            elif not cam_param_dict["serial_number"].isdigit() or len(cam_param_dict["serial_number"]) != sn_len:
                err_msg = f"Invalid serial number for camera {cam_index} in {params_path}! Must be {sn_len} digits."
                self.get_logger().error(err_msg)
                raise ValueError(err_msg)
            cam_param_dict["serial_number"] = int(cam_param_dict["serial_number"])

            # check resolution
            if "resolution" not in cam_param_dict:
                cam_param_dict["resolution"] = "VGA"
            elif cam_param_dict["resolution"].lower() not in ["vga", "720", "1080", "2k"]:
                err_msg = f"Invalid resolution for camera {cam_index}! Must be one of 'VGA', '720', '1080', '2K'."
                self.get_logger().error(err_msg)
                raise ValueError(err_msg)

            if cam_param_dict["resolution"].lower() == "vga":
                cam_param_dict["resolution"] = sl.RESOLUTION.VGA
                self.H, self.W = 376, 672
            elif cam_param_dict["resolution"].lower() == "720":
                cam_param_dict["resolution"] = sl.RESOLUTION.HD720
                self.H, self.W = 720, 1280
            elif cam_param_dict["resolution"].lower() == "1080":
                cam_param_dict["resolution"] = sl.RESOLUTION.HD1080
                self.H, self.W = 1080, 1920
            elif cam_param_dict["resolution"].lower() == "2k":
                cam_param_dict["resolution"] = sl.RESOLUTION.HD2K
                self.H, self.W = 1242, 2208
            else:
                err_msg = (
                    f"Invalid resolution for camera {cam_index} in {params_path}! "
                    "Must be one of 'VGA', '720', '1080', '2K'."
                )
                self.get_logger().error(err_msg)
                raise ValueError(err_msg)

            # check fps
            if "fps" not in cam_param_dict:
                if cam_param_dict["resolution"] == "VGA":
                    cam_param_dict["fps"] = 100
                elif cam_param_dict["resolution"] == "720":
                    cam_param_dict["fps"] = 60
                elif cam_param_dict["resolution"] == "1080":
                    cam_param_dict["fps"] = 30
                elif cam_param_dict["resolution"] == "2K":
                    cam_param_dict["fps"] = 15
                else:
                    err_msg = (
                        f"Invalid resolution for camera {cam_index} in {params_path}! "
                        "Must be one of 'VGA', '720', '1080', '2K'."
                    )
                    self.get_logger().error(err_msg)
                    raise ValueError(err_msg)
            elif cam_param_dict["fps"] not in [15, 30, 60, 100]:
                err_msg = f"Invalid fps for camera {cam_index} in {params_path}! Must be one of 15, 30, 60, 100."
                self.get_logger().error(err_msg)
                raise ValueError(err_msg)
            elif cam_param_dict["resolution"] == "720":
                if cam_param_dict["fps"] not in [15, 30, 60]:
                    err_msg = (
                        f"Invalid fps for camera {cam_index} in {params_path} for 720 resolution! "
                        "Must be one of 15, 30, 60."
                    )
                    self.get_logger().error(err_msg)
                    raise ValueError(err_msg)
            elif cam_param_dict["resolution"] == "1080":
                if cam_param_dict["fps"] not in [15, 30]:
                    err_msg = (
                        f"Invalid fps for camera {cam_index} in {params_path} for 1080 resolution! "
                        "Must be one of 15, 30."
                    )
                    self.get_logger().error(err_msg)
                    raise ValueError(err_msg)
            elif cam_param_dict["resolution"] == "2K":
                if cam_param_dict["fps"] != 15:  # noqa: PLR2004
                    err_msg = f"Invalid fps forcamera {cam_index} in {params_path} for 2K resolution! Must be 15."
                    self.get_logger().error(err_msg)
                    raise ValueError(err_msg)

            # check depth
            if "depth" not in cam_param_dict:
                cam_param_dict["depth"] = False
            elif not isinstance(cam_param_dict["depth"], bool):
                err_msg = f"Invalid depth for camera {cam_index} in {params_path}! Must be a boolean."
                self.get_logger().error(err_msg)
                raise ValueError(err_msg)

            # check side
            if "side" not in cam_param_dict:
                cam_param_dict["side"] = "left"
            elif cam_param_dict["side"].lower() not in ["left", "right"]:
                err_msg = f"Invalid side for camera {cam_index} in {params_path}! Must be one of 'left', 'right'."
                self.get_logger().error(err_msg)
                raise ValueError(err_msg)

        # in this node, resolution, fps, and depth must match for all cameras
        resolutions = set([cam_param_dict["resolution"] for cam_param_dict in cam_param_dicts.values()])
        fpss = set([cam_param_dict["fps"] for cam_param_dict in cam_param_dicts.values()])
        depths = set([cam_param_dict["depth"] for cam_param_dict in cam_param_dicts.values()])

        if len(resolutions) > 1:
            err_msg = "All cameras must have the same resolution!"
            self.get_logger().error(err_msg)
            raise ValueError(err_msg)
        if len(fpss) > 1:
            err_msg = "All cameras must have the same fps!"
            self.get_logger().error(err_msg)
            raise ValueError(err_msg)
        if len(depths) > 1:
            err_msg = "All cameras must have the same depth setting!"
            self.get_logger().error(err_msg)
            raise ValueError(err_msg)

        self.resolution = resolutions.pop()
        self.fps = fpss.pop()
        self.depth = depths.pop()

        # set the camera param dicts
        self.cam_param_dicts = cam_param_dicts

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the ZED2 sensor node.

        Args:
            state: The lifecycle state.

        Returns:
            The transition callback return.
        """
        super().on_configure(state)

        # initializing cameras
        self.cams = {}
        self.cam_polled = {}
        self.cam_images = {}  # raw ZED image object
        self.cam_rt_params = {}
        self.frames = {}  # numpy array with image data
        for cam_index, cam_param_dict in self.cam_param_dicts.items():
            serial_number = cam_param_dict["serial_number"]

            # transients
            self.cam_polled[cam_index] = False  # whether a camera has been polled during a cycle
            self.cam_images[cam_index] = sl.Mat()  # image buffers for each camera

            # runtime parameters
            rtp = sl.RuntimeParameters()
            rtp.enable_depth = self.depth
            self.cam_rt_params[cam_index] = rtp

            # camera initialization
            # see depth mode recommendation here: https://www.stereolabs.com/docs/depth-sensing/depth-settings
            cam = sl.Camera()
            init_params = sl.InitParameters()
            init_params.set_from_serial_number(serial_number)
            init_params.camera_resolution = cam_param_dict["resolution"]
            init_params.camera_fps = self.fps
            init_params.depth_mode = sl.DEPTH_MODE.NONE if not self.depth else sl.DEPTH_MODE.ULTRA
            init_params.coordinate_units = sl.UNIT.METER  # depth in meters (mm by default)
            init_params.depth_minimum_distance = 0.3  # min depth in meters for zed 2, TODO(ahl): is this good?
            if self.depth:
                if cam_param_dict["side"] == "right":
                    init_params.enable_right_side_measure = True
            init_params.camera_image_flip = sl.FLIP_MODE.OFF  # don't automatically flip the camera based on orientation
            err = cam.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f"Failed to open camera with serial number {serial_number}!")
                raise RuntimeError(f"Failed to open camera with serial number {serial_number}!")
            self.get_logger().info(f"Camera with serial number {serial_number} initialized!")
            self.cams[cam_index] = cam

        # lock for synchronization
        self.lock = Lock()

        for cam_index, cam_param_dict in self.cam_param_dicts.items():

            def create_callback(cam_index: int = cam_index, cam_param_dict: Dict = cam_param_dict) -> Callable:
                """Factory function to create the callback for each camera.

                This resolves the complaint that the loop variable is not captured in the closure.
                """

                def callback() -> None:
                    """Callback to grab and retrieve images from the ZED2 cameras."""
                    with self.lock:
                        # cam settings
                        cam = self.cams[cam_index]
                        rtps = self.cam_rt_params[cam_index]
                        cam_image = self.cam_images[cam_index]
                        side = cam_param_dict["side"]

                        # grabbing the frame
                        if cam.grab(rtps) == sl.ERROR_CODE.SUCCESS:
                            if self.depth:
                                if side == "left":
                                    cam.retrieve_image(cam_image, sl.MEASURE.DEPTH)  # left by default
                                else:
                                    cam.retrieve_image(cam_image, sl.MEASURE.DEPTH_RIGHT)
                            elif side == "left":
                                cam.retrieve_image(cam_image, sl.VIEW.LEFT)
                            else:
                                cam.retrieve_image(cam_image, sl.VIEW.RIGHT)

                        # loading the frame into numpy
                        self.frames[cam_index] = cam_image.get_data()
                        self.cam_polled[cam_index] = True
                        self.check_and_publish_images()

                return callback

            timer_period_sec = 1.0 / self.fps
            timer = self.create_timer(
                timer_period_sec,
                callback=create_callback(),  # Create the callback with current loop variables
                callback_group=MutuallyExclusiveCallbackGroup(),  # type: ignore
                # autostart=False,  # TODO: feature only available after humble
            )
            timer.cancel()  # initially, the timer should be deactivated, TODO: remove if distro upgraded
            self.obk_timers[f"timer_{cam_index}"] = timer

        return TransitionCallbackReturn.SUCCESS

    def check_and_publish_images(self) -> None:
        """Publish the images from the ZED2 cameras."""
        if all(self.cam_polled.values()):
            msg = ObkImage()
            _y = []

            # sort and stack the cam frames by cam_index
            for cam_index in sorted(self.frames.keys()):
                _y.append(self.frames[cam_index])
            y = np.stack(_y, axis=0)

            if self.depth:
                # https://www.stereolabs.com/docs/depth-sensing/using-depth#displaying-depth-image
                msg.y = np_to_multiarray(y)  # (B, H, W, 1), UInt8MultiArray
                if y.shape[-3:] != (self.H, self.W, 1):
                    err_msg = f"Depth images have shape {y.shape} but expected shape {(self.H, self.W, 1)}!"
                    self.get_logger().error(err_msg)
                    raise ValueError(err_msg)
            else:
                # change color convention BGRA (ZED) -> RGB
                # https://www.stereolabs.com/docs/api/python/classpyzed_1_1sl_1_1VIEW.html
                y = y[..., :3]  # get rid of alpha. TODO(ahl): allow alpha
                y = y[..., ::-1].copy()  # (B, H, W, 3), reverse the order of the channels, copy to fix negative stride
                msg.y = np_to_multiarray(y)  # UInt8MultiArray
                if y.shape[-3:] != (self.H, self.W, 3):
                    err_msg = f"Color images have shape {y.shape} but expected shape {(self.H, self.W, 3)}!"
                    self.get_logger().error(err_msg)
                    raise ValueError(err_msg)

            # publish + reset polled list
            self.obk_publishers["pub_img"].publish(msg)
            self.cam_polled = {cam_index: False for cam_index in self.cam_polled.keys()}
