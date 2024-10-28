==================
Joystick Interface
==================

Many robots need some form of human interface to either move around or change states. We provide a light weight interface to the `joy ROS package <https://index.ros.org/p/joy/>`_.
Specifically we allow users to launch the joy node from the obelisk launch file.

Example Launch Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. code-block:: yaml

    joystick:
        on: True
        pub_topic: /obelisk/dummy/joy
        sub_topic: /obelisk/dummy/joy_feedback
        device_id: 0
        device_name: "my_device_name"
        deadzone: 0.05
        autorepeat_rate: 20.0
        sticky_buttons: False
        coalesce_interval_ms: 1


``device_id`` through ``coalesce_interval_ms`` are passed directly to the arguments in the ros package, and therefore you should refere to the joy package documentation for their meaning.

The settings are only parsed in ``on`` is set to ``True``. The ``pub_topic`` is the name of the topic where the joystick state is published to.
The ``sub_topic`` is the name of the topic where the joystick feedback, like rumble, is published.

Connecting the Joystick
^^^^^^^^^^^^^^^^^^^^^^^
To use the joystick, joy must be able to see and access the joystick. You can connect through one of two ways: (a) USB, (b) bluetooth.

Connecting via bluetooth is decidedly more complicated than through USB. We are still working on a guide for reliable bluetooth communication,
and therefore we will focus on the USB connection here.

USB
+++
You can verify if the controlled is connected to your computer by using `https://hardwaretester.com/gamepad <https://hardwaretester.com/gamepad>`_.
This will only verify that nominally the computer can see the joystick, not that its can be seen within a docker container or that joy can see it. This is a good first step.

We can also verify the controller connection by using ``evtest``::

    sudo apt-get update
    sudo apt-get install evtest
    sudo evtest /dev/input/eventX

where eventX is the the correct event number. You can check the connected inputs by using ``ls /dev/input``.

If you use evtest on the correct event then whenever the joystick is used something will be printed to the screen.

You can run ``ros2 run joy joy_enumerate_devices`` to see what devices are found. If no devices are found and you verified with evtest that the joystick is connected then
you may need to change the permissions on the joystick::

    sudo chmod 666 /dev/input/eventX

where once again you change the X with the correct number. Now the joystick should connect without issue.
