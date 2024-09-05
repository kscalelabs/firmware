IMU
==========================

This section provides an overview of the IMU (Inertial Measurement Unit) interface used in the robot, highlighting its design and how it integrates with the robot's control system.

The IMU is a critical component for providing orientation and motion data, which is essential for the robot's navigation and control.

IMU Interface
-------------

The IMU interface as defined in the :py:mod:`firmware.imu` module. This interface provides methods for calibrating, updating, and retrieving IMU data.

The :py:class:`IMUInterface` class defines the following key methods:

- `calibrate_yaw() -> None`: Calibrates the yaw of the IMU.
- `step(dt: float) -> list[Any]`: Updates the IMU data based on the time step `dt`.
- `get_measurement() -> list[list[float]]`: Retrieves the current IMU measurements, including orientation and gyroscope data.

Important Notes
--------------

Before using the IMU, it should be stepped and calibrated using the `step()` and `calibrate_yaw()` methods.
For example, you might run the following code to calibrate the IMU:

.. code-block:: python

    import time
    from firmware.imu.imu import IMUInterface

    imu = IMUInterface()
    dt = 0.1
    for _ in range(10):
        imu.step(dt)
        imu.calibrate_yaw()
        time.sleep(dt)

For detailed API documentation, refer to the following module:

- :py:mod:`firmware.imu` module: Contains the implementation for the IMU interface.
