import asyncio
from mavsdk import telemetry
from mavsdk import System
from mavsdk.offboard import (OffboardError, VelocityBodyYawspeed)


async def connect_drone(drone, system_address="serial:///dev/ttyTHS1:57600", retries=3, delay=5):
    # Connect to the drone function - remove the logger
    attempt = 0
    while attempt < retries:
        message = (f"Connect Drone:\n"
                   f"                  Attempt {attempt + 1}/{retries}")
        print(message)
        try:
            await drone.connect(system_address=system_address)
            message = (f"                  Waiting for drone to connect at {system_address}...")
            print(message)
            connection_task = asyncio.create_task(drone.core.connection_state().__anext__())
            state = await asyncio.wait_for(connection_task, delay)
            if state.is_connected:
                message = "                  Connected to drone!"
                print(message)
                # Fetch and log system identification information
                identification = await drone.info.get_identification()
                message = (f"                  Drone Identification: Hardware UID - {identification.hardware_uid}\n"
                           f"                                        Legacy UID   - {identification.legacy_uid}")
                print(message)
                message = "                  Subscribing to odometry data..."
                print(message)
                async for odometry in drone.telemetry.odometry():
                    message = (f"                  Odometry data received!\n"
                               f"                  Time Stamp: {odometry.time_usec}")
                    print(message)
                    break
                return True
            else:
                message = "-- warning: Connection state indicates not connected."
                print(message)
        except asyncio.TimeoutError:
            message = "-- warning: Connection to drone timed out."
            print(message)
        except Exception as e:
            message = f"-- error: Failed during connection or information retrieval: {e}, Type: {e.__class__.__name__}"
            print(message)
        finally:
            if attempt + 1 < retries:
                await asyncio.sleep(delay)
        attempt += 1
    return False


async def arm_drone(drone):
    """
    Asynchronously arms the drone, ensuring it's safe to fly, and provides feedback from the flight controller.
    Args:
        drone: An instance of a drone control object that supports async operations.
    Returns:
        bool: True if the drone was successfully armed within the timeout period, False otherwise.
    Raises:
        Exception: Propagates any unexpected exceptions from the drone control library.
    """
    message = ("Arming:\n                  "
               "Attempting to arm the drone...")
    print(message)
    try:
        await drone.action.arm()
        message = "                  Drone armed successfully!"
        print(message)
        timeout = 10  # seconds
        start_time = asyncio.get_event_loop().time()
        async for is_armed in drone.telemetry.armed():
            if is_armed:
                message = ("                  Confirmation: Drone is armed")
                print(message)
                return True
            elif (asyncio.get_event_loop().time() - start_time) > timeout:
                message = "-- warning.Timeout: Drone did not arm within the expected time"
                print(message)
                return False
            await asyncio.sleep(0.1)  # Brief pause to yield control
    except Exception as e:
        message = f"-- error: Failed to arm the drone: {e}"
        print(message)
        return False
    # Safety check in case of unexpected flow
    message = "-- warning: Unexpected condition: Reached end of arm_drone function without arming confirmation."
    print(message)
    return False


async def disarm_drone(drone):
    """
    Asynchronously disarms the drone, ensuring it's safe to handle, and provides feedback from the flight controller.
    Args:
        drone: An instance of a drone control object that supports async operations.
    Returns:
        bool: True if the drone was successfully disarmed, False otherwise.
    Raises:
        Exception: Propagates any unexpected exceptions from the drone control library.
    """
    message = ("Disarming:\n                  "
               "Attempting to disarm the drone")
    print(message)
    try:
        await drone.action.disarm()
        message = "                  Drone disarmed successfully"
        print(message)
        # Optionally, confirm the drone is disarmed by checking the armed state
        timeout = 10  # seconds
        start_time = asyncio.get_event_loop().time()
        async for is_armed in drone.telemetry.armed():
            if not is_armed:
                message = ("                  Confirmation: Drone is disarmed")
                print(message)
                return True
            elif (asyncio.get_event_loop().time() - start_time) > timeout:
                message = "-- warning.Timeout: Drone did not disarm within the expected time"
                print(message)
                return False
            await asyncio.sleep(0.1)  # Brief pause to yield control
    except Exception as e:
        message = f"-- error: Failed to disarm the drone: {e}"
        print(message)
        return False
    # Safety check in case of unexpected flow
    message = "-- warning: Unexpected condition: Reached end of disarm_drone function without disarming confirmation."
    print(message)
    return False




async def set_initial_setpoint(drone):
    """
    Asynchronously sets the initial setpoint for the drone and checks if the setpoint has been acknowledged
    by verifying the drone's velocity is as expected.
    Args:
        drone: An instance of a drone control object that supports async operations, specifically for offboard control.
    Returns:
        bool: True if the initial setpoint was successfully set and confirmed, False otherwise.
    Raises:
        Exception: Propagates specific exceptions related to setting the setpoint, if known and applicable.
    """
    message = ("Initial Setpoint:\n"
               "                  Setting initial setpoint...")
    print(message)
    try:
        await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))
        message = ("                  Initial setpoint set!\n"
                   "                  verifying acknowledgment...")
        print(message)

    except Exception as e:
        message = f"-- error: Failed to set initial setpoint: {e}"
        print(message)
        await drone.action.hold()
        return False


async def enable_offboard_mode(drone):
    """Enables offboard mode with error handling."""
    message = ("Enable Offboard:\n"
               "                  Enabling Offboard Mode...")
    print(message)
    try:
        await drone.offboard.start()
        message = ("                  Offboard Mode is enabled!")
        print(message)
    except OffboardError as error:
        message = f"-- error: Starting offboard mode failed with error: {error}"
        print(message)
        # Ensure that stopping offboard mode and disarming are attempted even in case of failure.
        try:
            await drone.offboard.stop()
            message = "                  Stopping Offboard Mode due to error"
            print(message)
        except Exception as stop_error:
            message = f"-- error: Failed to stop offboard mode: {stop_error}"
            print(message)
        try:
            await drone.action.disarm()
            message = "                  Disarming due to offboard mode error"
            print(message)
        except Exception as disarm_error:
            message = f"-- error: Failed to disarm the drone: {disarm_error}"
            print(message)
        return False


async def stop_offboard_mode(drone):
    """Stops offboard mode with comprehensive error handling and operational feedback."""
    message = ("Stop Offboard:\n"
               "                  Stopping Offboard Mode...")
    print(message)
    try:
        await drone.offboard.stop()
        message = "                  Offboard Mode Successfully Stopped!"
        print(message)
    except OffboardError as error:
        # Improved error feedback with more detailed message.
        message = f"-- error: Stopping offboard mode failed with error: {error}"
        print(message)
        # Optionally, disarm the drone for safety, depending on your use case.
        try:
            message = "                  Attempting to Disarm the Drone for Safety"
            print(message)
            await drone.action.disarm()
            message = "                  Drone Successfully Disarmed"
            print(message)
        except Exception as disarm_error:
            message = f"-- error: Failed to disarm the drone: {disarm_error}"
            print(message)
        # Raising the error to allow the calling context to handle or log the failure.
        raise
    except Exception as general_error:
        # Catch-all for any other exceptions not explicitly handled above.
        message = f"-- error: An unexpected error occurred while stopping offboard mode: {general_error}"
        print(message)
        # Consider whether to disarm the drone in this case as well.
        raise


async def takeoff_velocity(drone):
    """
    This function takes off the drone at a specified velocity.
    :param drone: the drone object
    :return: True if successful
    """
    await arm_drone(drone)

    await set_initial_setpoint(drone)

    await enable_offboard_mode(drone)

    print("-- only climb")
    await drone.offboard.set_velocity_body(
        VelocityBodyYawspeed(0.0, 0.0, -1.0, 0))
    await asyncio.sleep(7)

    print("stop the takeoff")
    await drone.action.hold()

    return True


async def confirm_airborne(drone: System):
    """Wait for the drone to be confirmed as airborne."""
    async for state in drone.telemetry.landed_state():
        if state == telemetry.LandedState.IN_AIR:
            message = "                  Airborne: Drone is airborne!"
            print(message)
            return True
        await asyncio.sleep(1)
    message = "-- warning: Drone is still on the ground."
    print(message)
    return False









async def control_velocity(drone, V_y: float = 0.0, V_x: float = 0.0, V_z: float = 0.0, Yaw_deg: float = 0.0):

        body_velocity = VelocityBodyYawspeed(V_y, V_x, V_z, Yaw_deg)  # Set correct velocity values
        await drone.offboard.set_velocity_body(body_velocity)

