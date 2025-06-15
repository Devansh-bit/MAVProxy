import os
from datetime import datetime
from typing import Callable, List, Literal, TypeVar

T = TypeVar('T')
BASE_DIR = os.path.dirname(__file__)

from langchain_core.tools import tool
from pymavlink import mavutil
from MAVProxy.mavproxy import MPState

def init(mpstate: MPState) -> List[Callable[[T], T]]:
    """Injects MPState in tools to be registered

    Args:
        mpstate: the MPState object

    Returns:
        List of tools with MPState injected
    """

    @tool
    def send_mavproxy_command(command: str) -> str:
        """Sends a command to MAVProxy as a string. Must be a proper commandformatted properly.

        Args:
            command: the full command as a string
        """
        #TODO: implement
        print("sending mavproxy command", command)
        return "Success"


    @tool
    def get_current_datetime() -> str:
        """Gets the current datetime
        Returns:
            Current datetime as string in format: Saturday, June 24, 2023 6:14:14 PM
        """
        print("Getting current datetime")
        return datetime.now().strftime("%A, %B %d, %Y %I:%M:%S %p")

    @tool
    def get_vehicle_type() -> str:
        """Return the vehicle type from the latest MAVLink heartbeat.

         e.g., Plane, Rover, Boat, Sub, Copter, heli, Tracker, Blimp and Unknown.
         If the vehicle is `Unknown`, ask the user to check MAVLink connection.

        Returns:
            Vehicle type as a str
        """
        try:
            hearbeat_msg = mpstate.master().messages.get("HEARTBEAT", None)
        except AttributeError:
            return "Unknown"
        if not hearbeat_msg:
            return "Unknown"
        if hearbeat_msg.type in [mavutil.mavlink.MAV_TYPE_FIXED_WING,
                                 mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR,
                                 mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
                                 mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR]:
            vehicle_type_str = "Plane"
        elif hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            vehicle_type_str = "Rover"
        elif hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_SURFACE_BOAT:
            vehicle_type_str = "Boat"
        elif hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_SUBMARINE:
            vehicle_type_str = "Sub"
        elif hearbeat_msg.type in [mavutil.mavlink.MAV_TYPE_QUADROTOR,
                                   mavutil.mavlink.MAV_TYPE_COAXIAL,
                                   mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                                   mavutil.mavlink.MAV_TYPE_OCTOROTOR,
                                   mavutil.mavlink.MAV_TYPE_TRICOPTER,
                                   mavutil.mavlink.MAV_TYPE_DODECAROTOR]:
            vehicle_type_str = "Copter"
        elif hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_HELICOPTER:
            vehicle_type_str = "Heli"
        elif hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER:
            vehicle_type_str = "Tracker"
        elif hearbeat_msg.type == mavutil.mavlink.MAV_TYPE_AIRSHIP:
            vehicle_type_str = "Blimp"
        else:
            vehicle_type_str = "Unknown"
        return vehicle_type_str

    @tool
    def search_local_docs(filename: str) -> str:
        """Get the contents of a file in the local docs (documentation) folder.
        Args:
            filename (str): filename of the required documentation:
        """
        print("queried", os.path.join(BASE_DIR, "docs", filename))
        try:
            with open(os.path.join(BASE_DIR, "docs", filename), "r") as f:
                result = f.read()
        except FileNotFoundError:
            return "No relevant information found in the MAVProxy documentation."
        return result

    @tool
    def get_vehicle_state():
        """Get the current arming status and mode of the vehicle from latest heartbeat.

        If connected properly to the vehicle, return the vehicle arming status and flight mode.
        Vehicle must be armed to make any movements. Confirm if the vehicle is ready using this tool.

        Returns:
            dict containing vehicle arming status and mode number
        """
        hearbeat_msg = mpstate.master().messages.get('HEARTBEAT', None)
        if hearbeat_msg is None:
            mode_number = 0
            print("chat: get_vehicle_state: vehicle mode is unknown")
        else:
            mode_number = hearbeat_msg.custom_mode
        return {
            "armed": (mpstate.master().motors_armed() > 0),
            "mode": mode_number
        }

    @tool
    def get_all_parameters():
        pass

    @tool
    def get_available_mavlink_messages():
        pass

    @tool
    def get_location_plus_distance_at_bearing():
        pass

    @tool
    def get_location_plus_offset():
        pass

    @tool
    def get_mavlink_message():
        pass

    @tool
    def get_parameter():
        pass

    @tool
    def get_parameter_description():
        pass

    @tool
    def get_vehicle_location_and_yaw():
        pass

    @tool
    def send_mavlink_command_int_message():
        pass

    @tool
    def send_mavlink_set_position_target_global_int():
        pass

    @tool
    def set_parameter():
        pass

    # TODO: Add more tools

    return [get_vehicle_type, get_current_datetime, search_local_docs, get_vehicle_state, send_mavproxy_command]



