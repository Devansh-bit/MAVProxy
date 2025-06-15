import os
from datetime import datetime
from typing import Callable, List, Literal, TypeVar

T = TypeVar('T')
BASE_DIR = os.path.dirname(__file__)

from langchain_core.tools import tool
from pymavlink import mavutil
from MAVProxy.mavproxy import MPState

#TODO remove
global armed
armed = True

def hybrid_rerank(docs, neural_results, bm25_results, alpha=0.5):
    """
    Simple hybrid rerank: average of normalized scores.
    neural_results: dict from ChromaDB query
    bm25_results: list of (doc_index, score)
    alpha: weight for neural score (1-alpha for BM25)
    """
    # Get neural scores (ChromaDB returns distances, not scores; convert to similarity)
    neural_scores = {}
    for doc_id, distance in zip(neural_results['ids'][0], neural_results['distances'][0]):
        # Convert distance to similarity (smaller distance = more similar)
        neural_scores[int(doc_id)] = 1 / (1 + distance)

    # Get BM25 scores
    bm25_scores = {}
    for doc_idx, score in bm25_results:
        bm25_scores[doc_idx] = score

    # Normalize scores
    if neural_scores:
        max_neural = max(neural_scores.values())
        for k in neural_scores: neural_scores[k] /= max_neural
    if bm25_scores:
        max_bm25 = max(bm25_scores.values())
        for k in bm25_scores: bm25_scores[k] /= max_bm25

    # Combine scores
    combined_scores = {}
    for doc_idx in range(len(docs)):
        neural = neural_scores.get(doc_idx, 0)
        bm25 = bm25_scores.get(doc_idx, 0)
        combined_scores[doc_idx] = alpha * neural + (1 - alpha) * bm25

    # Sort by combined score
    sorted_docs = sorted(combined_scores.items(), key=lambda x: -x[1])
    return sorted_docs



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
        print("Getting vehicle type")
        return "Copter" # TODO: Remove

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
    def search_local_docs(filename: Literal) -> str:
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
        global armed
        armed = not armed
        return { # TODO: Remove
            "armed": armed,
            "mode": 4
        }


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

    return [get_vehicle_type, get_current_datetime, search_local_docs, get_vehicle_state, send_mavproxy_command]

# TODO: Add more tools

# @tool
# def retrieve_documentation(query, n_results=2):
#     """Performs retrieval of local documentation and mavproxy documentation. Should be used liberally to gather information.
#
#     Query should be as descriptive as possible, as a neural network is performing retrieval of documentation
#
#     Args:
#          query (str): Query string
#          n_results (int): Number of documents to return. It Should always be less than 3
#     """
#     query_embed = model.encode(query)
#     neural_results = collection.query(
#         query_embeddings=[query_embed.tolist()],
#         n_results=n_results
#     )
#     content = ""
#     for document in neural_results['documents']:
#         content += document[0] + "\n"
#     print(content)
#     return content

# get mode mapping
# def get_mode_mapping(self, arguments):
#     # get name and/or number arguments
#     mode_name = arguments.get("name", None)
#     if mode_name is not None:
#         mode_name = mode_name.upper()
#     mode_number = arguments.get("number", None)
#     if mode_number is not None:
#         mode_number = int(mode_number)
#
#     # check mode mapping is available
#     if self.mpstate.master() is None or self.mpstate.master().mode_mapping() is None:
#         return "get_mode_mapping: failed to retrieve mode mapping"
#
#     # prepare list of modes
#     mode_list = []
#     mode_mapping = self.mpstate.master().mode_mapping()
#
#     # handle request for all modes
#     if mode_name is None and mode_number is None:
#         for mname in mode_mapping:
#             mnumber = mode_mapping[mname]
#             mode_list.append({"name": mname.upper(), "number": mnumber})
#
#     # handle request using mode name
#     elif mode_name is not None:
#         for mname in mode_mapping:
#             if mname.upper() == mode_name:
#                 mode_list.append({"name": mname.upper(), "number": mode_mapping[mname]})
#
#     # handle request using mode number
#     elif mode_number is not None:
#         for mname in mode_mapping:
#             mnumber = mode_mapping[mname]
#             if mnumber == mode_number:
#                 mode_list.append({"name": mname.upper(), "number": mnumber})
#
#     # return list of modes
#     return mode_list
#
# # get vehicle state including armed, mode
# def get_vehicle_state(self, arguments):
#     # get mode from latest HEARTBEAT message
#     hearbeat_msg = self.mpstate.master().messages.get('HEARTBEAT', None)
#     if hearbeat_msg is None:
#         mode_number = 0
#         print("chat: get_vehicle_state: vehicle mode is unknown")
#     else:
#         mode_number = hearbeat_msg.custom_mode
#     return {
#         "armed": (self.mpstate.master().motors_armed() > 0),
#         "mode": mode_number
#     }
#
# # return the vehicle's location and yaw
# def get_vehicle_location_and_yaw(self, arguments):
#     lat_deg = 0
#     lon_deg = 0
#     alt_amsl_m = 0
#     alt_rel_m = 0
#     yaw_deg = 0
#     gpi = self.mpstate.master().messages.get('GLOBAL_POSITION_INT', None)
#     if gpi:
#         lat_deg = gpi.lat * 1e-7,
#         lon_deg = gpi.lon * 1e-7,
#         alt_amsl_m = gpi.alt * 1e-3,
#         alt_rel_m = gpi.relative_alt * 1e-3
#         yaw_deg = gpi.hdg * 1e-2
#     location = {
#         "latitude": lat_deg,
#         "longitude": lon_deg,
#         "altitude_amsl": alt_amsl_m,
#         "altitude_above_home": alt_rel_m,
#         "yaw" : yaw_deg
#     }
#     return location
#
# # Calculate the latitude and longitude given distances (in meters) North and East
# def get_location_plus_offset(self, arguments):
#     lat = arguments.get("latitude", 0)
#     lon = arguments.get("longitude", 0)
#     dist_north = arguments.get("distance_north", 0)
#     dist_east = arguments.get("distance_east", 0)
#     lat_with_offset, lon_with_offset = self.get_latitude_longitude_given_offset(lat, lon, dist_north, dist_east)
#     return {
#         "latitude": lat_with_offset,
#         "longitude": lon_with_offset
#     }
#
# # Calculate the latitude and longitude given a distance (in meters) and bearing (in degrees)
# def get_location_plus_dist_at_bearing(self, arguments):
#     lat = arguments.get("latitude", 0)
#     lon = arguments.get("longitude", 0)
#     distance = arguments.get("distance", 0)
#     bearing_deg = arguments.get("bearing", 0)
#     dist_north = math.cos(math.radians(bearing_deg)) * distance
#     dist_east  = math.sin(math.radians(bearing_deg)) * distance
#     lat_with_offset, lon_with_offset = self.get_latitude_longitude_given_offset(lat, lon, dist_north, dist_east)
#     return {
#         "latitude": lat_with_offset,
#         "longitude": lon_with_offset
#     }
#
# # send a mavlink command_int message to the vehicle
# def send_mavlink_command_int(self, arguments):
#     target_system = arguments.get("target_system", self.mpstate.settings.target_system)
#     target_component = arguments.get("target_component", 1)
#     frame = arguments.get("frame", 0)
#     if ("command" not in arguments):
#         return "command_int not sent.  command field required"
#     command = arguments.get("command", 0)
#     current = arguments.get("current", 0)
#     autocontinue = arguments.get("autocontinue", 0)
#     param1 = arguments.get("param1", 0)
#     param2 = arguments.get("param2", 0)
#     param3 = arguments.get("param3", 0)
#     param4 = arguments.get("param4", 0)
#     x = arguments.get("x", 0)
#     y = arguments.get("y", 0)
#     z = arguments.get("z", 0)
#     # sanity check arguments
#     if command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF and z == 0:
#         return "command_int not sent.  MAV_CMD_NAV_TAKEOFF requires alt in z field"
#     self.mpstate.master().mav.command_int_send(target_system, target_component,
#                                                frame, command, current, autocontinue,
#                                                param1, param2, param3, param4,
#                                                x, y, z)
#
#     # wait for command ack
#     mav_result = self.wait_for_command_ack_fn(command)
#
#     # check for timeout
#     if mav_result is None:
#         print("send_mavlink_command_int: timed out")
#         return "command_int timed out"
#
#     # update assistant with result
#     if mav_result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
#         return "command_int succeeded"
#     if mav_result == mavutil.mavlink.MAV_RESULT_FAILED:
#         return "command_int failed"
#     if mav_result == mavutil.mavlink.MAV_RESULT_DENIED:
#         return "command_int denied"
#     if mav_result == mavutil.mavlink.MAV_RESULT_UNSUPPORTED:
#         return "command_int unsupported"
#     if mav_result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
#         return "command_int temporarily rejected"
#     print("send_mavlink_command_int: received unexpected command ack result")
#     return "command_int unknown result"
#
# # send a mavlink send_mavlink_set_position_target_global_int message to the vehicle
# def send_mavlink_set_position_target_global_int(self, arguments):
#     if arguments is None:
#         return "send_mavlink_set_position_target_global_int: arguments is None"
#     time_boot_ms = arguments.get("time_boot_ms", 0)
#     target_system = arguments.get("target_system", self.mpstate.settings.target_system)
#     target_component = arguments.get("target_component", 1)
#     coordinate_frame = arguments.get("coordinate_frame", 5)
#     type_mask = arguments.get("type_mask", 0)
#     lat_int = int(arguments.get("latitude", 0) * 1e7)
#     lon_int = int(arguments.get("longitude", 0) * 1e7)
#     alt = arguments.get("alt", 0)
#     vx = arguments.get("vx", 0)
#     vy = arguments.get("vy", 0)
#     vz = arguments.get("vz", 0)
#     afx = arguments.get("afx", 0)
#     afy = arguments.get("afy", 0)
#     afz = arguments.get("afz", 0)
#     yaw = arguments.get("yaw", 0)
#     yaw_rate = arguments.get("yaw_rate", 0)
#
#     # sanity check arguments
#     if type_mask == 3576:
#         # if position is specified check lat, lon, alt are provided
#         if "latitude" not in arguments.keys():
#             return "send_mavlink_set_position_target_global_int: latitude field required"
#         if "longitude" not in arguments.keys():
#             return "send_mavlink_set_position_target_global_int: longitude field required"
#         if "alt" not in arguments.keys():
#             return "send_mavlink_set_position_target_global_int: alt field required"
#
#     self.mpstate.master().mav.set_position_target_global_int_send(time_boot_ms, target_system, target_component,
#                                                                   coordinate_frame, type_mask,
#                                                                   lat_int, lon_int, alt,
#                                                                   vx, vy, vz,
#                                                                   afx, afy, afz,
#                                                                   yaw, yaw_rate)
#     return "set_position_target_global_int sent"
#
# # get a list of mavlink message names that can be retrieved using the get_mavlink_message function
# def get_available_mavlink_messages(self, arguments):
#     # check if no messages available
#     if self.mpstate.master().messages is None or len(self.mpstate.master().messages) == 0:
#         return "get_available_mavlink_messages: no messages available"
#
#     # retrieve each available message's name
#     mav_msg_names = []
#     for msg in self.mpstate.master().messages:
#         # append all message names except MAV
#         if msg != "MAV":
#             mav_msg_names.append(msg)
#
#     # return list of message names
#     return mav_msg_names
#
# # get a mavlink message including all fields and values sent by the vehicle
# def get_mavlink_message(self, arguments):
#     if arguments is None:
#         return "get_mavlink_message: arguments is None"
#
#     # retrieve requested message's name
#     mav_msg_name = arguments.get("message", None)
#     if mav_msg_name is None:
#         return "get_mavlink_message: message not specified"
#
#     # retrieve message
#     mav_msg = self.mpstate.master().messages.get(mav_msg_name, None)
#     if mav_msg is None:
#         return "get_mavlink_message: message not found"
#
#     # return message
#     return mav_msg.to_dict()
#
# # get all available parameters names and their values
# def get_all_parameters(self, arguments):
#     # check if any parameters are available
#     if self.mpstate.mav_param is None or len(self.mpstate.mav_param) == 0:
#         return "get_all_parameters: no parameters are available"
#     param_list = {}
#     for param_name in sorted(self.mpstate.mav_param.keys()):
#         param_list[param_name] = self.mpstate.mav_param.get(param_name)
#     return param_list
#
# # get a vehicle parameter's value
# def get_parameter(self, arguments):
#     param_name = arguments.get("name", None)
#     if param_name is None:
#         print("get_parameter: name not specified")
#         return "get_parameter: name not specified"
#
#     # start with empty parameter list
#     param_list = {}
#
#     # handle param name containing regex
#     if self.contains_regex(param_name):
#         pattern = re.compile(param_name)
#         for existing_param_name in sorted(self.mpstate.mav_param.keys()):
#             if pattern.match(existing_param_name) is not None:
#                 param_value = self.mpstate.functions.get_mav_param(existing_param_name, None)
#                 if param_value is None:
#                     print("chat: get_parameter unable to get " + existing_param_name)
#                 else:
#                     param_list[existing_param_name] = param_value
#     else:
#         # handle simple case of a single parameter name
#         param_value = self.mpstate.functions.get_mav_param(param_name, None)
#         if param_value is None:
#             return "get_parameter: " + param_name + " parameter not found"
#         param_list[param_name] = param_value
#
#     return param_list
#
# # set a vehicle parameter's value
# def set_parameter(self, arguments):
#     param_name = arguments.get("name", None)
#     if param_name is None:
#         return "set_parameter: parameter name not specified"
#     param_value = arguments.get("value", None)
#     if param_value is None:
#         return "set_parameter: value not specified"
#     self.mpstate.functions.param_set(param_name, param_value, retries=3)
#     return "set_parameter: parameter value set"
#
# # get vehicle parameter descriptions including description, units, min and max
# def get_parameter_description(self, arguments):
#     param_name = arguments.get("name", None)
#     if param_name is None:
#         return "get_parameter_description: name not specified"
#
#     # get parameter definitions
#     phelp = param_help.ParamHelp()
#     phelp.vehicle_name = "ArduCopter"
#     param_help_tree = phelp.param_help_tree(True)
#
#     # start with an empty parameter description dictionary
#     param_descriptions = {}
#
#     # handle param name containing regex
#     if self.contains_regex(param_name):
#         pattern = re.compile(param_name)
#         for existing_param_name in sorted(self.mpstate.mav_param.keys()):
#             if pattern.match(existing_param_name) is not None:
#                 param_desc = self.get_single_parameter_description(param_help_tree, existing_param_name)
#                 if param_desc is not None:
#                     param_descriptions[existing_param_name] = param_desc
#     else:
#         # handle simple case of a single parameter name
#         param_desc = self.get_single_parameter_description(param_help_tree, param_name)
#         if param_desc is None:
#             return "get_parameter_description: " + param_name + " parameter description not found"
#         param_descriptions[param_name] = param_desc
#
#     return param_descriptions
#
# # get a single parameter's descriptions as a dictionary
# # returns None if parameter description cannot be found
# def get_single_parameter_description(self, param_help_tree, param_name):
#     # search for parameter
#     param_info = None
#     if param_name in param_help_tree.keys():
#         param_info = param_help_tree[param_name]
#     else:
#         # check each possible vehicle name
#         for vehicle_name in ["ArduCopter", "ArduPlane", "Rover", "Sub"]:
#             vehicle_param_name = vehicle_name + ":" + param_name
#             if vehicle_param_name in param_help_tree.keys():
#                 param_info = param_help_tree[vehicle_param_name]
#                 break
#         if param_info is None:
#             return None
#
#     # start with empty dictionary
#     param_desc_dict = {}
#
#     # add name
#     param_desc_dict['name'] = param_name
#
#     # get description
#     param_desc_dict['description'] = param_info.get('documentation')
#
#     # iterate over fields to get units and range
#     param_fields = param_info.find('field')
#     if param_fields is not None:
#         for field in param_info.field:
#             field_name = field.get('name')
#             if field_name == 'Units':
#                 param_desc_dict['units'] = str(field)
#             if field_name == 'Range':
#                 if ' ' in str(field):
#                     param_desc_dict['min'] = str(field).split(' ')[0]
#                     param_desc_dict['max'] = str(field).split(' ')[1]
#
#     # iterate over values
#     param_values = param_info.find('values')
#     if param_values is not None:
#         param_value_dict = {}
#         for c in param_values.getchildren():
#             value_int = int(c.get('code'))
#             param_value_dict[value_int] = str(c)
#         if len(param_value_dict) > 0:
#             param_desc_dict['values'] = param_value_dict
#
#     # return dictionary
#     return param_desc_dict
#
# # set a wakeup timer
# def set_wakeup_timer(self, arguments):
#     # check required arguments are specified
#     seconds = arguments.get("seconds", -1)
#     if seconds < 0:
#         return "set_wakeup_timer: seconds not specified"
#     message = arguments.get("message", None)
#     if message is None:
#         return "set_wakeup_timer: message not specified"
#
#     # add timer to wakeup schedule
#     self.wakeup_schedule.append({"time": time.time() + seconds, "message": message})
#     return "set_wakeup_timer: wakeup timer set"
#
# # get wake timers
# def get_wakeup_timers(self, arguments):
#     # check message argument, default to None meaning all
#     message = arguments.get("message", None)
#
#     # prepare list of matching timers
#     matching_timers = []
#
#     # handle simple case of all timers
#     if message is None:
#         matching_timers = self.wakeup_schedule
#
#     # handle regex in message
#     elif self.contains_regex(message):
#         message_pattern = re.compile(message, re.IGNORECASE)
#         for wakeup_timer in self.wakeup_schedule:
#             if message_pattern.match(wakeup_timer["message"]) is not None:
#                 matching_timers.append(wakeup_timer)
#
#     # handle case of a specific message
#     else:
#         for wakeup_timer in self.wakeup_schedule:
#             if wakeup_timer["message"] == message:
#                 matching_timers.append(wakeup_timer)
#
#     # return matching timers
#     return matching_timers
#
# # delete wake timers
# def delete_wakeup_timers(self, arguments):
#     # check message argument, default to all
#     message = arguments.get("message", None)
#
#     # find matching timers
#     num_timers_deleted = 0
#
#     # handle simple case of deleting all timers
#     if message is None:
#         num_timers_deleted = len(self.wakeup_schedule)
#         self.wakeup_schedule.clear()
#
#     # handle regex in message
#     elif self.contains_regex(message):
#         message_pattern = re.compile(message, re.IGNORECASE)
#         for wakeup_timer in self.wakeup_schedule:
#             if message_pattern.match(wakeup_timer["message"]) is not None:
#                 num_timers_deleted = num_timers_deleted + 1
#                 self.wakeup_schedule.remove(wakeup_timer)
#     else:
#         # handle simple case of a single message
#         for wakeup_timer in self.wakeup_schedule:
#             if wakeup_timer["message"] == message:
#                 num_timers_deleted = num_timers_deleted + 1
#                 self.wakeup_schedule.remove(wakeup_timer)
#
#     # return number deleted and remaining
#     return "delete_wakeup_timers: deleted " + str(num_timers_deleted) + " timers, " + str(len(self.wakeup_schedule)) + " remaining" # noqa
#
# # check if any wakeup timers have expired and send messages if they have
# # this function never returns so it should be called from a new thread
# def check_wakeup_timers(self):
#     while True:
#         # wait for one second
#         time.sleep(1)
#
#         # check if any timers are set
#         if len(self.wakeup_schedule) == 0:
#             continue
#
#         # get current time
#         now = time.time()
#
#         # check if any timers have expired
#         for wakeup_timer in self.wakeup_schedule:
#             if now >= wakeup_timer["time"]:
#                 # send message to assistant
#                 message = "WAKEUP:" + wakeup_timer["message"]
#                 self.send_to_assistant(message)
#
#                 # remove from wakeup schedule
#                 self.wakeup_schedule.remove(wakeup_timer)
#
# # wrap latitude to range -90 to 90
# def wrap_latitude(self, latitude_deg):
#     if latitude_deg > 90:
#         return 180 - latitude_deg
#     if latitude_deg < -90:
#         return -(180 + latitude_deg)
#     return latitude_deg
#
# # wrap longitude to range -180 to 180
# def wrap_longitude(self, longitude_deg):
#     if longitude_deg > 180:
#         return longitude_deg - 360
#     if longitude_deg < -180:
#         return longitude_deg + 360
#     return longitude_deg
#
# # calculate latitude and longitude given distances (in meters) North and East
# # returns latitude and longitude in degrees
# def get_latitude_longitude_given_offset(self, latitude, longitude, dist_north, dist_east):
#     lat_lon_to_meters_scaling = 89.8320495336892 * 1e-7
#     lat_diff = dist_north * lat_lon_to_meters_scaling
#     lon_diff = dist_east * lat_lon_to_meters_scaling / max(0.01, math.cos(math.radians((latitude+lat_diff)/2)))
#     return self.wrap_latitude(latitude + lat_diff), self.wrap_longitude(longitude + lon_diff)
