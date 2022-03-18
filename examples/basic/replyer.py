import random
import time

from osparc_control import CommandManifest
from osparc_control import CommandParameter
from osparc_control import CommnadType
from osparc_control import ControlInterface

random_in_range_manifest = CommandManifest(
    action="random_in_range",
    description="gets the time",
    params=[
        CommandParameter(name="a", description="lower bound for random numbers"),
        CommandParameter(name="b", description="upper bound for random numbers"),
    ],
    command_type=CommnadType.WITH_IMMEDIATE_REPLY,
)

control_interface = ControlInterface(
    remote_host="localhost",
    exposed_interface=[random_in_range_manifest],
    remote_port=2346,
    listen_port=2345,
)

wait_for_requests = True

control_interface.start_background_sync()

while wait_for_requests:
    commands = control_interface.get_incoming_requests()
    for command in commands:
        if command.action == random_in_range_manifest.action:
            random_int = random.randint(  
                command.params["a"], command.params["b"]
            )
            control_interface.reply_to_command(
                request_id=command.request_id, payload=random_int
            )
            time.sleep(0.1)
            wait_for_requests = False
            print(command)

# # allow for message to be delivered
# time.sleep(0.01)

control_interface.stop_background_sync()
