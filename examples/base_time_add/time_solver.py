from time import sleep
import random
from sidecar_solver import command_add
from sidecar_solver import command_get_time
from sidecar_solver import command_print_solver_status
from sidecar_solver import command_random_in_range 
from sidecar_solver import control_interface

from osparc_control.core import ControlInterface
from osparc_control.models import CommandRequest


def handle_inputs(time_solver: "TimeSolver", request: CommandRequest) -> None:
    if request.action == command_add.action:
        sum_result = time_solver._add(**request.params)
        time_solver.control_interface.reply_to_command(
            request_id=request.request_id, payload=sum_result
        )
        return

    if request.action == command_get_time.action:
        time_solver.control_interface.reply_to_command(
            request_id=request.request_id, payload=time_solver.time
        )
        return

    if request.action == command_print_solver_status.action:
        print("Solver internal status", time_solver)
        # finally exit
        time_solver.can_continue = False
        return
    
    if request.action == command_random_in_range.action :
        random_int = random.randint(  
            request.params["a"], request.params["b"]
        )
        time_solver.control_interface.reply_to_command(
            request_id=request.request_id, payload=random_int
        )
        time_solver.can_continue = False
        return


class TimeSolver:
    def __init__(
        self, initial_time: float, control_interface: ControlInterface
    ) -> None:
        self.time = initial_time
        self.control_interface = control_interface

        # internal time tick
        self.sleep_internal: float = 0.1
        self.can_continue: bool = True

    def __repr__(self) -> str:
        return (
            f"<{self.__class__.__name__} time={self.time}, "
            f"sleep_interval={self.sleep_internal}>"
        )

    def _add(self, a: float) -> float:
        return self.time + a

    def run(self) -> None:
        """main loop of the solver"""
        while self.can_continue:
            # process incoming requests from remote
            for request in self.control_interface.get_incoming_requests():
                handle_inputs(time_solver=self, request=request)

            # process internal stuff
            self.time += 1
            # sleep(self.sleep_internal)


def main() -> None:
    control_interface.start_background_sync()

    solver = TimeSolver(initial_time=0, control_interface=control_interface)
    solver.run()

    control_interface.stop_background_sync()


if __name__ == "__main__":
    main()
