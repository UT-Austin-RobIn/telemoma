import numpy as np
from dataclasses import dataclass, field
from telemoma.utils.general_utils import AttrDict


@dataclass
class TeleopAction(AttrDict):
    left: np.ndarray = field(default_factory=lambda: np.r_[np.zeros(6), np.ones(1)])
    right: np.ndarray = field(default_factory=lambda: np.r_[np.zeros(6), np.ones(1)])
    base: np.ndarray = field(default_factory=lambda: np.zeros(3))
    torso: float = field(default_factory=lambda: 0.)
    extra: dict = field(default_factory=dict)


@dataclass
class TeleopObservation(AttrDict):
    left: np.ndarray = field(default_factory=lambda: np.r_[np.zeros(6), np.ones(2)])
    right: np.ndarray = field(default_factory=lambda: np.r_[np.zeros(6), np.ones(2)])
    base: np.ndarray = field(default_factory=lambda: np.zeros(3))
    torso: float = field(default_factory=lambda: 0.)
    extra: dict = field(default_factory=dict)
    

class BaseTeleopInterface:
    """
    Base class for teleop policies
    """

    def __init__(self, *args, **kwargs) -> None:
        """
        Initializes the teleop policy
        """
        # raw data directly obtained from the teleoperation device, updated by the _update_internal_state method
        # this attribute will be used in the get_action method to generate action for each teleop robot part
        self.raw_data = {}

    def start(self) -> None:
        """
        Starts the teleop policy
        """
        raise NotImplementedError

    def stop(self) -> None:
        """
        Stops the teleop policy
        """
        raise NotImplementedError

    def reset_state(self) -> None:
        """
        Reset all internal states
        """
        raise NotImplementedError

    def _update_internal_state(self) -> None:
        """
        Update self.raw_data
        """
        raise NotImplementedError

    def get_action(self, obs: TeleopObservation) -> TeleopAction:
        """
        Get the action of a body part
        """
        raise NotImplementedError

    def get_default_action(self) -> TeleopAction:
        return TeleopAction()
