from telemoma.utils.general_utils import AttrDict

teleop_config = AttrDict(
    arm_left_controller=None,
    arm_right_controller=None,
    base_controller=None,
    torso_controller=None,
    use_oculus=False,
    interface_kwargs=AttrDict(
        oculus={},
        vision={},
        mobile_phone={},
        spacemouse={},
        keyboard={},
    )
)