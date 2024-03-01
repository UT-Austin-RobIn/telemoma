from telemoma.human_interface.oculus import OculusPolicy
from telemoma.human_interface.vision import VisionTeleopPolicy
from telemoma.human_interface.keyboard import KeyboardInterface
from telemoma.human_interface.spacemouse import SpaceMouseInterface
from telemoma.human_interface.mobile_phone import MobilePhonePolicy

INTERFACE_MAP = {
    'oculus': OculusPolicy,
    'vision': VisionTeleopPolicy,
    'keyboard': KeyboardInterface,
    'spacemouse': SpaceMouseInterface,
    'mobile_phone': MobilePhonePolicy
}
