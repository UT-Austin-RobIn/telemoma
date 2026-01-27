INTERFACE_MAP = {}

def try_import(module, class_name, interface_name):
    try:
        interface_class = __import__(module, fromlist=[class_name])
        INTERFACE_MAP[interface_name] = getattr(interface_class, class_name)
    except ImportError as e:
        print(e)

try_import('telemoma.human_interface.oculus', 'OculusPolicy', 'oculus')
try_import('telemoma.human_interface.vision', 'VisionTeleopPolicy', 'vision')
try_import('telemoma.human_interface.keyboard', 'KeyboardInterface', 'keyboard')
try_import('telemoma.human_interface.spacemouse', 'SpaceMouseInterface', 'spacemouse')
try_import('telemoma.human_interface.mobile_phone', 'MobilePhonePolicy', 'mobile_phone')
