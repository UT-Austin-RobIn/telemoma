import numpy as np
from telemoma.robot_interface.igibson import *
from telemoma.human_interface.teleop_policy import TeleopPolicy
from importlib.machinery import SourceFileLoader

COMPATIBLE_ROBOTS = ['tiago', 'fetch']


def get_random_action():
    return {
        'base': np.random.uniform(-1, 1, 3),
        'torso': np.random.uniform(-1, 1, 1),
        'left': np.random.uniform(-1, 1, 7),
        'right': np.random.uniform(-1, 1, 7),
    }

def main(args):
    from igibson.utils.assets_utils import download_assets, download_demo_data
    download_assets()
    download_demo_data()

    env = FetchEnv() if args.robot=='fetch' else TiagoEnv()

    teleop_config = SourceFileLoader('conf', args.teleop_config).load_module().teleop_config
    teleop = TeleopPolicy(teleop_config)
    teleop.start()

    obs = env.reset()
    for i in range(1000):
        action = teleop.get_action(obs) # get_random_action()
        buttons = action.extra['buttons'] if 'buttons' in action.extra else {}
    
        if buttons.get('A', False) or buttons.get('B', False):
            break

        obs, _, _, _ = env.step(action)
        
    teleop.stop()
    env.close()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--robot', type=str, default='tiago', help='Robot to use. Choose between tiago and fetch.')
    parser.add_argument('--teleop_config', type=str, help='Path to the teleop config to use.')
    args = parser.parse_args()

    assert args.robot in COMPATIBLE_ROBOTS, f'Unknown robots. Choose one from: {" ".join(COMPATIBLE_ROBOTS)}' 
    main(args)