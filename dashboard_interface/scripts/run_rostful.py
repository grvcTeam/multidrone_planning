#!/usr/bin/env python
import subprocess
import rospkg
import rospy


def main():

    # Get an instance of RosPack with the default search paths
    rospack = rospkg.RosPack()

    # Get config file path
    config_path = rospack.get_path("dashboard_interface") + "/config/rostful.cfg"
    
    # Run rostful
    rostful_args = "python -m rostful run -c " + config_path

    try:
        subprocess.call(rostful_args, shell=True)
    except KeyboardInterrupt:
        pass
    finally:
        pass


if __name__ == "__main__":
    main()
