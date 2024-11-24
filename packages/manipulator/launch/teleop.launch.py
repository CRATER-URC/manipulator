import os

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                      import LaunchDescription
from launch.actions              import Shutdown
from launch_ros.actions          import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('manipulator'), 'rviz/viewurdf.rviz')

    # Locate/load the robot's URDF file (XML).
    urdf = os.path.join(pkgdir('manipulator'), 'urdf/arm.urdf')
    with open(urdf, 'r') as file:
        robot_description = file.read()


    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS
    
    node_teleop = Node(
        name       = 'Teleop',
        package    = 'manipulator',
        executable = 'Teleop',
        output     = 'screen',
        on_exit    = Shutdown())



    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST

    return LaunchDescription([

        node_teleop
    ])
