import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
from launch.events import matches_action
from launch.actions import RegisterEventHandler
from launch_ros.event_handlers import OnStateTransition
    
def generate_launch_description():
    package_dir = get_package_share_directory('warehouse_robot')
    slam_config = os.path.join(
        package_dir,
        'config',
        'slam.yaml'
    )
    slam_node = LifecycleNode(
    package='slam_toolbox',
    executable='async_slam_toolbox_node',
    name='slam_toolbox',
    namespace='',
    parameters=[slam_config]
)
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE
    )
)
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE
                )
            )
        ]
    )
)
    return LaunchDescription([
        slam_node,
        configure_slam,
        activate_slam
])