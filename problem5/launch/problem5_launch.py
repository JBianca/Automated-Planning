import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    package_name = "problem5"  
    example_dir = get_package_share_directory(package_name)
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch_monolithic.py')),
        launch_arguments={
          'model_file': example_dir + '/pddl/domain.pddl',
          'namespace': namespace
          }.items())
    
    move_action_node = Node(
        package=package_name,
        executable="move_action_node",
        name="move_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )

    seal_action_node = Node(
        package=package_name,
        executable="seal_action_node",
        name="seal_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )
    
    unseal_action_node = Node(
        package=package_name,
        executable="unseal_action_node",
        name="unseal_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )
    
    pick_hall_alpha_action_node = Node(
        package=package_name,
        executable="pick_hall_alpha_action_node",
        name="pick_hall_alpha_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )
    
    pick_hall_beta_action_node = Node(
        package=package_name,
        executable="pick_hall_beta_action_node",
        name="pick_hall_beta_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )
    
    drop_at_cryo_action_node = Node(
        package=package_name,
        executable="drop_at_cryo_action_node",
        name="drop_at_cryo_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )

    drop_at_pod_action_node = Node(
        package=package_name,
        executable="drop_at_pod_action_node",
        name="drop_at_pod_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )

    cool_artifact_action_node = Node(
        package=package_name,
        executable="cool_artifact_action_node",
        name="cool_artifact_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )
    
    activate_antivibration_action_node = Node(
        package=package_name,
        executable="activate_antivibration_action_node",
        name="activate_antivibration_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )
    
    pick_from_cryo_action_node = Node(
        package=package_name,
        executable="pick_from_cryo_action_node",
        name="pick_from_cryo_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )

    pick_from_pod_action_node = Node(
        package=package_name,
        executable="pick_from_pod_action_node",
        name="pick_from_pod_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )

    deliver_to_stasis_action_node = Node(
        package=package_name,
        executable="deliver_to_stasis_action_node",
        name="deliver_to_stasis_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )

    stabilize_action_node = Node(
        package=package_name,
        executable="stabilize_action_node",
        name="stabilize_action_node",
        namespace=namespace,
        output="screen",
        parameters=[],
    )

    ld = LaunchDescription()

    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(plansys2_cmd)

    ld.add_action(move_action_node)
    ld.add_action(seal_action_node)
    ld.add_action(unseal_action_node)
    ld.add_action(pick_hall_alpha_action_node)
    ld.add_action(pick_hall_beta_action_node)
    ld.add_action(drop_at_cryo_action_node)
    ld.add_action(drop_at_pod_action_node)
    ld.add_action(cool_artifact_action_node)
    ld.add_action(activate_antivibration_action_node)
    ld.add_action(pick_from_cryo_action_node)
    ld.add_action(pick_from_pod_action_node)
    ld.add_action(deliver_to_stasis_action_node)
    ld.add_action(stabilize_action_node)
    
    return ld