from pathlib import Path
import os
import shlex
import xml.etree.ElementTree as ET
from tempfile import TemporaryDirectory

from agri_sim_sensors.configuration import (
    find_workspace, load_config, resolve_backend, write_bridge, write_world, xacro_mappings,
)
from agri_sim_sensors.camera_configuration import (
    camera_xacro_mappings, image_bridge_environment, load_camera_config, write_camera_bridge,
    write_camera_rviz,
)

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    RegisterEventHandler,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import xacro


def _make_gazebo(context, world, *args, **kwargs):
    gui = LaunchConfiguration("gui").perform(context).lower()
    paused = LaunchConfiguration("paused").perform(context).lower()
    run_flag = "" if paused in {"1", "true", "yes", "on"} else " -r"
    server_only = " -s" if gui in {"0", "false", "no", "off"} else ""
    headless = (" --headless-rendering" if server_only and
                _enabled(context, "headless_rendering") else "")
    gz_share = Path(get_package_share_directory("ros_gz_sim"))
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(gz_share / "launch" / "gz_sim.launch.py")),
        # ros_gz_sim uses a shell command internally; quote paths because this
        # workspace may live under a directory containing spaces.
        launch_arguments={"gz_args": f'{run_flag}{server_only}{headless} {shlex.quote(str(world))}'}.items(),
    )
    return [gz_launch]


def _make_robot(context, model_xacro, controller_config, sensor_mappings):
    use_control = LaunchConfiguration("use_control").perform(context).lower()
    control_enabled = use_control in {"1", "true", "yes", "on"}
    robot_description = xacro.process_file(
        str(model_xacro),
        mappings={
            **sensor_mappings,
            "use_lidar": "true" if _enabled(context, "use_lidar") else "false",
            "use_camera": "true" if _enabled(context, "use_camera") else "false",
            "use_gazebo": "true",
            "use_ros2_control": "true" if control_enabled else "false",
            "use_control": "true" if control_enabled else "false",
            "controller_config": str(controller_config),
        },
    ).toxml()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
                # Joint-state broadcaster runs at 100 Hz. Keep image-time TF
                # close to the 30 Hz wrist camera, including moving joints.
                "publish_frequency": 100.0,
            }
        ],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_agri_robot",
        output="screen",
        arguments=[
            "-world",
            LaunchConfiguration("world_name"),
            "-x",
            LaunchConfiguration("spawn_x"),
            "-y",
            LaunchConfiguration("spawn_y"),
            "-Y",
            LaunchConfiguration("spawn_yaw"),
            "-topic",
            "robot_description",
            "-name",
            "agri_robot",
            "-z",
            LaunchConfiguration("spawn_z"),
        ],
    )
    return [robot_state_publisher, spawn]


def _enabled(context, name):
    return LaunchConfiguration(name).perform(context).lower() in {"1", "true", "yes", "on"}


def _prepare_simulation(context):
    sim_share = Path(get_package_share_directory("agri_sim_description"))
    sensor_share = Path(get_package_share_directory("agri_sim_sensors"))
    description_share = Path(get_package_share_directory("agri_robot_description"))
    control_share = Path(get_package_share_directory("agri_sim_control"))
    use_scan = _enabled(context, "use_scan")
    if use_scan and not _enabled(context, "use_lidar"):
        raise ValueError("use_scan:=true requires use_lidar:=true in the simulation")
    scan_share = Path(get_package_share_directory("agri_lidar_adapter"))
    workspace = find_workspace(sensor_share)
    world_arg = Path(LaunchConfiguration("world").perform(context))
    world_source = world_arg if world_arg.is_absolute() else sim_share / "worlds" / world_arg
    actual_world_name = ET.parse(world_source).getroot().find("world").get("name")
    requested_world_name = LaunchConfiguration("world_name").perform(context)
    if requested_world_name and requested_world_name != actual_world_name:
        raise ValueError(f"world_name={requested_world_name!r} does not match SDF world {actual_world_name!r}")
    context.launch_configurations["world_name"] = actual_world_name
    config_arg = LaunchConfiguration("lidar_config").perform(context)
    config = load_config(Path(config_arg) if config_arg else sensor_share / "config" / "mid360.yaml")
    camera_arg = LaunchConfiguration("camera_config").perform(context)
    camera_config = load_camera_config(
        Path(camera_arg) if camera_arg else sensor_share / "config" / "d405.yaml")
    prefix_arg = LaunchConfiguration("rgl_install_prefix").perform(context)
    patterns_arg = LaunchConfiguration("rgl_patterns_dir").perform(context)
    prefix = Path(prefix_arg or config['rgl']['install_prefix'])
    patterns = Path(patterns_arg or config['rgl']['patterns_dir'])
    prefix = prefix if prefix.is_absolute() else workspace / prefix
    patterns = patterns if patterns.is_absolute() else workspace / patterns
    mode, message = resolve_backend(
        LaunchConfiguration("lidar_mode").perform(context) if _enabled(context, "use_lidar")
        else "gpu_lidar", prefix, patterns,
    )
    runtime = TemporaryDirectory(prefix="agri_mid360_")
    world = Path(runtime.name) / "world.sdf"
    bridge_config = Path(runtime.name) / "bridge.yaml"
    clock_bridge_config = Path(runtime.name) / "clock_bridge.yaml"
    camera_bridge_config = Path(runtime.name) / "camera_bridge.yaml"
    camera_raw_info_topic = camera_config['info_topic'] + '/gz_raw'
    write_world(world_source, world, config, mode)
    write_bridge(config, bridge_config, mode)
    write_camera_bridge(camera_config, camera_bridge_config, include_images=False,
                        info_topic_override=camera_raw_info_topic)
    model_xacro = sim_share / "urdf" / "agri_robot.gazebo.urdf.xacro"
    controller_config = control_share / "config" / "controllers.yaml"
    use_control = LaunchConfiguration("use_control")
    resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(filter(None, [LaunchConfiguration("resource_path").perform(context),
              str(description_share.parent),
              str(world_source.parent), context.environment.get("GZ_SIM_RESOURCE_PATH", "")])),
    )
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("agri_sim_control"))
                / "launch"
                / "control.launch.py"
            )
        ),
        condition=IfCondition(use_control),
    )

    base_adapter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("agri_base_adapter"))
                / "launch"
                / "base_adapter.launch.py"
            )
        ),
        launch_arguments={
            "config": str(
                Path(get_package_share_directory("agri_base_adapter"))
                / "config"
                / "base_adapter.yaml"
            ),
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_kinematics")),
    )

    kinematics_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                Path(get_package_share_directory("agri_base_kinematics"))
                / "launch"
                / "four_wheel_steering.launch.py"
            )
        ),
        launch_arguments={
            "config": str(
                Path(get_package_share_directory("agri_base_kinematics"))
                / "config"
                / "four_wheel_steering.yaml"
            ),
            "use_sim_time": "true",
            "cmd_vel_topic": "/cmd_vel_safe",
            "odom_topic": "/wheel/odom",
            "publish_tf": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_kinematics")),
    )

    scan_config_arg = LaunchConfiguration("scan_config").perform(context)
    scan_config = Path(scan_config_arg) if scan_config_arg else scan_share / "config" / "lidar_to_scan.yaml"
    scan_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(scan_share / "launch" / "lidar_to_scan.launch.py")),
        launch_arguments={
            "config": str(scan_config),
            "input_topic": LaunchConfiguration("scan_input_topic"),
            "cloud_topic": LaunchConfiguration("scan_cloud_topic"),
            "scan_topic": LaunchConfiguration("scan_topic"),
            "target_frame": LaunchConfiguration("scan_target_frame"),
            "min_height": LaunchConfiguration("scan_min_height"),
            "max_height": LaunchConfiguration("scan_max_height"),
            "range_min": LaunchConfiguration("scan_range_min"),
            "range_max": LaunchConfiguration("scan_range_max"),
            "angle_increment": LaunchConfiguration("scan_angle_increment"),
            "voxel_size": LaunchConfiguration("scan_voxel_size"),
            "use_sim_time": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_scan")),
    )

    # Bridge the clock owned by this exact world instead of Gazebo's global
    # /clock alias.  The alias can be supplied by another server in the same
    # Gazebo partition.  CLOCK QoS (depth 1, best effort, volatile) also keeps
    # a 1 kHz simulation clock from accumulating stale samples under load.
    gz_clock_topic = f"/world/{actual_world_name}/clock"
    clock_bridge_config.write_text(
        "- ros_topic_name: /clock\n"
        f"  gz_topic_name: {gz_clock_topic}\n"
        "  ros_type_name: rosgraph_msgs/msg/Clock\n"
        "  gz_type_name: gz.msgs.Clock\n"
        "  direction: GZ_TO_ROS\n"
        "  lazy: false\n"
        "  qos_profile: CLOCK\n",
        encoding="utf-8",
    )
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[{"config_file": str(clock_bridge_config), "use_sim_time": True}],
    )

    actions = [resource_path,
               SetEnvironmentVariable("GZ_PARTITION", LaunchConfiguration("gz_partition")),
               LogInfo(msg=message),
               LogInfo(msg=f"Clock bridge: {gz_clock_topic} -> /clock (CLOCK QoS)")]
    if mode == "rgl":
        actions.extend([
            SetEnvironmentVariable("GZ_SIM_SYSTEM_PLUGIN_PATH", os.pathsep.join(filter(None, [
                str(prefix / "RGLServerPlugin"), context.environment.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")]))),
            SetEnvironmentVariable("RGL_PATTERNS_DIR", str(patterns)),
        ])
    actions.extend(_make_gazebo(context, world))
    actions.append(clock_bridge)
    actions.extend(_make_robot(
        context, model_xacro, controller_config,
        {**xacro_mappings(config, mode), **camera_xacro_mappings(camera_config)},
    ))
    if _enabled(context, "use_lidar"):
        actions.append(Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="mid360_bridge",
            output="screen", parameters=[{'config_file': str(bridge_config), 'use_sim_time': True}],
            additional_env=image_bridge_environment(
                sensor_share / 'config' / 'd405_fastdds.xml', context.environment),
        ))
        if mode == "rgl":
            actions.append(Node(
                package="agri_sim_sensors", executable="normalize_mid360_cloud",
                name="mid360_cloud_filter", output="screen", parameters=[{
                    'use_sim_time': True,
                    'raw_topic': config['gz_topic'].rstrip('/') + '/rgl_raw',
                    'points_topic': config['points_topic'], 'frame_id': config['frame_id'],
                    'min_range': config['min_range'], 'max_range': config['max_range'],
                    'vertical_min_angle': config['vertical_min_angle'],
                    'vertical_max_angle': config['vertical_max_angle'],
                }],
            ))
    if _enabled(context, "use_camera"):
        actions.extend([
            LogInfo(msg=(f"D405: aligned RGB-D {camera_config['width']}x{camera_config['height']} "
                         f"at {camera_config['update_rate']:g} Hz, "
                         f"depth {camera_config['depth_near']:g}..{camera_config['depth_far']:g} m")),
            Node(
                package="ros_gz_bridge", executable="parameter_bridge", name="d405_bridge",
                output="screen", parameters=[{
                    'config_file': str(camera_bridge_config), 'use_sim_time': True,
                }],
            ),
            Node(
                package="agri_sim_sensors", executable="normalize_d405_camera_info",
                name="d405_camera_info", output="screen", parameters=[{
                    'use_sim_time': True, 'input_topic': camera_raw_info_topic,
                    'output_topic': camera_config['info_topic'],
                    'pixel_center_offset': float(camera_config['pixel_center_offset']),
                }],
            ),
            Node(
                package="ros_gz_image", executable="image_bridge", name="d405_image_bridge",
                output="screen",
                arguments=[camera_config['gz_topic'] + '/image',
                           camera_config['gz_topic'] + '/depth_image'],
                parameters=[{'qos': 'sensor_data', 'use_sim_time': True}],
                additional_env=image_bridge_environment(
                    sensor_share / 'config' / 'd405_fastdds.xml', context.environment),
                remappings=[
                    (camera_config['gz_topic'] + '/image', camera_config['color_topic']),
                    (camera_config['gz_topic'] + '/depth_image', camera_config['depth_topic']),
                ],
            ),
        ])
    actions.extend([control_launch, base_adapter_launch, kinematics_launch, scan_launch])
    if _enabled(context, "rviz"):
        rviz_config = sensor_share / 'rviz' / 'mid360.rviz'
        if _enabled(context, 'use_camera'):
            rviz_config = Path(runtime.name) / 'sensors.rviz'
            write_camera_rviz(camera_config, sensor_share / 'rviz' / 'sensors.rviz', rviz_config)
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=['-d', str(rviz_config)],
            parameters=[{'use_sim_time': True}], output='screen',
        ))
    def cleanup(_context):
        runtime.cleanup()
        return []
    actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="empty_greenhouse.sdf"),
        DeclareLaunchArgument("world_name", default_value="", description="Optional SDF world name; inferred when empty."),
        DeclareLaunchArgument("resource_path", default_value=""),
        DeclareLaunchArgument("gz_partition", default_value="agri_sim"),
        DeclareLaunchArgument("spawn_x", default_value="0.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("headless_rendering", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("spawn_z", default_value="0.40"),
        DeclareLaunchArgument("use_control", default_value="true"),
        DeclareLaunchArgument("use_kinematics", default_value=LaunchConfiguration("use_control")),
        DeclareLaunchArgument("use_lidar", default_value="true"),
        DeclareLaunchArgument("use_scan", default_value="false",
                             description="Publish a horizontal LaserScan from MID-360 points."),
        DeclareLaunchArgument("use_camera", default_value="true"),
        DeclareLaunchArgument("camera_config", default_value=""),
        DeclareLaunchArgument("lidar_mode", default_value="gpu_lidar"),
        DeclareLaunchArgument("lidar_config", default_value=""),
        DeclareLaunchArgument("scan_config", default_value=""),
        DeclareLaunchArgument("scan_input_topic", default_value="/mid360/points"),
        DeclareLaunchArgument("scan_cloud_topic", default_value="/mid360/navigation_points"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("scan_target_frame", default_value="mid360_scan_frame"),
        DeclareLaunchArgument("scan_min_height", default_value="-0.40"),
        DeclareLaunchArgument("scan_max_height", default_value="0.40"),
        DeclareLaunchArgument("scan_range_min", default_value="0.10"),
        DeclareLaunchArgument("scan_range_max", default_value="40.0"),
        DeclareLaunchArgument("scan_angle_increment", default_value="0.017453292519943295"),
        DeclareLaunchArgument("scan_voxel_size", default_value="0.0"),
        DeclareLaunchArgument("rgl_install_prefix", default_value=""),
        DeclareLaunchArgument("rgl_patterns_dir", default_value=""),
        OpaqueFunction(function=_prepare_simulation),
    ])
