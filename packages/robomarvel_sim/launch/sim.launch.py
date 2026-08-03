"""Единый launch симуляции Robomarvel.

Покомпонентно повторяет main.yaml робота. Собирается ступенями через аргументы,
чтобы можно было проверять систему по частям, а не в конце.

  ros2 launch robomarvel_sim sim.launch.py odom:=perfect slam:=off nav:=false detection:=false
  ros2 launch robomarvel_sim sim.launch.py odom:=icp     slam:=off nav:=false detection:=false
  ros2 launch robomarvel_sim sim.launch.py slam:=mapping nav:=false detection:=false
  ros2 launch robomarvel_sim sim.launch.py                              # всё включено
  ros2 launch robomarvel_sim sim.launch.py slam:=localization           # по готовой базе

Аргументы:
  world      путь к YAML мира (по умолчанию classroom.yaml из пакета)
  odom       icp | perfect   - настоящая ICP-одометрия или истинная поза (отладка)
  slam       mapping | localization | off
  nav        true | false    - стек Nav2 + goal_proxy
  detection  true | false    - виртуальная детекция объектов
  db         путь к базе rtabmap
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = "robomarvel_sim"

# Параметры ядра RTAB-Map передаются строками. Набор для режима «только 2D-лидар»:
# визуальные признаки отключены (Kp/MaxFeatures = -1), замыкание циклов - по геометрии.
RTABMAP_COMMON = {
    "Reg/Strategy": "1",
    "Reg/Force3DoF": "true",
    "Icp/Strategy": "1",            # libpointmatcher
    # 2D-скан плоский: нормали считать не по чему. Без обнуления PointToPlaneK
    # rtabmap всё равно лезет их считать (K=5) и падает.
    "Icp/PointToPlane": "false",
    "Icp/PointToPlaneK": "0",
    "Icp/PointToPlaneRadius": "0",
    "Icp/VoxelSize": "0.05",
    "Icp/RangeMin": "0.05",
    "Icp/Iterations": "10",
    "Icp/Epsilon": "0.001",
    "Icp/MaxCorrespondenceDistance": "0.5",
    "Icp/OutlierRatio": "0.7",
    "Icp/CorrespondenceRatio": "0.05",
}


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory(PKG)

    world = LaunchConfiguration("world").perform(context)
    if not world:
        world = os.path.join(share, "config", "worlds", "classroom.yaml")
    world = os.path.abspath(os.path.expanduser(world))
    if not os.path.isfile(world):
        raise RuntimeError(f"Файл мира не найден: {world}")
    odom_mode = LaunchConfiguration("odom").perform(context)
    slam_mode = LaunchConfiguration("slam").perform(context)
    use_nav = LaunchConfiguration("nav").perform(context).lower() in ("true", "1", "yes")
    use_det = LaunchConfiguration("detection").perform(context).lower() in ("true", "1", "yes")
    db_path = LaunchConfiguration("db").perform(context)
    if not db_path:
        db_path = "/src/maps/rtabmap.db"
    # expanduser + abspath: иначе "~/..." уходит в rtabmap как есть, а относительный
    # путь без каталога (например скопированный из README плейсхолдер) даёт пустой
    # dirname и makedirs падает с невнятным "[Errno 2] No such file or directory: ''"
    db_path = os.path.abspath(os.path.expanduser(db_path))
    if not db_path.endswith(".db"):
        raise RuntimeError(
            f"Аргумент db должен быть путём к файлу .db, получено: {db_path}"
        )
    os.makedirs(os.path.dirname(db_path), exist_ok=True)
    if slam_mode == "localization" and not os.path.isfile(db_path):
        raise RuntimeError(
            f"Режим localization требует готовую базу, а её нет: {db_path}\n"
            f"Сначала постройте карту: slam:=mapping"
        )
    print(f"[sim.launch.py] мир: {world}")
    print(f"[sim.launch.py] база rtabmap: {db_path}")

    urdf_path = os.path.join(share, "urdf", "robot.urdf")
    with open(urdf_path, "r", encoding="utf-8") as fh:
        robot_description = fh.read()

    nav2_params = os.path.join(share, "config", "nav2.yaml")

    perfect_odom = odom_mode == "perfect"
    nodes = []

    # ---------------------------------------------------------------- база

    nodes.append(Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    ))

    nodes.append(Node(
        package=PKG, executable="sim_hwnode", name="sim_hwnode", output="screen",
        parameters=[{"world": world, "publish_odom_tf": perfect_odom}],
    ))

    nodes.append(Node(
        package=PKG, executable="sim_lidar", name="sim_lidar", output="screen",
        parameters=[{"world": world}],
    ))

    nodes.append(Node(
        package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
        output="screen", parameters=[{"port": 8765, "address": "0.0.0.0"}],
    ))

    # ---------------------------------------------------------------- одометрия

    if not perfect_odom:
        nodes.append(Node(
            package="rtabmap_odom", executable="icp_odometry", name="icp_odometry",
            output="screen",
            parameters=[dict(RTABMAP_COMMON, **{
                "frame_id": "base_link",
                "odom_frame_id": "odom",
                "publish_tf": True,
                "wait_for_transform": 0.2,
                "approx_sync": True,
                "deskewing": False,
                # ОБЯЗАТЕЛЬНО: без явного предела дальности rtabmap не отсекает
                # inf из пропусков скана, и бесконечности доходят до ICP.
                "scan_range_max": 12.0,
                "Odom/Strategy": "0",
                "Odom/GuessMotion": "true",
                "Odom/ResetCountdown": "0",
                "Odom/ScanKeyFrameThr": "0.6",
                "Icp/MaxTranslation": "2.0",
                "OdomF2M/ScanSubtractRadius": "0.05",
                "OdomF2M/ScanMaxSize": "15000",
                "OdomF2M/BundleAdjustment": "false",
            })],
            # icp_odometry подписывается и на scan, и на scan_cloud - второй уводим
            # в несуществующий топик, чтобы вход был ровно один.
            remappings=[
                ("scan", "/scan"),
                ("scan_cloud", "/unused_scan_cloud"),
                ("odom", "/icp/odom"),
            ],
        ))

    # ---------------------------------------------------------------- карта

    if slam_mode in ("mapping", "localization"):
        localization = slam_mode == "localization"
        rtabmap_params = dict(RTABMAP_COMMON, **{
            "frame_id": "base_link",
            "map_frame_id": "map",
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_rgbd": False,
            "subscribe_stereo": False,
            "subscribe_scan": True,
            "approx_sync": True,
            "publish_tf": True,
            "database_path": db_path,
            "Grid/FromDepth": "false",
            "Grid/CellSize": "0.025",
            "Grid/RangeMax": "12.0",
            "Grid/RayTracing": "true",
            "Grid/3D": "false",
            "RGBD/NeighborLinkRefining": "true",
            "RGBD/ProximityBySpace": "true",
            "RGBD/ProximityPathMaxNeighbors": "1",
            "RGBD/AngularUpdate": "0.05",
            "RGBD/LinearUpdate": "0.05",
            "RGBD/OptimizeFromGraphEnd": "false",
            "Mem/NotLinkedNodesKept": "false",
            "Mem/STMSize": "30",
            "Optimizer/Slam2D": "true",
            "Kp/MaxFeatures": "-1",
            "Rtabmap/DetectionRate": "1.0",
            "Icp/MaxTranslation": "3.0",
        })
        if localization:
            rtabmap_params["Mem/IncrementalMemory"] = "false"
            rtabmap_params["Mem/InitWMWithAllNodes"] = "true"
            # Без этого rtabmap стартует от последней позы ПРЕДЫДУЩЕЙ сессии (там, где
            # нажали Ctrl+C после объезда) и держит её, пока не узнает местность: map->odom
            # застывает в бессмысленном значении, скан не ложится на карту.
            # У нас робот всегда стартует из world.start, а это и есть начало карты.
            rtabmap_params["RGBD/StartAtOrigin"] = "true"
            args = []
        else:
            rtabmap_params["Mem/IncrementalMemory"] = "true"
            args = ["-d"]  # --delete_db_on_start

        nodes.append(Node(
            package="rtabmap_slam", executable="rtabmap", name="rtabmap", output="screen",
            parameters=[rtabmap_params],
            # У rtabmap есть свой встроенный планировщик, подписанный на "goal".
            # Это тот же топик, куда шлёт цель Foxglove и который слушает goal_proxy,
            # поэтому уводим внутренний планировщик rtabmap в сторону.
            remappings=[
                ("scan", "/scan"),
                ("odom", "/icp/odom"),
                ("goal", "/rtabmap/goal"),
                ("goal_out", "/rtabmap/goal_out"),
            ],
            arguments=args,
        ))
    else:
        # без SLAM всё равно нужен map -> odom, иначе Nav2 и Foxglove не построят дерево
        nodes.append(Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="static_map_odom",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        ))

    # ---------------------------------------------------------------- детекция

    if use_det:
        nodes.append(Node(
            package=PKG, executable="sim_detection", name="sim_detection", output="screen",
            parameters=[{"world": world}],
        ))

    # ---------------------------------------------------------------- Nav2

    if use_nav:
        for pkg, exe, name in [
            ("nav2_controller", "controller_server", "controller_server"),
            ("nav2_planner", "planner_server", "planner_server"),
            ("nav2_behaviors", "behavior_server", "behavior_server"),
            ("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        ]:
            nodes.append(Node(
                package=pkg, executable=exe, name=name, output="screen",
                parameters=[nav2_params],
            ))

        nodes.append(Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="lifecycle_manager_navigation", output="screen",
            parameters=[{
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
                "bond_timeout": 20.0,
                "attempt_respawn_reconnection": True,
            }],
        ))

        nodes.append(Node(
            package=PKG, executable="goal_proxy", name="goal_proxy", output="screen",
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=""),
        DeclareLaunchArgument("odom", default_value="icp",
                              choices=["icp", "perfect"]),
        DeclareLaunchArgument("slam", default_value="mapping",
                              choices=["mapping", "localization", "off"]),
        DeclareLaunchArgument("nav", default_value="true"),
        DeclareLaunchArgument("detection", default_value="true"),
        DeclareLaunchArgument("db", default_value=""),
        OpaqueFunction(function=launch_setup),
    ])
