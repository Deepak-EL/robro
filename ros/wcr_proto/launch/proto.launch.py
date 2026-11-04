"""WCR PROTOTYPE twin in Webots (Windows) + ROS 2 (WSL) on the dam faces of wcr_dam_sim.

  ros2 launch wcr_proto proto.launch.py [world:=winter_dam|goldeneye_dam|goldeneye_gorge]
        [foxglove:=true] [rviz:=false] [nav:=false] [mission:=false area:=demo]
        [throttle:=full|2..7|auto] [speed:=0.11] [fast:=false] [start_webots:=true]

Reused from wcr_dam_sim (found through the installed wcr_webots package): the worlds and their
assets, inspection layouts and wall maps, the inspection_detector and wall_frames nodes, the
coverage mission (subclassed here for skid steer), Nav2 / EKF configs (adapted copies in config/).

- Writes webots/worlds/_proto_<world>.wbt in THIS repo: the dam world with the robot swapped for
  WcrProto (webots/protos, tools/build_proto_webots.py) and its asset paths pointed at wcr_dam_sim.
- webots_ros2_driver loads resource/wcr_proto_driver.urdf: /scan, /wall_camera, /front_camera, /imu
  and the ProtoDriver plugin (thrust, skid steer, /odom, /joint_states, /ground_truth, /thrust).
- throttle: '2'..'7' = the servo-tester settings with the thrust measured on the scale (100..400 g),
  'full' = datasheet full throttle (~850 g), 'auto' = planned ESP32 thrust control from the IMU tilt.
"""
import os
import subprocess
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction,
                            RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

WEBOTS_HOME_WIN = '/mnt/c/Users/DEEPAK E L/AppData/Local/Programs/Webots'
WORLDS = ('winter_dam', 'goldeneye_dam', 'goldeneye_gorge')
# Twin's collision face per world (face_box): the drive surface as h = h0 + a s + b t [m] above the
# layout's face plane, fitted to the VISUAL face mesh by tools/fit_face.py. The prototype rides only
# 7.5 mm off the wall, so a collision plane in front of / behind the drawn wall shows it inside the
# wall or floating. winter_dam keeps its own box face (the drawn face is 3 mm proud of it: a texture
# layer). goldeneye_gorge: the collision is a mesh (exactly the face plane here, drawn 3 mm proud),
# and asymmetric tyre friction needs a primitive. goldeneye_dam: the drawn face (obj_Object_7) is
# 7-24 mm behind the dam sim's best-fit face box, which is moved out of the way.
FACE_FIT = {'goldeneye_gorge': (0.003, 0.0, 0.0),
            'goldeneye_dam': (-0.02377 + 0.003, 0.003578, 0.000225)}
# drawn parts in front of the drive surface: pushed back behind it (metres along -normal)
PUSH_BACK = {'goldeneye_dam': {'obj_Object_11.obj': 0.08}}   # a lower band, 28-33 mm proud


def shifted_layout(src, fit, dst):
    """Copy of the inspection layout with its face plane (the map frame) moved along the normal onto
    the twin's drive surface at the demo area centre, so the robot is not drawn below the map layers."""
    import yaml
    doc = yaml.safe_load(src.read_text())
    h0, a, b = fit
    sc, tc = (sum(doc['demo_area']['s']) / 2, sum(doc['demo_area']['t']) / 2)
    h = h0 + a * sc + b * tc
    f = doc['face']
    f['base'] = [float(p + h * c) for p, c in zip(f['base'], f['normal'])]
    dst.write_text('# proto twin: ' + src.name + f' with the face plane moved {h * 1000:+.1f} mm along its normal\n'
                   + yaml.safe_dump(doc, sort_keys=False))
    return dst


def push_back(text, mesh, dist, n):
    """Wrap the Shape that draws `mesh` in a Pose shifted by -dist along the face normal n."""
    i = text.find(mesh)
    j = text.rfind('Shape {', 0, i)
    if i < 0 or j < 0:
        raise RuntimeError(f'{mesh} not found')
    depth, k = 0, j
    while True:                                               # matching brace of 'Shape {'
        c = text[k]
        depth += c == '{'
        depth -= c == '}'
        k += 1
        if depth == 0 and c == '}':
            break
    d = [-dist * v for v in n]
    return (text[:j] + f'Pose {{ translation {d[0]:.5f} {d[1]:.5f} {d[2]:.5f} children [ ' + text[j:k]
            + ' ] }  # proto twin: pushed behind the drive surface' + text[k:])


def webots_running():
    try:
        out = subprocess.check_output(['tasklist.exe', '/FI', 'IMAGENAME eq webots-bin.exe', '/NH'], timeout=10)
        return b'webots-bin.exe' in out
    except Exception:
        return False


def close_webots(context=None):
    """The Webots window is a Windows program: stopping the launch does not close it (and the next
    run would then get port 1235 while its controller connects to the old window on 1234)."""
    subprocess.run(['taskkill.exe', '/F', '/IM', 'webots-bin.exe'], capture_output=True, timeout=20)
    subprocess.run(['taskkill.exe', '/F', '/IM', 'webots.exe'], capture_output=True, timeout=20)
    return []


def face_box(layout, fit=(0.001, 0.0, 0.0)):
    """Thin collision box for the drive area, its top on h(s, t) = h0 + a s + b t above the face
    plane of the inspection layout (fit = (h0, a, b), see FACE_FIT)."""
    import numpy as np
    import yaml
    doc = yaml.safe_load(layout.read_text())
    f = doc['face']
    base, up, ac, n = (np.array(f[k], float) for k in ('base', 'up', 'across', 'normal'))
    h0, a, b = fit
    s0, s1, t0, t1 = -0.5, 5.5, -3.0, 3.0
    size = (s1 - s0, t1 - t0, 0.1)
    up2 = up + a * n
    up2 /= np.linalg.norm(up2)
    ac2 = ac + b * n
    ac2 -= (ac2 @ up2) * up2
    ac2 /= np.linalg.norm(ac2)
    n2 = np.cross(up2, ac2)
    sc, tc = (s0 + s1) / 2, (t0 + t1) / 2
    c = base + up * sc + ac * tc + n * (h0 + a * sc + b * tc) - n2 * size[2] / 2
    R = np.c_[up2, ac2, n2]
    tr = np.trace(R)                                          # rotation matrix -> quaternion -> axis-angle
    if tr > 0:
        k = 2 * np.sqrt(tr + 1)
        q = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1], k * k / 4]) / k
    else:
        i = int(np.argmax(np.diag(R)))
        j, m = (i + 1) % 3, (i + 2) % 3
        k = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[m, m])
        q = np.zeros(4)
        q[i], q[j], q[m], q[3] = k / 4, (R[j, i] + R[i, j]) / k, (R[m, i] + R[i, m]) / k, (R[m, j] - R[j, m]) / k
    v = np.linalg.norm(q[:3])
    axis = q[:3] / v if v > 1e-9 else np.array([0.0, 0.0, 1.0])
    ang = 2 * np.arctan2(v, q[3])
    return ('\nSolid {  # proto twin: flat box face for the tyre model (launch/proto.launch.py)\n'
            '  name "proto_face"\n  contactMaterial "concrete"\n'
            f'  translation {c[0]:.5f} {c[1]:.5f} {c[2]:.5f}\n'
            f'  rotation {axis[0]:.6f} {axis[1]:.6f} {axis[2]:.6f} {ang:.6f}\n'
            f'  boundingObject Box {{ size {size[0]:.3f} {size[1]:.3f} {size[2]:.3f} }}\n}}\n')


def proto_world(src, dst, dam_repo, tyre, world, layout):
    """The dam world with the prototype instead of the Wcr robot; relative asset paths re-pointed;
    the prototype's tyre contact ("proto_wheel" on concrete) added; the drive surface matched to
    the drawn face (FACE_FIT, PUSH_BACK)."""
    import re

    import yaml
    text = src.read_text()
    if world in FACE_FIT:
        n = yaml.safe_load(layout.read_text())['face']['normal']
        if world == 'goldeneye_dam':                          # the dam sim's best-fit face box: 5 cm back
            def back(mo):
                p = [float(v) - 0.05 * c for v, c in zip(mo.group(1).split(), n)]
                return f'Pose {{ translation {p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {mo.group(2)}  # face (proto twin: 5 cm back)'
            text, k = re.subn(r'Pose \{ translation ([-\d. ]+?) (rotation .*?)  # face\b', back, text)
            if k != 1:
                raise RuntimeError(f'{src}: face box not found')
        for mesh, dist in PUSH_BACK.get(world, {}).items():
            text = push_back(text, mesh, dist, n)
        text += face_box(layout, FACE_FIT[world])
    text = text.replace('contactProperties [', 'contactProperties [\n' + tyre.rstrip('\n'), 1)
    for key in ('EXTERNPROTO "../protos/Wcr.proto"', '\nWcr {', 'controllerArgs [ "keyboard" ]'):
        if key not in text:
            raise RuntimeError(f'{src}: "{key.strip()}" not found')
    text = text.replace('EXTERNPROTO "../protos/Wcr.proto"', 'EXTERNPROTO "../protos/WcrProto.proto"')
    text = text.replace('\nWcr {', '\nWcrProto {').replace('controllerArgs [ "keyboard" ]', 'controller "<extern>"')
    rel = os.path.relpath(dam_repo / 'wcr_description', dst.parent).replace(os.sep, '/')
    text = text.replace('"../../wcr_description/', f'"{rel}/')
    text = text.replace('info [ "WCR wall-climbing robot', 'info [ "WCR PROTOTYPE twin')
    dst.parent.mkdir(parents=True, exist_ok=True)
    dst.write_text(text)
    # Webots restores a saved window layout per world (.<world>.wbproj), sometimes with the 3D view
    # hidden; this world is regenerated every run, so always start with the default layout
    dst.with_name(f'.{dst.stem}.wbproj').unlink(missing_ok=True)


def setup(context):
    share = Path(get_package_share_directory('wcr_proto'))
    repo = Path(os.path.realpath(share / 'resource' / 'wcr_proto.urdf')).parents[3]     # symlink-install -> checkout
    dam_share = Path(get_package_share_directory('wcr_webots'))
    dam_repo = Path(os.path.realpath(dam_share / 'resource' / 'wcr.urdf')).parents[2]
    world = LaunchConfiguration('world').perform(context)
    if world not in WORLDS:
        raise RuntimeError(f'world:={world}: the prototype twin runs on {", ".join(WORLDS)}')
    dst = repo / 'webots' / 'worlds' / f'_proto_{world}.wbt'
    mu = LaunchConfiguration('tyre_mu').perform(context)
    fds = LaunchConfiguration('tyre_fds').perform(context)
    tyre = (f'    ContactProperties {{ material1 "proto_wheel" material2 "concrete" coulombFriction [ {mu} ] '
            f'forceDependentSlip [ {fds} ] softCFM 1e-5 }}\n')
    layout = dam_repo / 'wcr_description' / 'worlds' / 'inspection' / f'{world}.yaml'
    proto_world(dam_repo / 'webots' / 'worlds' / f'{world}.wbt', dst, dam_repo, tyre, world, layout)
    if world in FACE_FIT:                   # the map plane (wall_frames, coverage, Foxglove) on the drive surface
        layout = shifted_layout(layout, FACE_FIT[world], dst.with_name(f'_proto_{world}_layout.yaml'))
    wall_map = dam_repo / 'wcr_description' / 'worlds' / 'maps' / f'{world}.yaml'
    results = repo / 'results'
    results.mkdir(exist_ok=True)

    mission = LaunchConfiguration('mission').perform(context) == 'true'
    nav = mission or LaunchConfiguration('nav').perform(context) == 'true'
    os.environ['WCR_ODOM_TF'] = '0' if nav else '1'             # with Nav2 the EKF publishes odom -> base_link
    speed = float(LaunchConfiguration('speed').perform(context))
    os.environ['WCR_MAX_SPEED'] = str(speed)
    os.environ['WCR_THROTTLE'] = LaunchConfiguration('throttle').perform(context)

    os.environ.setdefault('WEBOTS_HOME', os.environ.get('ROS2_WEBOTS_HOME', WEBOTS_HOME_WIN))
    import webots_ros2_driver.utils as wr_utils
    from webots_ros2_driver.webots_controller import WebotsController
    # webots_ros2 takes the Windows host from resolv.conf; with WSL DNS tunnelling it is the gateway
    gateway = subprocess.check_output(['ip', 'route']).decode().split('default via ')[1].split()[0]
    wr_utils.get_wsl_ip_address = lambda: gateway

    actions = []
    if LaunchConfiguration('start_webots').perform(context) == 'true':
        if webots_running():
            raise RuntimeError('Webots is already running: close it first (one simulation at a time; '
                               'a second window would take port 1235 and the controller would drive the old one), '
                               'or pass start_webots:=false to use the open one')
        actions.append(RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=close_webots)])))
        exe = Path(os.environ['WEBOTS_HOME']) / 'msys64' / 'mingw64' / 'bin' / 'webots.exe'
        win_world = subprocess.check_output(['wslpath', '-w', str(dst)]).decode().strip()
        mode = 'fast' if LaunchConfiguration('fast').perform(context) == 'true' else 'realtime'
        actions.append(ExecuteProcess(cmd=[str(exe), '--port=1234', f'--mode={mode}', win_world],
                                      output='screen', name='webots'))
    actions.append(WebotsController(
        robot_name='wcr', port='1234',
        parameters=[{'robot_description': str(share / 'resource' / 'wcr_proto_driver.urdf'), 'use_sim_time': True}],
        respawn=True))
    actions.append(Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': (share / 'resource' / 'wcr_proto.urdf').read_text(), 'use_sim_time': True}]))
    actions.append(Node(                                             # reused: defects in the wall camera view
        package='wcr_webots', executable='inspection_detector', output='screen',
        parameters=[{'layout': str(layout), 'log_file': str(results / f'inspection_log_{world}.csv'),
                     'use_sim_time': True}]))
    actions.append(Node(                                             # reused: world -> map (face) -> odom
        package='wcr_webots', executable='wall_frames', output='screen',
        parameters=[{'layout': str(layout), 'use_sim_time': True}]))
    if nav:
        from nav2_common.launch import RewrittenYaml
        cfg = share / 'config'
        nav2_params = RewrittenYaml(
            source_file=str(cfg / 'nav2_params.yaml'), convert_types=True,
            param_rewrites={'vx_max': str(speed), 'vx_min': str(-speed),
                            'default_nav_through_poses_bt_xml': str(dam_share / 'config' / 'navigate_through_poses_coverage.xml')}
        ).perform(context)
        actions.append(Node(
            package='robot_localization', executable='ekf_node', name='ekf_filter_node', output='screen',
            parameters=[str(cfg / 'ekf.yaml')]))
        # map server after the start-up rush (its lifecycle reply was lost under load in the dam sim)
        actions.append(TimerAction(period=6.0, actions=[
            Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
                 parameters=[nav2_params, {'yaml_filename': str(wall_map), 'use_sim_time': True}]),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_map',
                 output='screen', parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}])]))
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'navigation_launch.py')),
            launch_arguments={'params_file': nav2_params, 'use_sim_time': 'true', 'autostart': 'true'}.items()))
    if mission:
        actions.append(Node(
            package='wcr_proto', executable='coverage_mission', output='screen',
            parameters=[{'layout': str(layout), 'area': LaunchConfiguration('area').perform(context),
                         'speed': speed, 'report_dir': str(results), 'use_sim_time': True}]))
    actions.append(Node(
        package='rviz2', executable='rviz2', output='screen', condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', str(dam_share / 'resource' / 'wcr.rviz')], parameters=[{'use_sim_time': True}]))
    actions.append(Node(                                             # Foxglove: ws://localhost:8765, layout resource/wcr_proto_foxglove.json
        package='foxglove_bridge', executable='foxglove_bridge', output='screen',
        condition=IfCondition(LaunchConfiguration('foxglove')),
        parameters=[{'port': 8765, 'use_sim_time': True, 'send_buffer_limit': 50000000,
                     'capabilities': ['clientPublish', 'parameters', 'services', 'assets']}]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='winter_dam', description=' | '.join(WORLDS)),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('foxglove', default_value='true', description='foxglove_bridge on ws://localhost:8765'),
        DeclareLaunchArgument('start_webots', default_value='true'),
        DeclareLaunchArgument('throttle', default_value='full',
                              description="'2'..'7' servo-tester setting (measured thrust), 'full' (~850 g), 'auto'"),
        DeclareLaunchArgument('speed', default_value='0.11',
                              description='max driving speed [m/s]; 0.11 = the N20s at 60 rpm, more = faster demo'),
        DeclareLaunchArgument('fast', default_value='false', description='run Webots as fast as possible'),
        DeclareLaunchArgument('tyre_mu', default_value='0.35 0.45',
                              description='tyre-concrete friction "side roll". Webots friction is a pyramid: a 4WD skid steer with '
                                          'wheelbase >= track only turns if side < roll x track / wheelbase. With this asymmetric '
                                          'setting the tyres grip ~1.6x the coefficient, so 0.35 / 0.45 behaves like rubber on '
                                          'concrete (mu ~0.7): checked with tools/wsl_hold_test.sh against the hand calculation'),
        DeclareLaunchArgument('tyre_fds', default_value='0.02 0',
                              description='tyre force-dependent slip "side roll" [m/s/N]: the rubber creeps sideways under '
                                          'load, so the skid steer turns slowly instead of locking'),
        DeclareLaunchArgument('nav', default_value='false', description='EKF + wall map + Nav2 (goals from Foxglove)'),
        DeclareLaunchArgument('mission', default_value='false', description='nav + autonomous coverage, report in results/'),
        DeclareLaunchArgument('area', default_value='demo', description="'demo' or 's0,s1,t0,t1' in map metres"),
        OpaqueFunction(function=setup),
    ])
