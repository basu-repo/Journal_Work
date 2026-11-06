#!/usr/bin/env python3
"""Load any configured terrain and robot, optionally with a selected navigator.

Examples:
    python3 run_simulation.py --map bayland --robot a300
    python3 run_simulation.py --map sandHeightMap --robot a200 --model nav2
    python3 run_simulation.py --map warehouse --robot a300 --model astar
    python3 run_simulation.py --list
    python3 run_simulation.py --map warehouse --robot a200 --check

The saved world supplies the spawn marker; the robot SDF supplies the driving
topic and ground clearance. Generated world / GUI files live only in a temporary
directory. Source worlds and robots are never overwritten. Manual mode launches
the simulation without autonomous navigation. Nav2 or A* mode starts the autonomous stack and saves logs. Neither
mode enables camera following / automatic robot focus. Autonomous modes
record a compressed ROS bag in bag/<run-id> by default.

Autonomous modes default to --localization gnss (wheel/IMU EKFs with GNSS correction).
Use --localization slam for odometry with RTAB-Map map correction when available. GNSS mode
uses the selected terrain datum and known simulation spawn heading; it pauses
commands when GNSS fusion is unhealthy. RTAB-Map builds a LiDAR occupancy map from fused poses in this mode.
It does not publish pose TF when GNSS/EKF owns localization.
--gnss-mode periodic is an opt-in autonomous-navigation experiment: stop for a fresh
GNSS burst, follow the current global route, gate fixes during up to 3 m
of wheel/IMU coasting, then stop and reacquire. The raw Gazebo GNSS sensor
remains active; only fixes passed to localization are gated.

A* consumes the live SLAM occupancy map for route search and checks each
velocity rollout against fresh LiDAR points. Its planning copy reconciles nearby
occupied cells only when adjacent current scan rays confirm free space; it does
not alter the saved SLAM map. It has no rolling local costmap or short
route commitment. Unknown SLAM cells remain possible global route candidates;
the live LiDAR check still rejects detected obstacles. A* loads no world
obstacle geometry or trained policy.
The robot navigation.yaml controls usable LiDAR range; astar/settings.yaml
controls route and trajectory settings.
A* uses distance, unknown-space and obstacle-clearance costs. It checks the
remaining route against new SLAM maps and replans when invalid. Time-limited
searches may return partial paths. /astar/path shows the route proposal.
The model does not start Nav2 servers.
Graphical autonomous runs open RTAB-Map's own viewer for the RTAB-Map backend,
or RViz for slam_toolbox. Use --no-costmap-view to disable the viewer.
Closing it does not stop navigation.
Every autonomous run gets an unused ROS domain by default; --ros-domain selects
an explicit unoccupied domain. Shutdown also stops children of exited wrappers.
Every autonomous run saves planned_paths.jsonl (changed global/local routes),
actual_path.csv (timestamped Gazebo truth), path_comparison.png and
path_summary.json alongside raw logs. Interrupted runs retain partial paths.
--score also exports these artifacts for older logs, labeling reconstructed plans.
"""

import argparse
import contextlib
import fcntl
import random
import math
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import time
import json
import sys
from datetime import datetime
import yaml
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parent


def pose(element):
    p = element.find('pose')
    if p is not None and (p.get('degrees', 'false') == 'true'
                          or p.get('rotation_format', 'euler_rpy') != 'euler_rpy'):
        raise ValueError('Spawn and clearance poses must use Euler angles in radians.')
    values = [float(v) for v in (p.text if p is not None else '0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('Expected six finite pose coordinates.')
    return values


def clearance(model, drive):
    """Read vertical ground offset from footprint frames or wheel joint children.

    Existing A200/A300 clearance frame chains are unrotated. Reject other frame
    conventions instead of silently putting a robot through the ground.
    """
    entities = {e.get('name'): e for e in model if e.tag in ('link', 'joint', 'frame')}

    def height(name, seen=()):
        if name in ('', '__model__', model.get('name')):
            return 0.0
        if name in seen or name not in entities:
            raise ValueError(f'Cannot resolve clearance frame {name}; use --spawn-height.')
        e = entities[name]
        xyzrpy = pose(e)
        if any(abs(v) > 1e-8 for v in xyzrpy[3:]):
            raise ValueError('Rotated clearance frames require explicit --spawn-height.')
        p = e.find('pose')
        default = e.get('attached_to', '__model__') if e.tag == 'frame' else '__model__'
        if e.tag == 'joint':
            default = e.findtext('child', '__model__')
        parent = p.get('relative_to', default) if p is not None else default
        return xyzrpy[2] + height(parent, (*seen, name))

    if 'base_footprint' in entities:
        return -height('base_footprint')
    radius = float(drive.findtext('wheel_radius', 'nan'))
    wheels = [entities[j.text].findtext('child')
              for j in (*drive.findall('left_joint'), *drive.findall('right_joint'))]
    if not wheels or not math.isfinite(radius) or radius <= 0:
        raise ValueError('Cannot derive wheel clearance; use --spawn-height.')
    return max(radius - height(wheel) for wheel in wheels)


def ensure_sensor_systems(world, model):
    """Enable Gazebo producers required by the selected robot's SDF sensors.

    Bridges transport messages; they cannot create the sensor measurements.
    Preserve world-specific plugin settings and add only missing systems.
    """
    systems = {
        'gpu_lidar': ('Sensors', 'sensors'),
        'gpu_ray': ('Sensors', 'sensors'),
        'camera': ('Sensors', 'sensors'),
        'depth_camera': ('Sensors', 'sensors'),
        'rgbd_camera': ('Sensors', 'sensors'),
        'imu': ('Imu', 'imu'),
        'navsat': ('NavSat', 'navsat'),
        'gps': ('NavSat', 'navsat'),
        'contact': ('Contact', 'contact'),
        'magnetometer': ('Magnetometer', 'magnetometer'),
        'air_pressure': ('AirPressure', 'air-pressure'),
    }
    required = {systems[s.get('type')] for s in model.iter('sensor')
                if s.get('type') in systems}
    for name, library in sorted(required):
        filename = f'gz-sim-{library}-system'
        exists = any(p.get('name', '').split('::')[-1] == name or
                     filename in p.get('filename', '')
                     for p in world.findall('plugin'))
        if not exists:
            plugin = ET.SubElement(world, 'plugin',
                                   filename=filename, name=f'gz::sim::systems::{name}')
            if name == 'Sensors':
                ET.SubElement(plugin, 'render_engine').text = 'ogre2'
            print(f'Enabled robot sensor system: {name}', flush=True)
    if ('NavSat', 'navsat') in required and world.find('spherical_coordinates') is None:
        raise ValueError('GNSS needs spherical_coordinates in the selected terrain world.sdf')


def prepare(args, directory):
    world_path = ROOT / 'terrain' / args.map / 'world.sdf'
    robot_dir = ROOT / 'robot' / args.robot
    tree = ET.parse(world_path)
    world = tree.getroot().find('world')
    if getattr(args, 'model', 'manual') != 'manual' and args.localization == 'slam':
        # Build a throwaway robot model for GNSS-denied runs. The saved A200/A300
        # files remain intact, and Gazebo cannot publish a fix from this robot.
        runtime_robot_dir = directory / 'robot_without_gnss'
        shutil.copytree(robot_dir, runtime_robot_dir)
        robot_dir = runtime_robot_dir
        model_tree = ET.parse(robot_dir / 'model.sdf')
        for link in model_tree.findall('.//link'):
            for sensor in list(link.findall('sensor')):
                if sensor.get('type') in ('navsat', 'gps'):
                    link.remove(sensor)
        if args.fastlio_imu_rate is not None:
            imu_sensors=model_tree.findall(".//sensor[@type='imu']")
            if len(imu_sensors)!=1:
                raise ValueError('FAST-LIO2 IMU-rate experiment requires exactly one IMU sensor')
            update_rate=imu_sensors[0].find('update_rate')
            if update_rate is None:
                update_rate=ET.SubElement(imu_sensors[0],'update_rate')
            update_rate.text=str(args.fastlio_imu_rate)
            print(f'Experiment-only runtime IMU rate: {args.fastlio_imu_rate:g} Hz',flush=True)
        model_tree.write(robot_dir / 'model.sdf', encoding='utf-8', xml_declaration=True)
        print('GNSS denied: runtime robot has no NavSat sensor.', flush=True)
    model = ET.parse(robot_dir / 'model.sdf').getroot().find('model')

    if world is None or model is None:
        raise ValueError('Expected an SDF world and an SDF robot model.')
    ensure_sensor_systems(world, model)

    spawn = next((m for m in world.findall('model') if m.get('name') == args.spawn), None)

    if spawn is None:
        raise ValueError(f'No model named {args.spawn!r} in {world_path}.')
    spawn_pose = spawn.find('pose')

    if spawn_pose is not None and spawn_pose.get('relative_to', '') not in ('', 'world', '__world__'):
        raise ValueError('The spawn marker must have a world-relative pose.')
    drives = [p for p in model.findall('plugin') if 'DiffDrive' in p.get('name', '')]

    if len(drives) != 1:
        raise ValueError('Expected one DiffDrive plugin in the robot SDF.')
    
    name = args.robot

    occupied = [m.get('name') for m in world.findall('model')]

    occupied += [m.findtext('name') for m in world.findall('include')]

    if name in occupied:
        raise ValueError(f'The saved world already contains {name!r}.')
    topic = drives[0].findtext('topic', f'/model/{name}/cmd_vel').strip()

    if not topic:
        raise ValueError('Empty robot command topic.')
    coordinates = pose(spawn)

    offset = args.spawn_height if args.spawn_height is not None else clearance(model, drives[0])

    if not math.isfinite(offset) or offset < 0:
        raise ValueError('Spawn height must be finite and nonnegative.')
    
    if getattr(args, 'model', 'manual') != 'manual':
        targets = mission_markers(world, getattr(args, 'targets', None))
        coordinates[5] = math.atan2(targets[0][1][1]-coordinates[1], targets[0][1][0]-coordinates[0])

    coordinates[2] += offset + 0.03  # Small settling gap above the saved marker.

    # Preserve local world assets when placing the runtime world in /tmp.
    for uri in world.iter('uri'):
        value = (uri.text or '').strip()
        if value and '://' not in value and not Path(value).is_absolute():
            uri.text = str((world_path.parent / value).resolve())
    include = ET.SubElement(world, 'include')
    ET.SubElement(include, 'uri').text = robot_dir.as_uri()
    ET.SubElement(include, 'name').text = name
    ET.SubElement(include, 'pose').text = ' '.join(map(str, coordinates))
    for gui in world.findall('gui'):
        world.remove(gui)  # Use the generated GUI configuration below.
    runtime_world = directory / 'world.sdf'
    tree.write(runtime_world, encoding='utf-8', xml_declaration=True)

    candidates = sorted(Path.home().glob('.gz/sim/*/gui.config'),
                        key=lambda p: int(p.parent.name) if p.parent.name.isdigit() else -1)
    template = args.gui_config or (candidates[-1] if candidates else None)

    if template is None:
        raise ValueError('Open Gazebo once to create its GUI config, or pass --gui-config.')
    raw = re.sub(r'<\?xml[^>]*\?>', '', Path(template).read_text())

    gui = ET.fromstring('<config>' + raw + '</config>')
    for plugin in list(gui.findall('plugin')):
        if any(name in plugin.get('filename', '')
               for name in ('Teleop', 'ResourceSpawner')):
            gui.remove(plugin)
    if not any('MarkerManager' in plugin.get('filename', '') for plugin in gui.findall('plugin')):
        ET.SubElement(gui, 'plugin', filename='MarkerManager', name='Navigation overlays')
    scene = next((p for p in gui.findall('plugin') if p.get('filename') == 'MinimalScene'), None)

    if scene is None:
        raise ValueError('GUI template needs the Gazebo MinimalScene plugin.')
    # Initial overview of the mission markers, never a robot tracking request.
    markers = [pose(m) for m in world.findall('model')
               if m.get('name') == args.spawn or m.get('name', '').startswith(('waypoint', 'goal'))]
    x0, x1 = min(p[0] for p in markers), max(p[0] for p in markers)
    y0, y1 = min(p[1] for p in markers), max(p[1] for p in markers)
    camera = scene.find('camera_pose')

    if camera is None:
        camera = ET.SubElement(scene, 'camera_pose')
    camera.text = f'{(x0+x1)/2} {(y0+y1)/2} {max(p[2] for p in markers)+max(25, 2*max(x1-x0,y1-y0))} 0 {math.pi/2} {math.pi/2}'
    
    for following in list(gui.iter('follow')):
        following.text = ''
    for control in gui.findall("plugin[@filename='WorldControl']"):
        paused = control.find('start_paused')
        if paused is not None:
            paused.text = str(args.paused).lower()
    runtime_gui = directory / 'gui.config'
    runtime_gui.write_text('<?xml version="1.0"?>\n' + '\n'.join(ET.tostring(e, encoding='unicode') for e in gui))

    paths = [ROOT / 'terrain', ROOT / 'robot', world_path.parent, robot_dir]
    paths += sorted({p.parent for p in world_path.parent.rglob('model.config')})
    paths += sorted({p.parent.parent for p in world_path.parent.rglob('model.config')})
    env = os.environ.copy()
    for key in ('GZ_SIM_RESOURCE_PATH', 'SDF_PATH'):
        env[key] = os.pathsep.join(dict.fromkeys(map(str, paths))) + (os.pathsep + env[key] if env.get(key) else '')
    print(f'Map: {args.map} | Robot: {args.robot} | Drive topic: {topic}', flush=True)
    print(f'Spawn: {coordinates} | Camera: free overview', flush=True)
    return runtime_world, runtime_gui, env


def active_gazebo():
    """Avoid accidentally opening another simulation; never terminate an existing one."""
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            cmd = (proc / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
            if re.search(r'(^|/|\s)gz sim(?:\s|$)', cmd) and ' --help' not in cmd:
                return proc.name
        except (OSError, PermissionError):
            pass
    return None


def mission_markers(world, selected=None):
    """Read only named mission markers; no obstacle geometry is exposed to navigation."""
    markers = {m.get('name'): m for m in world.findall('model')}
    if selected is None:
        names = sorted((n for n in markers if re.fullmatch(r'waypoint_?\d+', n)),
                       key=lambda n: int(re.search(r'\d+$', n).group()))
        if 'goal' not in markers:
            raise ValueError('Selected terrain needs a goal marker')
        names.append('goal')
    else:
        names = selected
    if not names:
        raise ValueError('No mission targets selected')
    result=[]
    for name in names:
        if name not in markers:
            raise ValueError(f'Mission marker {name!r} is missing')
        element=markers[name]; element_pose=element.find('pose')
        if element_pose is not None and element_pose.get('relative_to','') not in ('','world','__world__'):
            raise ValueError('Mission markers must use world-relative poses')
        result.append((name,pose(element)))
    return result


def model_settings(path, seen=()):
    """Read model defaults with an explicit relative base; reject inheritance loops."""
    path=path.resolve()
    if path in seen:raise ValueError('Model settings inheritance loop')
    settings=yaml.safe_load(path.read_text())
    base=settings.pop('extends',None)
    return {**(model_settings(path.parent/base,(*seen,path)) if base else {}),**settings}


def navigation_files(args, world_path, directory):
    from robot.adapter import describe
    from ament_index_python.packages import get_package_prefix
    packages=[args.slam_backend if args.slam_backend=='slam_toolbox' else 'rtabmap_slam',
              'rtabmap_util','pointcloud_to_laserscan','robot_state_publisher','ros_gz_bridge']
    if args.pose_backend=='kiss_icp':packages.append('kiss_icp')
    if args.slam_backend=='rtabmap' and (args.costmap_view if args.costmap_view is not None else not args.headless):
        packages+=['rtabmap_viz','rviz2']
    if args.model=='nav2':packages+=['nav2_controller','nav2_bringup','nav2_collision_monitor']
    for package in packages:
        get_package_prefix(package)
    settings=model_settings(ROOT/args.model/'settings.yaml')
    # Optional experiment-only overrides. None preserves the selected model's
    # frozen defaults, so existing commands and baselines are unchanged.
    if args.slam_scan_matching is not None:
        settings['slam_use_scan_matching']=args.slam_scan_matching
    if args.slam_loop_closing is not None:
        settings['slam_do_loop_closing']=args.slam_loop_closing
    settings['odom_frame']='odom_lidar' if args.pose_backend in ('kiss_icp','fast_lio2') else 'odom'
    settings['localization_mode']=args.localization
    if args.model=='LocalizationTest':settings['localization_test_profile']=args.localization_profile
    effective_gnss_mode=args.gnss_mode if args.localization=='gnss' else 'denied'
    settings['gnss_mode']=effective_gnss_mode
    if args.startup_delay is not None:settings['startup_delay']=args.startup_delay
    robot_dir=ROOT/'robot'/args.robot
    cfg=describe(robot_dir,args.robot,directory/'robot.urdf')
    cfg['odom_frame']=settings['odom_frame']
    # Navigation must consume the fused map-frame estimate whenever GNSS
    # localization is selected.  KISS remains the local odometry input to the
    # EKF; subscribing the planner directly to it would bypass every target
    # GNSS correction and can expose delayed LiDAR timestamps as stale pose.
    cfg['odom_topic']=('/odometry/global' if args.localization=='gnss' else
                       '/odometry/fused_local' if args.pose_backend=='kiss_icp' and args.local_ekf else
                       '/kiss/odometry' if args.pose_backend=='kiss_icp' else
                       '/fastlio/odometry_base' if args.pose_backend=='fast_lio2' else '/odom')
    cfg['pose_backend']=args.pose_backend
    if args.model in ('DstarLattice','DstarLiteLattice','DijkstraLattice'):
        from DstarLattice.primitives import compile_library
        primitive_file=directory/'motion_primitives.json'
        compile_library(cfg,settings,primitive_file)
        settings['primitive_file']=str(primitive_file)
    source=ET.parse(ROOT/'terrain'/args.map/'world.sdf').getroot().find('world')
    spawn=pose(next(m for m in source.findall('model') if m.get('name')==args.spawn))
    targets=mission_markers(source,args.targets)
    yaw=math.atan2(targets[0][1][1]-spawn[1],targets[0][1][0]-spawn[0])
    route=[]
    for index,(name,p) in enumerate(targets):
        dx,dy=p[0]-spawn[0],p[1]-spawn[1]
        next_p=targets[index+1][1] if index+1<len(targets) else p
        heading=math.atan2(next_p[1]-p[1],next_p[0]-p[0])-yaw if index+1<len(targets) else p[5]-yaw
        route.append(dict(name=name,x=math.cos(yaw)*dx+math.sin(yaw)*dy,
                          y=-math.sin(yaw)*dx+math.cos(yaw)*dy,yaw=heading))
    points=[(0.,0.)]+[(p['x'],p['y']) for p in route]
    extent=max(max(p[i] for p in points)-min(p[i] for p in points) for i in (0,1))
    cfg['localization_mode']=args.localization
    localization_config=None
    if args.localization=='gnss':
        from robot.localization import build as build_localization
        localization_config=build_localization(directory,cfg,source,spawn[:5]+[yaw],args.gnss_mode,args.pose_backend)
    from robot.mapping import build as build_mapping
    build_mapping(directory,cfg,settings,args.slam_backend)
    from robot.mapping import build_view
    if args.slam_backend=='slam_toolbox':
        build_view(directory/'costmap.rviz',cfg,settings,args.model)
    else:
        build_view(directory/'robot_view.rviz',cfg,settings,args.model,robot_view=True)
    if args.model=='nav2':
        from nav2.config import build
        build(directory/'navigation.yaml',cfg,extent,settings)
    bridge=yaml.safe_load((robot_dir/'bridge.yaml').read_text())
    if args.localization=='slam':
        bridge=[item for item in bridge if item.get('ros_type_name')!='sensor_msgs/msg/NavSatFix'
                and item.get('gz_type_name')!='gz.msgs.NavSat']
    if args.localization=='gnss':
        bridge=[b for b in bridge if b['ros_topic_name']!='/tf']
        for b in bridge:
            if b['ros_topic_name']=='/odom':b['ros_topic_name']='/wheel/odometry'
    elif args.pose_backend in ('kiss_icp','fast_lio2'):
        # KISS owns odom_lidar -> base_link.  Publishing the simulated wheel
        # odom -> base_link transform at the same time gives base_link two TF
        # parents and makes map/robot lookups nondeterministic.  Keep /odom as
        # diagnostic data, but do not bridge its TF.
        bridge=[b for b in bridge if b['ros_topic_name']!='/tf']
    for b in bridge:
        if b['direction']=='ROS_TO_GZ' and b['ros_topic_name']=='/cmd_vel':b['ros_topic_name']='/robot/cmd_vel'
    bridge.append(dict(ros_topic_name='/simulation/ground_truth/pose',gz_topic_name=f'/model/{args.robot}/pose',
                       ros_type_name='geometry_msgs/msg/PoseStamped',gz_type_name='gz.msgs.Pose',direction='GZ_TO_ROS',qos_profile='SENSOR_DATA'))
    (directory/'bridge.yaml').write_text(yaml.safe_dump(bridge,sort_keys=False))
    cfg['contact_links']=[b['ros_topic_name'].rsplit('/',1)[1] for b in bridge if b['ros_topic_name'].startswith('/simulation/contacts/')]
    cfg['path_topics']={'global':'/plan'} if args.model=='nav2' else {'global':'/dijkstra_lattice/path' if args.model=='DijkstraLattice' else '/dstarlite/path' if args.model=='DstarLite' else '/dstarlitelattice/path' if args.model=='DstarLiteLattice' else '/dstarlattice/path' if args.model=='DstarLattice' else '/dstar/path' if args.model=='Dstar' else '/astar/path'}
    (directory/'robot.yaml').write_text(yaml.safe_dump(cfg,sort_keys=False))
    (directory/'state_publisher.yaml').write_text(yaml.safe_dump({'robot_state_publisher':{'ros__parameters':{'use_sim_time':True,'robot_description':(directory/'robot.urdf').read_text()}}}))
    scan={'use_sim_time':True,'target_frame':cfg['base_frame'],'transform_tolerance':.5,
          'min_height':cfg['scan_min_height'],'max_height':cfg['scan_max_height'],
          'angle_min':-math.pi,'angle_max':math.pi,'angle_increment':cfg['scan_angle_increment'],
          'scan_time':.1,'range_min':cfg['self_filter_radius'],'range_max':cfg['scan_max_range'],
          'use_inf':True,'inf_epsilon':1.0,'concurrency_level':1}
    (directory/'scan.yaml').write_text(yaml.safe_dump({'pointcloud_to_laserscan':{'ros__parameters':scan}}))
    if args.pose_backend=='kiss_icp':
        kiss={'use_sim_time':True,
              'data':{'deskew':False,'max_range':float(cfg['scan_max_range']),
                      'min_range':float(cfg['self_filter_radius'])},
              'mapping':{'voxel_size':.5,'max_points_per_voxel':12},
              'adaptive_threshold':{'initial_threshold':1.0,'min_motion_th':.05},
              'registration':{'max_num_iterations':50,'convergence_criterion':.001,
                              'max_num_threads':2}}
        (directory/'kiss_icp.yaml').write_text(yaml.safe_dump(
            {'kiss_icp_node':{'ros__parameters':kiss}},sort_keys=False))
    if args.localization=='slam' and args.pose_backend=='kiss_icp' and args.local_ekf:
        ekf=yaml.safe_load((ROOT/'robot'/'ekf_gnss_denied.yaml').read_text())
        if not args.kiss_yaw_fusion:
            # Ablation only: retain the baseline EKF and disable solely the
            # KISS absolute-yaw measurement. KISS still runs for diagnostics.
            ekf['ekf_gnss_denied']['ros__parameters']['odom0_config'][5]=False
        (directory/'ekf_gnss_denied.yaml').write_text(yaml.safe_dump(ekf,sort_keys=False))
        cfg['kiss_yaw_fusion']=bool(args.kiss_yaw_fusion)
    mission={**settings,'targets':route,'base_frame':cfg['base_frame'],'lidar_frame':cfg['lidar_frame']}
    (directory/'mission.yaml').write_text(yaml.safe_dump(mission,sort_keys=False))
    print('Mission:', ' -> '.join(p['name'] for p in route), flush=True)
    return {'map':args.map,'robot':args.robot,'model':args.model,'world_spawn':spawn[:5]+[yaw],
            'world_targets':dict(targets),'mission':mission,'robot_capabilities':cfg,'localization':localization_config,
            'gnss_mode':effective_gnss_mode}


def verify_simulation_arrivals(result):
    """Score saved simulation truth independently; never feed it to navigation."""
    from bisect import bisect_left
    metadata = yaml.safe_load((result / 'run.yaml').read_text())
    summary_path = result / 'summary.json'
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    events_path = result / 'events.jsonl'
    events = [json.loads(line) for line in events_path.read_text().splitlines()] if events_path.exists() else []
    truth_path = result / 'truth.jsonl'
    truth = [json.loads(line) for line in truth_path.read_text().splitlines()] if truth_path.exists() else []
    truth = sorted(truth, key=lambda sample: sample['sim_time'])
    times = [sample['sim_time'] for sample in truth]
    tolerance = float(metadata['mission']['goal_tolerance'])
    arrivals = {}
    for event in events:
        if event['event'] != 'waypoint_reached':
            continue
        name = event['target']; stamp = event['sim_time']
        entry = {'estimated_distance': event['estimated_distance'], 'sim_time': stamp,
                 'truth_distance': None, 'verified': False}
        i = bisect_left(times, stamp)
        if i < len(times) and abs(times[i] - stamp) < 1e-8:
            xy = truth[i]['xyz'][:2]
        elif 0 < i < len(times) and times[i] - times[i-1] <= 0.5:
            weight = (stamp-times[i-1])/(times[i]-times[i-1])
            xy = [truth[i-1]['xyz'][axis]*(1-weight)+truth[i]['xyz'][axis]*weight for axis in (0, 1)]
        else:
            xy = None
            entry['reason'] = 'No sufficiently close bracketing truth samples'
        if xy is not None:
            distance = math.dist(xy, metadata['world_targets'][name][:2])
            entry.update(truth_xy=xy, truth_distance=distance, verified=distance <= tolerance)
        arrivals[name] = entry
    requested = [target['name'] for target in metadata['mission']['targets']]
    estimated_success = summary.get('estimated_route_success', summary.get('route_success', False))
    verified_success = bool(estimated_success and requested and
                            all(arrivals.get(name, {}).get('verified', False) for name in requested))
    summary.update(estimated_route_success=estimated_success, route_success=verified_success,
                   arrival_basis='Independent Gazebo truth at each estimated arrival; evaluation only',
                   goal_tolerance=tolerance, simulation_arrivals=arrivals,
                   verified_completed_targets=[name for name in requested if arrivals.get(name, {}).get('verified', False)])
    summary_path.write_text(json.dumps(summary, indent=2))
    print('Estimated mission success:', estimated_success,
          '| Independently verified mission success:', verified_success, flush=True)
    for name, entry in arrivals.items():
        print(f"  {name}: estimated={entry['estimated_distance']:.3f} m, truth={entry['truth_distance']} m", flush=True)
    return verified_success


def used_ros_domains():
    """Detect domains of existing ROS/Work processes, including old orphans."""
    used=set()
    for proc in Path('/proc').glob('[0-9]*'):
        if proc.name==str(os.getpid()):continue
        try:
            command=(proc/'cmdline').read_bytes()
            if b'/opt/ros/' not in command and str(ROOT).encode() not in command:continue
            for item in (proc/'environ').read_bytes().split(b'\0'):
                if item.startswith(b'ROS_DOMAIN_ID='):used.add(int(item.split(b'=',1)[1]))
        except (OSError,ValueError):continue
    return used


@contextlib.contextmanager
def ros_domain_lease(requested=None):
    """Choose and lock an unused domain; an explicit occupied domain is an error."""
    used=used_ros_domains()
    choices=[requested] if requested is not None else random.SystemRandom().sample(range(20,200),180)
    for domain in choices:
        if domain in used:continue
        lock=Path(tempfile.gettempdir())/f'work_sim_ros_{os.getuid()}_{domain}.lock'
        stream=lock.open('a')
        try:fcntl.flock(stream,fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            stream.close();continue
        try:yield domain
        finally:stream.close()
        return
    raise ValueError('Requested ROS domain is occupied' if requested is not None else 'No unused ROS domain available')


def stop_process_groups(processes,timeout=8.):
    """Reap owned groups even if a ros2 wrapper exited before its children.

    Repeated Ctrl-C must not interrupt cleanup halfway through. Keep the bridge
    and simulator alive until command producers/watchdog have stopped.
    """
    handlers={sig:signal.signal(sig,signal.SIG_IGN) for sig in (signal.SIGINT,signal.SIGTERM)}
    def send(p,sig):
        try:os.killpg(p.pid,sig)
        except ProcessLookupError:pass
    def alive(p):
        p.poll()
        try:os.killpg(p.pid,0);return True
        except ProcessLookupError:return False
    try:
        ordered=list(reversed(processes))
        for batch,batch_timeout in (([p for name,p in ordered if name not in ('bag_record','bridge','gazebo')],timeout),
                                ([p for name,p in ordered if name=='bag_record'],max(timeout,60.)),
                                ([p for name,p in ordered if name in ('bridge','gazebo')],timeout)):
            for p in batch:send(p,signal.SIGINT)
            deadline=time.monotonic()+batch_timeout
            while any(alive(p) for p in batch) and time.monotonic()<deadline:time.sleep(.05)
            for p in batch:
                if alive(p):send(p,signal.SIGKILL)
            for p in batch:
                try:p.wait(timeout=1.)
                except subprocess.TimeoutExpired:pass
    finally:
        for sig,handler in handlers.items():signal.signal(sig,handler)


def recording_topics(directory):
    """Use the recording profile plus all topics exposed by the selected robot."""
    profile=yaml.safe_load((ROOT/'robot/bag_topics.yaml').read_text())
    bridge=yaml.safe_load((directory/'bridge.yaml').read_text())
    robot=yaml.safe_load((directory/'robot.yaml').read_text())
    topics=list(profile['topics'])
    excluded=tuple(profile.get('exclude_bridge_prefixes',[]))
    topics += [entry['ros_topic_name'] for entry in bridge
               if not entry['ros_topic_name'].startswith(excluded)]
    topics += list(robot.get('path_topics',{}).values())
    if any(not isinstance(topic,str) or not topic.startswith('/') for topic in topics):
        raise ValueError('Recording profile contains an invalid topic')
    return list(dict.fromkeys(topics))


def summarize_bag(result, bag_path):
    """Inspect finalized rosbag metadata so an empty bag is never reported as useful."""
    metadata=bag_path/'metadata.yaml'
    report={'path':str(bag_path),'status':'missing','message_count':0,'topics':{}}
    if metadata.exists():
        info=yaml.safe_load(metadata.read_text()).get('rosbag2_bagfile_information',{})
        report['topics']={entry['topic_metadata']['name']:entry['message_count']
                          for entry in info.get('topics_with_message_count',[])}
        report['message_count']=sum(report['topics'].values())
        report['status']='recorded' if report['message_count'] else 'empty'
        report['storage_identifier']=info.get('storage_identifier')
    log=result/'bag_record.log'
    lost=re.findall(r'Number of messages lost on the transport layer: (\d+)',
                    log.read_text() if log.exists() else '')
    report['transport_messages_lost']=sum(map(int,lost))
    if report['status']=='recorded' and report['transport_messages_lost']:
        report['status']='recorded_with_losses'
    (result/'bag_status.json').write_text(json.dumps(report,indent=2))
    return report


def run_navigation(args, world, gui, env, directory, metadata):
    result=ROOT/'result'/datetime.now().astimezone().strftime('%Y%m%d_%H%M%S_%f')
    result.mkdir(parents=True)
    if args.slam_backend=='rtabmap':
        config_path=directory/'rtabmap.yaml'
        config=yaml.safe_load(config_path.read_text())
        config['rtabmap']['ros__parameters']['database_path']=str(result/'rtabmap.db')
        config_path.write_text(yaml.safe_dump(config,sort_keys=False))
    bag_path=ROOT/'bag'/result.name
    if args.record_bag:(ROOT/'bag').mkdir(exist_ok=True)
    if args.record_bag and shutil.disk_usage(ROOT/'bag').free < args.min_free_gb*1024**3:
        raise ValueError(f'Less than {args.min_free_gb:g} GiB free for a complete sensor bag')
    topics=recording_topics(directory) if args.record_bag else []
    metadata={**metadata,'ros_domain_id':args.ros_domain,'slam_backend':args.slam_backend,
              'pose_backend':args.pose_backend,
              'slam_input':'/lidar/points' if args.slam_backend=='rtabmap' else '/scan',
              'obstacle_input':'/lidar/points',
              'safety_scan_source':'/lidar/obstacles',
              'recording':{'enabled':args.record_bag,'topics':topics,'path':str(bag_path) if args.record_bag else None,
                           'min_free_gb':args.min_free_gb}}
    (result/'run.yaml').write_text(yaml.safe_dump(metadata,sort_keys=False))
    env=env.copy();env['ROS_LOG_DIR']=str(result/'ros_logs');env['PYTHONUNBUFFERED']='1'
    env['PYTHONPATH']=str(ROOT)+os.pathsep+env.get('PYTHONPATH','')
    # Keep the selected simulation and ROS stack isolated from other experiments.
    env['GZ_PARTITION']='work_navigation_'+str(os.getpid())
    env['ROS_DOMAIN_ID']=str(args.ros_domain)
    processes=[];streams=[]
    def launch(name,command):
        stream=(result/(name+'.log')).open('w');streams.append(stream)
        p=subprocess.Popen(command,env=env,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        processes.append((name,p));return p
    def rosrun(package,node,params=None,extra=()):
        command=['ros2','run',package,node]
        if params or extra:command+=['--ros-args']
        if params:command+=['--params-file',str(params)]
        return command+list(extra)
    old=signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    start=time.monotonic();outcome='interrupted';code=130
    try:
        sim=['gz','sim',str(world),'-r']
        sim+=['-s','--headless-rendering'] if args.headless else ['--gui-config',str(gui)]
        gz=launch('gazebo',sim)
        launch('bridge',rosrun('ros_gz_bridge','parameter_bridge',extra=['-p','config_file:='+str(directory/'bridge.yaml')]))
        if args.record_bag:
            launch('bag_record',['ros2','bag','record','-o',str(bag_path),'-s','mcap',
                                 '--storage-preset-profile','zstd_fast',
                                 '--disable-keyboard-controls','--use-sim-time','--topics',*topics])
        launch('clock_trace',[sys.executable,str(ROOT/'robot/clock_trace.py'),'--output',str(result),
                              '--ros-args','-p','use_sim_time:=true'])
        launch('robot_state',rosrun('robot_state_publisher','robot_state_publisher',directory/'state_publisher.yaml'))
        if args.pose_backend=='kiss_icp':
            xyz=metadata['robot_capabilities']['lidar_mount_xyz']
            rpy=metadata['robot_capabilities']['lidar_mount_rpy']
            launch('lidar_static',['ros2','run','tf2_ros','static_transform_publisher',
                   '--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),
                   '--roll',str(rpy[0]),'--pitch',str(rpy[1]),'--yaw',str(rpy[2]),
                   '--frame-id',metadata['robot_capabilities']['base_frame'],
                   '--child-frame-id',metadata['robot_capabilities']['lidar_frame'],
                   '--ros-args','-p','use_sim_time:=true'])
        launch('obstacles3d',rosrun('rtabmap_util','obstacles_detection',directory/'obstacles3d.yaml',
               ['-r','__node:=obstacles_detection','-r','cloud:=/lidar/points',
                '-r','obstacles:=/lidar/obstacles','-r','ground:=/lidar/ground']))
        launch('scan',rosrun('pointcloud_to_laserscan','pointcloud_to_laserscan_node',directory/'scan.yaml',
               ['-r','__node:=pointcloud_to_laserscan','-r','cloud_in:=/lidar/obstacles','-r','scan:=/scan']))
        if args.pose_backend=='kiss_icp':
            fused_local=(args.localization=='slam' and args.local_ekf)
            launch('kiss_icp',['ros2','launch','kiss_icp','odometry.launch.py',
                   'topic:=/lidar/points','base_frame:='+metadata['robot_capabilities']['base_frame'],
                   'lidar_odom_frame:=odom_lidar','publish_odom_tf:='+str(not fused_local).lower(),
                   'position_covariance:='+('4.0' if fused_local else '0.1'),
                   'orientation_covariance:='+('0.25' if fused_local else '0.1'),
                   'invert_odom_tf:=false','visualize:=false','use_sim_time:=true',
                   'config_file:='+str(directory/'kiss_icp.yaml')])
        elif args.pose_backend=='fast_lio2':
            fast_ws=ROOT/'FASTLIO2Experiment'
            fastlio_config=('husky_a300_snapshot_sensor_matched.yaml'
                            if args.fastlio_imu_noise_profile=='sensor_matched'
                            else 'husky_a300_snapshot.yaml')
            launch('fast_lio2',['bash','-lc',
                   'source /opt/ros/jazzy/setup.bash; '
                   'source '+str(fast_ws/'install'/'setup.bash')+'; '
                   'exec ros2 launch fast_lio husky_a300_snapshot.launch.py '
                   'config_file:='+str(fast_ws/'src'/'FAST_LIO'/'config'/fastlio_config)+' '
                   'point_filter_num:='+str(args.fastlio_point_filter_num)])
            launch('fastlio_adapter',[sys.executable,str(ROOT/'robot'/'fastlio_adapter.py'),
                   '--output',str(result),'--ros-args','-p','use_sim_time:=true'])
        if args.localization=='slam' and args.pose_backend=='kiss_icp' and args.local_ekf:
            # KISS publishes measurements only; the EKF exclusively owns the
            # odom_lidar -> base_link transform.
            launch('sensor_conditioner',[sys.executable,str(ROOT/'robot'/'sensor_conditioner.py'),
                   '--output',str(result),
                   '--ros-args','-p','use_sim_time:=true'])
            launch('ekf_gnss_denied',rosrun('robot_localization','ekf_node',directory/'ekf_gnss_denied.yaml',
                   ['-r','__node:=ekf_gnss_denied','-r','odometry/filtered:=/odometry/fused_local']))
        launch('robot_watchdog',[sys.executable,str(ROOT/'robot/runtime.py'),'--config',str(directory/'robot.yaml'),'--output',str(result),'--ros-args','-p','use_sim_time:=true'])
        if args.localization=='gnss':
            launch('localization_input',[sys.executable,str(ROOT/'robot/localization_input.py'),'--config',str(directory/'localization_input.yaml'),'--output',str(result),'--ros-args','-p','use_sim_time:=true'])
            filters=([('ekf_global','/odometry/global')] if args.pose_backend=='kiss_icp'
                     else [('ekf_local','/odom'),('ekf_global','/odometry/global')])
            for name,topic in filters:
                launch(name,rosrun('robot_localization','ekf_node',directory/(name+'.yaml'),['-r','__node:='+name,'-r','odometry/filtered:='+topic]))
            launch('navsat_transform',rosrun('robot_localization','navsat_transform_node',directory/'navsat_transform.yaml',['-r','__node:=navsat_transform','-r','odometry/filtered:=/odometry/global','-r','gps/fix:=/localization/fix']))
        if args.slam_backend=='rtabmap':
            odom_topic='/odometry/global' if args.localization=='gnss' else metadata['robot_capabilities']['odom_topic']
            slam_command=rosrun('rtabmap_slam','rtabmap',directory/'rtabmap.yaml',
                ['-r','odom:='+odom_topic,'-r','scan_cloud:=/lidar/points','-r','map:=/map'])
        else:
            slam_command=['ros2','launch','slam_toolbox','online_async_launch.py','use_sim_time:=true','slam_params_file:='+str(directory/'slam.yaml')]
        slam=launch('slam',slam_command)
        slam_started_at=time.monotonic();slam_retries=0
        if args.model=='nav2':
            launch('nav2',['ros2','launch',str(ROOT/'nav2/navigation.launch.py'),'params_file:='+str(directory/'navigation.yaml')])
            command=[sys.executable,str(ROOT/'nav2/mission.py')]
        else:
            command=[sys.executable,'-m',args.model+'.node','--robot',str(directory/'robot.yaml')]
        mission=launch('mission',command+['--mission',str(directory/'mission.yaml'),'--output',str(result),'--ros-args','-p','use_sim_time:=true'])
        if not args.headless and args.model!='manual':
            launch('gazebo_overlays',[sys.executable,str(ROOT/'robot/gazebo_overlays.py'),
                   '--config',str(directory/'robot.yaml'),'--spawn',*map(str,metadata['world_spawn']),
                   '--ros-args','-p','use_sim_time:=true'])
        viewer=None;viewer_reported=False;robot_viewer=None;robot_viewer_reported=False
        if args.costmap_view if args.costmap_view is not None else not args.headless:
            if args.slam_backend=='rtabmap':
                odom_topic='/odometry/global' if args.localization=='gnss' else metadata['robot_capabilities']['odom_topic']
                viewer_command=rosrun('rtabmap_viz','rtabmap_viz',directory/'rtabmap_viz.yaml',
                    ['-r','odom:='+odom_topic,'-r','scan_cloud:=/lidar/points'])
                viewer_name='RTAB-Map'
            else:
                viewer_command=['ros2','run','rviz2','rviz2','-d',str(directory/'costmap.rviz'),
                                '--ros-args','-p','use_sim_time:=true']
                viewer_name='RViz'
            viewer=launch('map_view',viewer_command)
            print(f'{viewer_name} viewer opens alongside Gazebo. Closing it leaves navigation running.',flush=True)
            if args.slam_backend=='rtabmap':
                robot_viewer=launch('robot_view',['ros2','run','rviz2','rviz2',
                    '-d',str(directory/'robot_view.rviz'),'--ros-args','-p','use_sim_time:=true'])
                print('3D robot view opens alongside RTAB-Map: colored Husky mesh and LiDAR height colors.',flush=True)
        print('Logs:',result,flush=True)
        print('Sensors start now. Autonomous motion waits for readiness and the startup delay.',flush=True)
        event_offset=0;last_disk_check=0.
        while True:
            events=result/'events.jsonl'
            if events.exists():
                with events.open() as stream:
                    stream.seek(event_offset)
                    for line in stream:
                        event=json.loads(line)
                        if event['event']=='progress':print(f"Remaining distance: {event['distance_remaining']:.1f} m | recoveries: {event['recoveries']}",flush=True)
                        else:print('Mission:',event['event'],event.get('target',''),flush=True)
                    event_offset=stream.tell()
            if args.record_bag and time.monotonic()-last_disk_check>=1.:
                last_disk_check=time.monotonic()
                if shutil.disk_usage(ROOT/'bag').free < args.min_free_gb*1024**3:
                    outcome='disk_space_limit';code=7;print('Stopping before disk runs low',flush=True);break
            if args.wall_seconds and time.monotonic()-start>=args.wall_seconds:
                outcome='wall_time_limit';code=124;break
            if args.model in ('astar','Dijkstra','DstarLite','Dstar','DstarLattice','DstarLiteLattice','DijkstraLattice') and time.monotonic()-slam_started_at>=30.0:
                slam_log=result/('slam_retry.log' if slam_retries else 'slam.log')
                activated=slam_log.exists() and (
                    'Activating' in slam_log.read_text(errors='replace') if args.slam_backend=='slam_toolbox'
                    else ('rtabmap subscribed to' in slam_log.read_text(errors='replace').lower()
                          and '/lidar/points' in slam_log.read_text(errors='replace')))
                if not activated:
                    if slam_retries:
                        outcome='slam_activation_failed';code=8
                        print('SLAM did not activate after one restart; see',slam_log,flush=True)
                        break
                    print('SLAM configuration stalled; restarting its owned process group once.',flush=True)
                    stop_process_groups([('slam',slam)],timeout=3.)
                    processes.remove(('slam',slam))
                    slam=launch('slam_retry',slam_command)
                    slam_started_at=time.monotonic();slam_retries+=1
            if mission.poll() is not None:
                code=mission.returncode;outcome='mission_complete' if code==0 else 'mission_failed';break
            if viewer is not None and viewer.poll() is not None and not viewer_reported:
                viewer_reported=True
                print('Map viewer closed; navigation continues. Viewer log:',result/'map_view.log',flush=True)
            if robot_viewer is not None and robot_viewer.poll() is not None and not robot_viewer_reported:
                robot_viewer_reported=True
                print('Robot view closed; navigation continues. Viewer log:',result/'robot_view.log',flush=True)
            failures=[(n,p.returncode) for n,p in processes if n not in ('mission','map_view','robot_view') and p.poll() is not None]
            if failures:
                outcome='process_exited';code=4;print('Stopped component:',failures,flush=True);break
            time.sleep(.2)
    except KeyboardInterrupt:pass
    finally:
        stop_process_groups(processes)
        for stream in streams:stream.close()
        signal.signal(signal.SIGTERM,old)
        bag_report=summarize_bag(result,bag_path) if args.record_bag else None
        verified = verify_simulation_arrivals(result)
        if code == 0 and not verified:
            outcome='mission_not_verified';code=5
        if args.record_bag and bag_report['status'] in ('missing','empty'):
            print('Bag recording failed:',bag_report['status'],'|',result/'bag_record.log',flush=True)
            if code==0:outcome='recording_failed';code=6
        (result/'run_status.json').write_text(json.dumps({'outcome':outcome,'exit_code':code,'wall_seconds':time.monotonic()-start},indent=2))
        try:
            from robot.path_history import export_paths
            export_paths(result)
            print('Path comparison:',result/'path_comparison.png',flush=True)
        except Exception as exc:
            (result/'path_export_error.txt').write_text(str(exc))
            print('Path export failed; raw logs retained:',exc,flush=True)
        if bag_report is not None:
            print('Bag:',bag_report['status'],'| messages:',bag_report['message_count'],
                  '| transport losses:',bag_report['transport_messages_lost'],
                  '|',bag_path,flush=True)
        print('Run ended:',outcome,'| Logs:',result,flush=True)
    return code


def ensure_environment():
    """Source the installed ROS environment once when launched from a fresh terminal."""
    try:
        import ament_index_python
        kiss_requested='--pose-backend' in sys.argv and sys.argv[sys.argv.index('--pose-backend')+1]=='kiss_icp'
        kiss_available=not kiss_requested or (Path(__file__).resolve().parent/'pose_ws/install/setup.bash').exists() and os.environ.get('WORK_KISS_ENV_READY')
        if shutil.which('gz') and shutil.which('ros2') and kiss_available:
            return
    except ImportError:
        pass
    if os.environ.get('WORK_SIM_ENV_READY'):
        raise ValueError('ROS/Gazebo tools are unavailable after sourcing the installation')
    candidates=sorted(Path('/opt/ros').glob('*/setup.bash'))
    preferred=Path('/opt/ros')/os.environ.get('ROS_DISTRO','')/'setup.bash'
    if preferred in candidates:
        setup=preferred
    elif len(candidates)==1:
        setup=candidates[0]
    else:
        raise ValueError('Source the desired installed ROS distribution before running')
    overlay=Path(__file__).resolve().parent/'pose_ws/install/setup.bash'
    script='source "$1" >/dev/null'
    if '--pose-backend' in sys.argv and sys.argv[sys.argv.index('--pose-backend')+1]=='kiss_icp':
        if not overlay.exists():raise ValueError('KISS-ICP is not built; expected '+str(overlay))
        script+=' && source "$2" >/dev/null'
    script+=' && env -0'
    loaded=subprocess.run(['bash','-c',script,'bash',str(setup),str(overlay)],capture_output=True,check=True)
    env=dict(item.split('=',1) for item in loaded.stdout.decode().split('\0') if '=' in item)
    env['WORK_SIM_ENV_READY']='1';env['WORK_KISS_ENV_READY']='1';env['PYTHONDONTWRITEBYTECODE']='1'
    os.execve(sys.executable,[sys.executable,str(Path(__file__).resolve()),*sys.argv[1:]],env)


def main():
    maps=sorted(p.parent.name for p in (ROOT/'terrain').glob('*/world.sdf'))
    robots=sorted(p.parent.name for p in (ROOT/'robot').glob('*/model.sdf'))
    parser=argparse.ArgumentParser(description=__doc__,formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--map',choices=maps,default='bayland')
    parser.add_argument('--robot',choices=robots,default='a300')
    parser.add_argument('--model',type=lambda s:'LocalizationTest' if s.lower() in ('localizationtest','localization_test') else 'DstarLite' if s.lower()=='dstarlite' else 'DstarLiteLattice' if s.lower()=='dstarlitelattice' else 'DstarLattice' if s.lower()=='dstarlattice' else 'DijkstraLattice' if s.lower()=='dijkstralattice' else 'Dijkstra' if s.lower()=='dijkstra' else 'Dstar' if s.lower()=='dstar' else s.lower(),choices=['manual','LocalizationTest','nav2','astar','Dijkstra','DstarLite','Dstar','DstarLattice','DstarLiteLattice','DijkstraLattice'],default='manual')
    parser.add_argument('--localization',choices=['gnss','slam'],default='gnss',help='Autonomous pose source: GNSS/wheel/IMU fusion (default) or LiDAR SLAM')
    parser.add_argument('--slam-backend',choices=['rtabmap','slam_toolbox'],default='slam_toolbox',help='LiDAR mapping backend; slam_toolbox is the default')
    parser.add_argument('--slam-scan-matching',action=argparse.BooleanOptionalAction,default=None,
        help='Experiment override for SLAM Toolbox scan matching; omitted preserves model settings')
    parser.add_argument('--slam-loop-closing',action=argparse.BooleanOptionalAction,default=None,
        help='Experiment override for SLAM Toolbox loop closing; omitted preserves model settings')
    parser.add_argument('--pose-backend',choices=['wheel','kiss_icp','fast_lio2'],default='wheel',help='GNSS-denied local odometry: wheel encoders, KISS-ICP, or isolated FAST-LIO2')
    parser.add_argument('--fastlio-imu-noise-profile',choices=['baseline','sensor_matched'],default='baseline',
                        help='FAST-LIO2 IMU process-noise experiment; default preserves the validated baseline')
    parser.add_argument('--fastlio-imu-rate',type=float,default=None,
                        help='Experiment-only runtime IMU rate in Hz; saved robot model is unchanged')
    parser.add_argument('--fastlio-point-filter-num',type=int,choices=[1,2,4],default=4,
                        help='FAST-LIO2 LiDAR point stride experiment; default preserves the baseline value 4')
    parser.add_argument('--local-ekf',action=argparse.BooleanOptionalAction,default=False,
                        help='Fuse KISS-ICP, wheel velocity and IMU yaw rate for GNSS-denied localization')
    parser.add_argument('--kiss-yaw-fusion',action=argparse.BooleanOptionalAction,default=True,
                        help='GNSS-denied EKF ablation: fuse KISS yaw (default) or leave yaw to IMU integration')
    parser.add_argument('--localization-profile',choices=['stationary','straight','rotation','circle','square'],default='square',
                        help='Open-loop motion profile used only by --model LocalizationTest')
    parser.add_argument('--gnss-mode',choices=['continuous','periodic','target'],default='continuous',help='GNSS policy: continuous, periodic, or only inside each target proximity zone')
    parser.add_argument('--spawn',default='spawn')
    parser.add_argument('--spawn-height',type=float)
    parser.add_argument('--gui-config',type=Path)
    parser.add_argument('--paused',action='store_true')
    parser.add_argument('--headless',action='store_true')
    parser.add_argument('--costmap-view',action=argparse.BooleanOptionalAction,default=None,help='Open RTAB-Map viewer for RTAB-Map runs or RViz for slam_toolbox; default on for graphical runs')
    parser.add_argument('--record-bag',action=argparse.BooleanOptionalAction,default=True,help='Record selected ROS topics for autonomous runs (default: on)')
    parser.add_argument('--min-free-gb',type=float,default=20.,help='Stop autonomous recording before free disk space falls below this GiB (default: 20)')
    parser.add_argument('--wall-seconds',type=float,default=0.0,help='Optional bounded validation duration')
    parser.add_argument('--startup-delay',type=float,help='Autonomous startup delay; default is in the selected model settings.yaml')
    parser.add_argument('--targets',nargs='+',help='Optional ordered subset of saved mission markers')
    parser.add_argument('--ros-domain',type=int,default=None,help='ROS domain; default selects an unused isolated domain per run')
    parser.add_argument('--score',type=Path,help='Independently verify an existing simulation result; no simulation is started')
    parser.add_argument('--list',action='store_true')
    parser.add_argument('--check',action='store_true',help='Validate configuration without starting simulation')
    args=parser.parse_args()
    if args.score:
        verified=verify_simulation_arrivals(args.score.resolve())
        from robot.path_history import export_paths
        export_paths(args.score.resolve())
        return 0 if verified else 5
    if args.list:
        print('Maps:',', '.join(maps),'\nRobots:',', '.join(robots),'\nModels: manual, nav2, astar, Dijkstra, DstarLite, Dstar, DstarLattice, DstarLiteLattice, DijkstraLattice');return 0
    ensure_environment()
    if args.model=='manual' and args.costmap_view:parser.error('Costmap view requires an autonomous model')
    if args.model=='manual' and args.record_bag:args.record_bag=False
    if args.gnss_mode in ('periodic','target') and (args.localization!='gnss' or
            args.model not in ('astar','Dijkstra','DstarLite','Dstar','DstarLattice','DstarLiteLattice','DijkstraLattice')):
        parser.error('Periodic/target GNSS requires GNSS localization and an A* or D* model')
    if args.gnss_mode=='target' and args.pose_backend!='kiss_icp':
        parser.error('Target-zone GNSS currently requires --pose-backend kiss_icp')
    if (args.slam_scan_matching is not None or args.slam_loop_closing is not None) and not (args.localization=='slam' and args.slam_backend=='slam_toolbox'):
        parser.error('SLAM correction overrides require --localization slam --slam-backend slam_toolbox')
    if args.local_ekf and not (args.localization=='slam' and args.pose_backend=='kiss_icp'):
        parser.error('--local-ekf requires --localization slam --pose-backend kiss_icp')
    if args.fastlio_imu_noise_profile!='baseline' and args.pose_backend!='fast_lio2':
        parser.error('--fastlio-imu-noise-profile requires --pose-backend fast_lio2')
    if args.fastlio_imu_rate is not None and (args.pose_backend!='fast_lio2' or args.localization!='slam'):
        parser.error('--fastlio-imu-rate requires --localization slam --pose-backend fast_lio2')
    if args.fastlio_imu_rate is not None and (not math.isfinite(args.fastlio_imu_rate) or args.fastlio_imu_rate<=0):
        parser.error('--fastlio-imu-rate must be finite and positive')
    if args.fastlio_point_filter_num!=4 and args.pose_backend!='fast_lio2':
        parser.error('--fastlio-point-filter-num requires --pose-backend fast_lio2')
    if not args.kiss_yaw_fusion and not args.local_ekf:
        parser.error('--no-kiss-yaw-fusion requires --local-ekf')
    if args.model!='LocalizationTest' and args.localization_profile!='square':
        parser.error('--localization-profile is only valid with --model LocalizationTest')
    if args.model!='manual' and args.paused:parser.error('Autonomous mode starts physics immediately for sensor initialization')
    if args.startup_delay is not None and (not math.isfinite(args.startup_delay) or args.startup_delay<0):parser.error('Startup delay must be finite and nonnegative')
    if not math.isfinite(args.wall_seconds) or args.wall_seconds<0:parser.error('Duration must be finite and nonnegative')
    if not math.isfinite(args.min_free_gb) or args.min_free_gb<0:parser.error('Minimum free space must be finite and nonnegative')
    if args.ros_domain is not None and not 0<=args.ros_domain<=232:parser.error('ROS domain must be 0..232')
    if not shutil.which('gz'):parser.error('Source your installed ROS / Gazebo environment first')
    if not args.check:
        existing=active_gazebo()
        if existing:parser.error(f'Gazebo already running (PID {existing}); close it before starting another run')
    with tempfile.TemporaryDirectory(prefix='work_sim_') as tmp:
        directory=Path(tmp);world,gui,env=prepare(args,directory)
        metadata=navigation_files(args,world,directory) if args.model!='manual' else None
        if args.check:return subprocess.run(['gz','sdf','-k',str(world)],env=env,timeout=60).returncode
        if args.model!='manual':
            with ros_domain_lease(args.ros_domain) as domain:
                args.ros_domain=domain
                print('Isolated ROS domain:',domain,flush=True)
                return run_navigation(args,world,gui,env,directory,metadata)
        command=['gz','sim',str(world)]
        command+=['-s','--headless-rendering'] if args.headless else ['--gui-config',str(gui)]
        if not args.paused:command.append('-r')
        print('Simulation only; no autonomous navigation or recording.',flush=True)
        child=subprocess.Popen(command,env=env,start_new_session=True)
        old=signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
        try:return child.wait(timeout=args.wall_seconds or None)
        except (KeyboardInterrupt,subprocess.TimeoutExpired):return 130
        finally:
            signal.signal(signal.SIGTERM,old)
            stop_process_groups([('gazebo',child)])


if __name__=='__main__':
    try:raise SystemExit(main())
    except (ValueError,OSError,ET.ParseError,subprocess.TimeoutExpired) as exc:raise SystemExit(f'Error: {exc}')
