"""Shared LiDAR mapping configuration, independent of navigation algorithm."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory as share


def build(directory,capabilities,settings,backend='slam_toolbox'):
    base=capabilities['base_frame']
    # Use the same 3D cloud segmentation for the navigation scan with either
    # mapping backend. The scan is only a ground-plane interface for controllers.
    grid3d={'Grid/NormalsSegmentation':'true',
            'Grid/RangeMin':str(capabilities['self_filter_radius']),
            'Grid/RangeMax':str(capabilities['scan_max_range']),
            'Grid/CellSize':str(settings['costmap_resolution']),
            'Grid/MaxObstacleHeight':str(capabilities['scan_max_height'])}
    (directory/'obstacles3d.yaml').write_text(yaml.safe_dump(
        {'obstacles_detection':{'ros__parameters':{'use_sim_time':True,**grid3d}}},sort_keys=False))
    if backend=='rtabmap':
        from ament_index_python.packages import get_package_prefix
        get_package_prefix('rtabmap_slam')
        params={'use_sim_time':True,'frame_id':base,'map_frame_id':'map',
                'subscribe_scan':False,'subscribe_depth':False,'subscribe_rgb':False,
                'subscribe_scan_cloud':True,'scan_cloud_is_2d':False,'approx_sync':False,
                'publish_tf':settings.get('localization_mode')=='slam',
                'database_path':str(directory/'rtabmap.db'),
                'qos_scan':2,'qos_odom':2,
                'Reg/Strategy':'1','Reg/Force3DoF':'true',
                'Grid/Sensor':'0','Grid/3D':'false',**grid3d}
        (directory/'rtabmap.yaml').write_text(yaml.safe_dump({'rtabmap':{'ros__parameters':params}},sort_keys=False))
        viz_params={key:params[key] for key in (
            'use_sim_time','frame_id','subscribe_scan','subscribe_scan_cloud',
            'scan_cloud_is_2d','subscribe_depth','subscribe_rgb','approx_sync',
            'qos_scan','qos_odom')}
        (directory/'rtabmap_viz.yaml').write_text(yaml.safe_dump(
            {'rtabmap_viz':{'ros__parameters':viz_params}},sort_keys=False))
        return
    if backend!='slam_toolbox':raise ValueError(f'Unknown SLAM backend: {backend}')
    slam=yaml.safe_load((Path(share('slam_toolbox'))/'config/mapper_params_online_async.yaml').read_text())
    slam['slam_toolbox']['ros__parameters'].update(use_sim_time=True,base_frame=base,odom_frame=settings.get('odom_frame','odom'),map_frame='map',scan_topic='/scan',resolution=settings['costmap_resolution'],map_update_interval=settings['slam_map_update_interval'],minimum_travel_distance=.2,minimum_travel_heading=.15,minimum_time_interval=.15,max_laser_range=capabilities['scan_max_range'],transform_timeout=.5,enable_interactive_mode=False)
    if 'slam_use_scan_matching' in settings:
        slam['slam_toolbox']['ros__parameters']['use_scan_matching']=bool(settings['slam_use_scan_matching'])
    if 'slam_do_loop_closing' in settings:
        slam['slam_toolbox']['ros__parameters']['do_loop_closing']=bool(settings['slam_do_loop_closing'])
    if settings.get('localization_mode')=='gnss':
        # EKFs own map->odom->base. Build the sensor map directly from fused poses.
        slam['slam_toolbox']['ros__parameters'].update(odom_frame='map',map_frame='map',
            transform_publish_period=0.0,use_scan_matching=False,do_loop_closing=False)
    (directory/'slam.yaml').write_text(yaml.safe_dump(slam,sort_keys=False))


def build_view(output,capabilities,settings,model,robot_view=False):
    """Generate a lightweight RViz view for the selected navigation topics."""
    def topic(value,latched=False,reliability='Reliable'):
        return {'Value':value,'Depth':5,'History Policy':'Keep Last',
                'Reliability Policy':reliability,'Durability Policy':'Transient Local' if latched else 'Volatile'}
    def display(kind,name,**properties):
        return {'Class':'rviz_default_plugins/'+kind,'Name':name,'Enabled':True,'Value':True,**properties}
    nav2=model=='nav2'
    global_topic='/plan' if nav2 else '/dijkstra_lattice/path' if model=='DijkstraLattice' else '/dstarlite/path' if model=='DstarLite' else '/dstarlitelattice/path' if model=='DstarLiteLattice' else '/dstarlattice/path' if model=='DstarLattice' else '/dstar/path' if model=='Dstar' else '/astar/path'
    displays=[
        display('Map','Accumulated sensor map',Topic=topic('/map',True),Alpha=.65,
                **{'Color Scheme':'map','Draw Behind':True,'Use Timestamp':False}),
        display('PointCloud2','Live 3D LiDAR height colors',Topic=topic('/lidar/points',reliability='Best Effort'),
                Alpha=1.,Style='Points',
                **{'Color Transformer':'AxisColor','Axis':'Z','Use rainbow':True,
                   'Autocompute Value Bounds':{'Value':True,'Min Value':-1.,'Max Value':2.},
                   'Position Transformer':'XYZ','Size (Pixels)':1,'Decay Time':0,'Queue Size':5}),
        display('Path','Global planned route',Topic=topic(global_topic,not nav2),Color='40; 145; 255',Alpha=1.,
            **{'Line Style':'Billboards','Line Width':.07,'Buffer Length':1,'Pose Style':'None','Offset':{'X':0,'Y':0,'Z':.08}}),
        display('Axes','Robot pose (red axis is forward)',**{'Reference Frame':capabilities['base_frame'],'Length':.8,'Radius':.06}),
    ]
    displays.insert(0,display('RobotModel','Husky in original mesh colors',
        **{'Description Source':'Topic','Description Topic':topic('/robot_description',True),
           'Visual Enabled':True,'Collision Enabled':False,'Alpha':1.0}))
    if nav2:
        displays.insert(1,display('Map','Nav2 local costs',Topic=topic('/local_costmap/costmap',True),Alpha=.85,
            **{'Color Scheme':'costmap','Draw Behind':False,'Use Timestamp':False}))
    size=settings.get('local_costmap_size',20.) if nav2 else 20.
    view={'Class':'rviz_default_plugins/Orbit','Name':'Current View',
          'Target Frame':capabilities['base_frame'] if robot_view else '<Fixed Frame>',
          'Distance':10. if robot_view else max(size,20.)*1.5,
          'Pitch':.8,'Yaw':.8,'Near Clip Distance':.01,
          'Focal Point':{'X':0.,'Y':0.,'Z':0.}}
    config={'Panels':[{'Class':'rviz_common/Displays','Name':'Displays'}, {'Class':'rviz_common/Views','Name':'Views'}],
        'Visualization Manager':{'Class':'','Name':'root','Enabled':True,'Value':True,'Displays':displays,
            'Global Options':{'Fixed Frame':'map','Background Color':'48; 48; 48','Frame Rate':15},
            'Tools':[{'Class':'rviz_default_plugins/MoveCamera'},{'Class':'rviz_default_plugins/Select'},{'Class':'rviz_default_plugins/Measure'}],
            'Views':{'Current':view}},
        'Window Geometry':{'Width':1100,'Height':800,'Hide Left Dock':False,'Hide Right Dock':True}}
    output.write_text(yaml.safe_dump(config,sort_keys=False))
