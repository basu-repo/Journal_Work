"""Extract navigation geometry and TF from the selected local SDF, never a world."""
import itertools
import math
import warnings
import xml.etree.ElementTree as E
import numpy as np
import yaml
from scipy.spatial.transform import Rotation


def matrix(pose):
    if pose is None:
        return np.eye(4)
    values=list(map(float,(pose.text or '0 0 0 0 0 0').split()))
    if pose.get('rotation_format','euler_rpy')!='euler_rpy' or len(values)!=6:
        raise ValueError('Robot adapter requires Euler SDF poses')
    t=np.eye(4)
    t[:3,:3]=Rotation.from_euler('xyz',values[3:],degrees=pose.get('degrees')=='true').as_matrix()
    t[:3,3]=values[:3]
    return t


def describe(directory, instance, output):
    model=E.parse(directory/'model.sdf').getroot().find('model')
    cfg=yaml.safe_load((directory/'navigation.yaml').read_text())
    entities={e.get('name'):e for e in model if e.tag in ('link','frame','joint')}
    cache={'__model__':np.eye(4),model.get('name'):np.eye(4),'':np.eye(4)}
    def transform(name, trail=()):
        if name in cache:return cache[name]
        if name in trail or name not in entities:raise ValueError(f'Unresolved/cyclic robot frame: {name}')
        e=entities[name]; p=e.find('pose')
        default=e.get('attached_to','__model__') if e.tag=='frame' else '__model__'
        if e.tag=='joint':default=e.findtext('child')
        parent=p.get('relative_to',default) if p is not None else default
        cache[name]=transform(parent,(*trail,name))@matrix(p)
        return cache[name]
    base=cfg['base_frame']; origin=np.linalg.inv(transform(base))
    robot=E.Element('robot',name=instance)
    E.SubElement(robot,'link',name=base)
    known={base}
    def fixed(name,parent,t):
        if name in known:return
        E.SubElement(robot,'link',name=name);known.add(name)
        joint=E.SubElement(robot,'joint',name='frame_'+str(len(known)),type='fixed')
        E.SubElement(joint,'parent',link=parent);E.SubElement(joint,'child',link=name)
        E.SubElement(joint,'origin',xyz=' '.join(map(str,t[:3,3])),rpy=' '.join(map(str,Rotation.from_matrix(t[:3,:3]).as_euler('xyz'))))
    # Moving non-wheel joints are driven by the model's actual joint-state publisher.
    drive=next(e for e in model.findall('plugin') if e.get('name','').endswith('::DiffDrive'))
    wheel_joints={e.text for e in (*drive.findall('left_joint'),*drive.findall('right_joint'))}
    moving={e.findtext('child'):e for e in model.findall('joint') if e.get('type') in ('revolute','continuous') and e.get('name') not in wheel_joints}
    pending=[e for e in model.findall('link') if e.get('name')!=base]
    while pending:
        progressed=False
        for link in list(pending):
            name=link.get('name'); j=moving.get(name)
            if j is None:
                fixed(name,base,origin@transform(name))
            else:
                parent=j.findtext('parent')
                if parent not in known:continue
                t=np.linalg.inv(transform(parent))@transform(name)
                fixed(name,parent,t)
                out=robot.findall('joint')[-1];out.set('type','continuous');out.set('name',j.get('name'))
                E.SubElement(out,'axis',xyz=j.findtext('axis/xyz','0 0 1'))
            pending.remove(link);progressed=True
        if not progressed:raise ValueError('Cannot order robot link tree')
    # Keep the actual SDF visual meshes in the generated URDF. RViz can then
    # show the selected robot with its Collada materials while RTAB-Map remains
    # the SLAM viewer. Resolve paths from the selected robot folder, not a
    # fixed machine or model name.
    urdf_links={link.get('name'):link for link in robot.findall('link')}
    for sdf_link in model.findall('link'):
        link_name=sdf_link.get('name')
        for sdf_visual in sdf_link.findall('visual'):
            shape=sdf_visual.find('geometry')
            if shape is None:continue
            pose=sdf_visual.find('pose')
            parent=pose.get('relative_to',link_name) if pose is not None else link_name
            local=np.linalg.inv(transform(link_name))@transform(parent)@matrix(pose)
            visual=E.SubElement(urdf_links[link_name],'visual')
            with warnings.catch_warnings():
                # Equivalent Euler angles exist at gimbal lock; SciPy selects
                # one valid set for the URDF visual transform.
                warnings.filterwarnings('ignore',message='Gimbal lock detected.*')
                rpy=Rotation.from_matrix(local[:3,:3]).as_euler('xyz')
            E.SubElement(visual,'origin',
                         xyz=' '.join(map(str,local[:3,3])),
                         rpy=' '.join(map(str,rpy)))
            geometry=E.SubElement(visual,'geometry')
            mesh=shape.find('mesh')
            if mesh is not None:
                uri=mesh.findtext('uri')
                mesh_path=(directory/uri).resolve()
                if not mesh_path.is_file():raise ValueError(f'Missing robot visual mesh: {mesh_path}')
                E.SubElement(geometry,'mesh',filename=mesh_path.as_uri(),
                             scale=mesh.findtext('scale','1 1 1'))
            elif shape.find('box') is not None:
                E.SubElement(geometry,'box',size=shape.findtext('box/size'))
            elif shape.find('cylinder') is not None:
                E.SubElement(geometry,'cylinder',radius=shape.findtext('cylinder/radius'),
                             length=shape.findtext('cylinder/length'))
            elif shape.find('sphere') is not None:
                E.SubElement(geometry,'sphere',radius=shape.findtext('sphere/radius'))
            else:
                urdf_links[link_name].remove(visual)
                continue
            # DAE meshes carry their own RGB materials. Primitive SDF visuals
            # need an explicit URDF color to keep their original appearance.
            rgba=sdf_visual.findtext('material/diffuse')
            if mesh is None and rgba:
                material=E.SubElement(visual,'material',name='sdf_'+sdf_visual.get('name','visual'))
                E.SubElement(material,'color',rgba=rgba)
    sensor_frames={};sensor_topics={}
    gnss_noise=[]
    for link in model.findall('link'):
        for sensor in link.findall('sensor'):
            if sensor.get('type')=='navsat':
                gnss_noise.append({axis:float(sensor.findtext('navsat/position_sensing/'+axis+'/noise/stddev','0')) for axis in ('horizontal','vertical')})
            frame=sensor.findtext('gz_frame_id') or f'{instance}/{link.get("name")}/{sensor.get("name")}'
            p=sensor.find('pose'); parent=p.get('relative_to',link.get('name')) if p is not None else link.get('name')
            t=np.linalg.inv(transform(link.get('name')))@transform(parent)@matrix(p)
            fixed(frame,link.get('name'),t)
            optical=sensor.findtext('camera/optical_frame_id')
            if optical:
                opt=np.eye(4);opt[:3,:3]=Rotation.from_euler('xyz',[-math.pi/2,0,-math.pi/2]).as_matrix()
                fixed(optical,frame,opt)
            sensor_frames.setdefault(sensor.get('type'),[]).append(frame)
            sensor_topics[sensor.findtext('topic','')]=frame
    lidar=sensor_frames.get('gpu_lidar',[])
    if len(lidar)!=1:raise ValueError('Select a robot with one configured 3D LiDAR for this baseline')
    # Conservative XY bounds of actual collision primitives, not visual mesh extents.
    points=[]
    for link in model.findall('link'):
        for c in link.findall('collision'):
            g=c.find('geometry'); box=g.find('box'); cyl=g.find('cylinder'); sphere=g.find('sphere')
            if box is not None:half=np.array(list(map(float,box.findtext('size').split())))/2
            elif cyl is not None:half=np.array([float(cyl.findtext('radius'))]*2+[float(cyl.findtext('length'))/2])
            elif sphere is not None:half=np.array([float(sphere.findtext('radius'))]*3)
            else:raise ValueError('Collision mesh needs an explicit supported geometry adapter')
            p=c.find('pose');parent=p.get('relative_to',link.get('name')) if p is not None else link.get('name')
            t=origin@transform(parent)@matrix(p)
            for signs in itertools.product((-1,1),repeat=3):points.append((t@np.r_[half*np.array(signs),1])[:2])
    lo=np.min(points,axis=0);hi=np.max(points,axis=0)
    cfg['footprint']=[[float(lo[0]),float(lo[1])],[float(hi[0]),float(lo[1])],[float(hi[0]),float(hi[1])],[float(lo[0]),float(hi[1])]]
    if len(gnss_noise)==1:cfg['gnss_simulation_noise']=gnss_noise[0]
    bridges=yaml.safe_load((directory/'bridge.yaml').read_text())
    for bridge in bridges:
        if bridge['ros_topic_name']=='/depth/front/image':
            cfg['policy_depth_frame']=sensor_topics.get(bridge['gz_topic_name']) or sensor_topics.get(bridge['gz_topic_name'].removesuffix('/depth_image'))
    cfg['sensor_frames']=sensor_frames
    cfg['lidar_frame']=lidar[0]
    lidar_joint=next(j for j in robot.findall('joint') if j.find('child').get('link')==lidar[0])
    lidar_origin=lidar_joint.find('origin')
    cfg['lidar_mount_xyz']=[float(v) for v in lidar_origin.get('xyz').split()]
    cfg['lidar_mount_rpy']=[float(v) for v in lidar_origin.get('rpy').split()]
    cfg['self_filter_radius']=float(max(np.linalg.norm(lo),np.linalg.norm(hi)))+cfg['footprint_padding']
    output.write_text(E.tostring(robot,encoding='unicode'))
    return cfg
