"""Một nguồn đăng ký độc lập, có provenance; không mặc định map == world."""
import hashlib
import json
import math
from pathlib import Path

import yaml

ROOT=Path(__file__).resolve().parents[1]
WORLD=ROOT/'src/nino_description/worlds/long_hall.sdf'
ROBOT=ROOT/'src/nino_description/urdf/nino.urdf.xacro'
CONVENTION='p_world = R(theta) * p_map + translation'


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def document_digest(document):
    return hashlib.sha256(json.dumps(document,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def transform(value):
    if not isinstance(value,(list,tuple)) or len(value)!=3 or any(
            isinstance(v,bool) or not isinstance(v,(float,int)) or not math.isfinite(v) for v in value):
        raise ValueError('map_to_world phải là [tx,ty,theta] hữu hạn')
    return [float(v) for v in value]


def asset_hashes(map_path,world_path=WORLD,robot_path=ROBOT):
    if map_path is None:raise ValueError('Thiếu map path để xác minh registration')
    map_path=Path(map_path).expanduser().resolve()
    data=yaml.safe_load(map_path.read_text())
    image=map_path.parent/Path(data['image']).expanduser()
    return dict(map_yaml_sha256=digest(map_path),map_image_sha256=digest(image),
                world_sdf_sha256=digest(world_path),robot_xacro_sha256=digest(robot_path))


def validate_document(doc,map_path,world_path=WORLD):
    if doc.get('version')!=1 or doc.get('convention')!=CONVENTION:
        raise ValueError('Thiếu version/convention đăng ký map→world')
    value=transform(doc.get('map_to_world'))
    if doc.get('frames')!=dict(source='map',target='world',robot_point='base_footprint'):
        raise ValueError('Đăng ký phải map→world tại cùng điểm base_footprint')
    source=doc.get('source',{})
    if source.get('independent_of_amcl') is not True or not source.get('method') or not source.get('files'):
        raise ValueError('Thiếu nguồn đăng ký độc lập AMCL')
    verification=doc.get('verification',{})
    if verification.get('status')!='verified_independent_estimate' or not verification.get('checks'):
        raise ValueError('Đăng ký chưa được xác minh độc lập')
    if doc.get('assets')!=asset_hashes(map_path,world_path):
        raise ValueError('Đăng ký không tương thích hash map/image/world/robot hiện tại')
    for evidence in source['files']:
        path=Path(evidence['path'])
        if not path.is_file() or digest(path)!=evidence.get('sha256'):
            raise ValueError('Nguồn đăng ký thiếu/đã đổi: '+str(path))
    return value


def load_registration(path,map_path,world_path=WORLD,required=False):
    if not path:
        if required:raise ValueError('Thiếu --registration độc lập đã xác minh; dừng luồng phụ thuộc GT')
        return None
    path=Path(path).expanduser().resolve()
    doc=yaml.safe_load(path.read_text())
    if not isinstance(doc,dict):raise ValueError('File đăng ký phải là object')
    value=validate_document(doc,map_path,world_path)
    return dict(path=str(path),sha256=digest(path),document_sha256=document_digest(doc),
                document=doc,map_to_world=value,convention=CONVENTION)


def registration_from_metadata(metadata,map_path,world_path=WORLD,required=False):
    evidence=metadata.get('registration')
    if not evidence:
        if required:raise ValueError('Metadata thiếu đăng ký đã xác minh; legacy map_to_world không đủ provenance')
        return None
    doc=evidence.get('document',{})
    if evidence.get('document_sha256')!=document_digest(doc):raise ValueError('Registration metadata digest mismatch')
    value=validate_document(doc,map_path,world_path)
    if evidence.get('map_to_world')!=value:raise ValueError('Registration transform khác document')
    path=Path(evidence.get('path',''))
    if path.is_file() and digest(path)!=evidence.get('sha256'):raise ValueError('File đăng ký đã đổi sau khi ghi metadata')
    return evidence
