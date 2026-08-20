"""Grasshopper UDP pose publisher for T_base_flange.

GhPython inputs:
    T        Rhino.Geometry.Plane or Rhino.Geometry.Transform
    enabled  bool; publish once per Grasshopper solution when true
    host     UDP host, default 127.0.0.1
    port     UDP port, default 49152
    session  optional session id
    state    optional robot state string, default READY

GhPython outputs:
    ok, error, seq, packet
"""

from __future__ import annotations

import json
import socket
import time

try:
    import scriptcontext as sc
except Exception:  # outside Grasshopper tests
    sc = None


def _coord(value, name):
    if hasattr(value, name):
        return float(getattr(value, name))
    lower = name.lower()
    if hasattr(value, lower):
        return float(getattr(value, lower))
    raise AttributeError('Vector/point object is missing coordinate ' + name)


def _transform_to_matrix(transform):
    names = ['M00', 'M01', 'M02', 'M03', 'M10', 'M11', 'M12', 'M13', 'M20', 'M21', 'M22', 'M23', 'M30', 'M31', 'M32', 'M33']
    if not all(hasattr(transform, name) for name in names):
        return None
    return [
        [float(transform.M00), float(transform.M01), float(transform.M02), float(transform.M03)],
        [float(transform.M10), float(transform.M11), float(transform.M12), float(transform.M13)],
        [float(transform.M20), float(transform.M21), float(transform.M22), float(transform.M23)],
        [float(transform.M30), float(transform.M31), float(transform.M32), float(transform.M33)],
    ]


def _plane_to_matrix(plane):
    if not all(hasattr(plane, name) for name in ['Origin', 'XAxis', 'YAxis', 'ZAxis']):
        return None
    origin = plane.Origin
    x_axis = plane.XAxis
    y_axis = plane.YAxis
    z_axis = plane.ZAxis
    return [
        [_coord(x_axis, 'X'), _coord(y_axis, 'X'), _coord(z_axis, 'X'), _coord(origin, 'X')],
        [_coord(x_axis, 'Y'), _coord(y_axis, 'Y'), _coord(z_axis, 'Y'), _coord(origin, 'Y')],
        [_coord(x_axis, 'Z'), _coord(y_axis, 'Z'), _coord(z_axis, 'Z'), _coord(origin, 'Z')],
        [0.0, 0.0, 0.0, 1.0],
    ]


def plane_or_transform_to_matrix(value):
    matrix = _transform_to_matrix(value)
    if matrix is not None:
        return matrix
    matrix = _plane_to_matrix(value)
    if matrix is not None:
        return matrix
    raise TypeError('T must be a Rhino.Geometry.Plane or Rhino.Geometry.Transform')


def build_pose_packet(T, seq, session_id=None, state='READY'):
    return {
        'type': 'POSE',
        'session_id': session_id,
        'seq': int(seq),
        'timestamp': time.time(),
        'T_base_flange': plane_or_transform_to_matrix(T),
        'matrix_name': 'T_base_flange',
        'source_frame': 'flange',
        'target_frame': 'base',
        'units': 'mm',
        'state': state or 'READY',
    }


def _sticky_key(suffix):
    if 'ghenv' in globals():
        return 'kuka_zivid_udp_' + str(ghenv.Component.InstanceGuid) + '_' + suffix
    return 'kuka_zivid_udp_standalone_' + suffix


ok = False
error = None
seq = None
packet = None

if 'T' in globals() and bool(globals().get('enabled', True)):
    try:
        sticky = sc.sticky if sc is not None else {}
        seq_key = _sticky_key('seq')
        seq = int(sticky.get(seq_key, globals().get('seq_start', 1)))
        packet = build_pose_packet(T, seq, globals().get('session', None), globals().get('state', 'READY'))
        payload = json.dumps(packet, separators=(',', ':')).encode('utf-8')
        udp_host = globals().get('host', None) or '127.0.0.1'
        udp_port = int(globals().get('port', None) or 49152)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            sock.sendto(payload, (udp_host, udp_port))
        finally:
            sock.close()
        sticky[seq_key] = seq + 1
        ok = True
    except Exception as exc:
        error = 'UDP PUBLISH ERROR: ' + str(exc)