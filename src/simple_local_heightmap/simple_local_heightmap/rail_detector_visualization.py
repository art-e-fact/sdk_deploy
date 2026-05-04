import math

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def build_markers(frame_id, stamp, detection, forward_span):
    """Build RViz markers for slice samples, rail hits, and the fitted centerline."""
    msg = MarkerArray()
    msg.markers.append(_delete_all_marker(frame_id, stamp))

    for index, slice_result in enumerate(detection['slices']):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'slice_profiles'
        marker.id = index
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.015
        marker.color.r = 0.2
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.75
        marker.points = [
            _point(x, y, z + 0.01)
            for (x, y), z in zip(slice_result['xy'], slice_result['z'])
            if math.isfinite(float(z))
        ]
        if marker.points:
            msg.markers.append(marker)

    if len(detection['hits']) != 0:
        msg.markers.append(
            _sphere_list_marker(
                frame_id,
                stamp,
                namespace='rail_hits',
                marker_id=100,
                points=detection['hits'],
                rgb=(0.5, 0.2, 0.95),
                scale=0.08,
            )
        )

    if len(detection['midpoints']) != 0:
        msg.markers.append(
            _sphere_list_marker(
                frame_id,
                stamp,
                namespace='center_samples',
                marker_id=101,
                points=detection['midpoints'],
                rgb=(1.0, 0.9, 0.1),
                scale=0.10,
            )
        )

    if detection['line'] is not None:
        line = detection['line']
        z_level = _marker_height(detection['midpoints'], detection['hits'])
        start = line['center'] - 0.8 * forward_span * line['tangent']
        end = line['center'] + 0.8 * forward_span * line['tangent']
        msg.markers.append(
            _line_marker(
                frame_id,
                stamp,
                namespace='centerline',
                marker_id=200,
                start=np.array([start[0], start[1], z_level + 0.04], dtype=np.float32),
                end=np.array([end[0], end[1], z_level + 0.04], dtype=np.float32),
                rgb=(0.1, 1.0, 0.2),
                width=0.05,
            )
        )
        text = (
            f'offset={line["signed_offset"]:+.2f} m\n'
            f'heading={math.degrees(line["yaw"]):.1f} deg'
        )
    else:
        text = 'rail parse incomplete'

    robot = detection['robot_xy']
    msg.markers.append(
        _text_marker(
            frame_id,
            stamp,
            namespace='summary',
            marker_id=300,
            x=float(robot[0]),
            y=float(robot[1]),
            z=_marker_height(detection['midpoints'], detection['hits']) + 0.25,
            text=text,
        )
    )
    return msg


def _delete_all_marker(frame_id, stamp):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.action = Marker.DELETEALL
    return marker


def _point(x, y, z):
    point = Point()
    point.x = float(x)
    point.y = float(y)
    point.z = float(z)
    return point


def _sphere_list_marker(frame_id, stamp, namespace, marker_id, points, rgb, scale):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.SPHERE_LIST
    marker.action = Marker.ADD
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = 0.95
    marker.points = [_point(x, y, z + 0.02) for x, y, z in np.asarray(points)]
    return marker


def _line_marker(frame_id, stamp, namespace, marker_id, start, end, rgb, width):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.scale.x = width
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = 0.95
    marker.points = [_point(*start), _point(*end)]
    return marker


def _text_marker(frame_id, stamp, namespace, marker_id, x, y, z, text):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.TEXT_VIEW_FACING
    marker.action = Marker.ADD
    marker.scale.z = 0.18
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 0.95
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.text = text
    return marker


def _marker_height(midpoints, hits):
    if len(midpoints) != 0:
        return float(np.nanmedian(midpoints[:, 2]))
    if len(hits) != 0:
        return float(np.nanmedian(hits[:, 2]))
    return 0.0