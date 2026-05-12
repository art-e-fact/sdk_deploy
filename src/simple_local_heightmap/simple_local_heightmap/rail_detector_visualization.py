import math

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


def build_base_markers(frame_id, stamp, front_clear_area=None):
    msg = MarkerArray()
    msg.markers.append(_delete_all_marker(frame_id, stamp))
    if front_clear_area is not None:
        msg.markers.extend(_front_clear_markers(frame_id, stamp, front_clear_area))
    return msg


def build_markers(frame_id, stamp, detection, forward_span, front_clear_area=None):
    """Build RViz markers for slice samples, rail hits, the centerline, and follow targets."""
    msg = build_base_markers(frame_id, stamp, front_clear_area=front_clear_area)

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

    if len(detection['follow_target_points']) != 0:
        msg.markers.append(
            _sphere_list_marker(
                frame_id,
                stamp,
                namespace='follow_target_candidates',
                marker_id=102,
                points=detection['follow_target_points'],
                rgb=(1.0, 0.55, 0.1),
                scale=0.08,
            )
        )

    if detection['follow_target'] is not None:
        msg.markers.append(
            _cylinder_marker(
                frame_id,
                stamp,
                namespace='follow_target',
                marker_id=103,
                x=float(detection['follow_target']['point'][0]),
                y=float(detection['follow_target']['point'][1]),
                z=0.9,
                rgb=(1.0, 0.5, 0.0),
                alpha=0.5,
                diameter=0.34,
                height=1.8,
            )
        )

    z_level = _marker_height(
        detection['midpoints'],
        detection['hits'],
        detection['follow_target_points'],
    )
    if detection['line'] is not None:
        line = detection['line']
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

    if detection['follow_target'] is None:
        text = f'{text}\nfollow_target=none'
    else:
        text = (
            f'{text}\n'
            f'follow_target={detection["follow_target"]["distance"]:.2f} m '
            f'(h={detection["follow_target"]["height"]:.2f} m)'
        )

    robot = detection['robot_xy']
    msg.markers.append(
        _text_marker(
            frame_id,
            stamp,
            namespace='summary',
            marker_id=300,
            x=float(robot[0]),
            y=float(robot[1]),
            z=z_level + 0.25,
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


def _front_clear_markers(frame_id, stamp, front_clear_area):
    origin = np.asarray(front_clear_area['origin'], dtype=np.float32)
    orientation = np.asarray(front_clear_area['orientation'], dtype=np.float32)
    offset_x = float(front_clear_area['offset_x'])
    length = float(front_clear_area['length'])
    width = float(front_clear_area['width'])

    center_local = np.array([[offset_x + 0.5 * length, 0.0, 0.0]], dtype=np.float32)
    corners_local = np.array([
        [offset_x, -0.5 * width, 0.0],
        [offset_x + length, -0.5 * width, 0.0],
        [offset_x + length, 0.5 * width, 0.0],
        [offset_x, 0.5 * width, 0.0],
        [offset_x, -0.5 * width, 0.0],
    ], dtype=np.float32)

    rotation = _quaternion_matrix(*orientation)
    world_center = (rotation @ center_local.T).T[0] + origin
    world_corners = (rotation @ corners_local.T).T + origin

    fill = Marker()
    fill.header.frame_id = frame_id
    fill.header.stamp = stamp
    fill.ns = 'front_clear_area'
    fill.id = 1
    fill.type = Marker.CUBE
    fill.action = Marker.ADD
    fill.scale.x = length
    fill.scale.y = width
    fill.scale.z = 0.02
    fill.color.r = 1.0
    fill.color.g = 0.45
    fill.color.b = 0.1
    fill.color.a = 0.18
    fill.pose.position.x = float(world_center[0])
    fill.pose.position.y = float(world_center[1])
    fill.pose.position.z = float(world_center[2]) + 0.01
    fill.pose.orientation.x = float(orientation[0])
    fill.pose.orientation.y = float(orientation[1])
    fill.pose.orientation.z = float(orientation[2])
    fill.pose.orientation.w = float(orientation[3])

    outline = Marker()
    outline.header.frame_id = frame_id
    outline.header.stamp = stamp
    outline.ns = 'front_clear_outline'
    outline.id = 2
    outline.type = Marker.LINE_STRIP
    outline.action = Marker.ADD
    outline.scale.x = 0.03
    outline.color.r = 1.0
    outline.color.g = 0.6
    outline.color.b = 0.2
    outline.color.a = 0.9
    outline.points = [_point(x, y, z + 0.03) for x, y, z in world_corners]
    return [fill, outline]


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


def _cylinder_marker(frame_id, stamp, namespace, marker_id, x, y, z, rgb, alpha, diameter, height):
    marker = Marker()
    marker.header.frame_id = frame_id
    marker.header.stamp = stamp
    marker.ns = namespace
    marker.id = marker_id
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD
    marker.scale.x = diameter
    marker.scale.y = diameter
    marker.scale.z = height
    marker.color.r, marker.color.g, marker.color.b = rgb
    marker.color.a = alpha
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
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


def _marker_height(midpoints, hits, follow_target_points):
    if len(midpoints) != 0:
        return float(np.nanmedian(midpoints[:, 2]))
    if len(hits) != 0:
        return float(np.nanmedian(hits[:, 2]))
    if len(follow_target_points) != 0:
        return float(np.nanmedian(follow_target_points[:, 2]))
    return 0.0


def _quaternion_matrix(x, y, z, w):
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return np.eye(3, dtype=np.float32)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float32)