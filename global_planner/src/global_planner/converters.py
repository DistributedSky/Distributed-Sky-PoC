from typing import Tuple

from shapely.geometry import LineString, Point
from geographic_msgs.msg import GeoPath, GeoPoint, GeoPoseStamped


class GeoConverter:

    @classmethod
    def geo_path_to_line_string(cls, geo_path: GeoPath) -> LineString:
        points = []
        for pose_wrapper in geo_path.poses:
            pose = pose_wrapper.pose
            position = pose.position
            point = Point(position.longitude, position.latitude, position.altitude)
            points.append(point)
        return LineString(points)

    @classmethod
    def point_to_geo_point(cls, point: Point) -> GeoPoint:
        geo_point = GeoPoint()
        geo_point.longitude = point.x
        geo_point.latitude = point.y
        geo_point.altitude = point.z
        return geo_point

    @classmethod
    def tuple_to_geo_point(cls, point_tuple: Tuple) -> GeoPoint:
        geo_point = GeoPoint()
        geo_point.longitude = point_tuple[0]
        geo_point.latitude = point_tuple[1]
        geo_point.altitude = point_tuple[2]
        return geo_point

    @classmethod
    def line_string_to_geo_path(cls, line_string: LineString) -> GeoPath:
        geo_path = GeoPath()

        for point in line_string.coords:
            geo_pose_stamped = GeoPoseStamped()
            geo_pose_stamped.pose.position = cls.tuple_to_geo_point(point)
            geo_path.poses.append(geo_pose_stamped)

        return geo_path
