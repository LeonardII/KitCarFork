from collections import namedtuple
from dataclasses import dataclass
from typing import List, Tuple

from simulation_groundtruth.msg import LabeledPolygon as LabeledPolygonMsg
from simulation_groundtruth.msg import Lane as LaneMsg
from simulation_groundtruth.msg import Line as LineMsg
from simulation_groundtruth.msg import Parking as ParkingMsg
from simulation_groundtruth.msg import Section as SectionMsg

from simulation.utils.geometry import Line, Polygon
from simulation.utils.road.road import Road
from simulation.utils.road.sections import Intersection


def _lane_msg_from_lines(left: Line, middle: Line, right: Line) -> LaneMsg:
    """Create a lane message from three lines."""
    msg = LaneMsg()
    msg.middle_line = middle.to_geometry_msgs()
    msg.left_line = left.to_geometry_msgs()
    msg.right_line = right.to_geometry_msgs()
    return msg


IntersectionTuple = namedtuple(
    "IntersectionTuple",
    ["turn", "rule", "south", "west", "east", "north"],
)
"""Container for information about an intersection."""

IntersectionTuple.turn.__doc__ = """int: Direction the road follows."""
IntersectionTuple.rule.__doc__ = """int: Priority rule."""

IntersectionTuple.south.__doc__ = """LaneMsg: Lane markings approaching the intersection."""
IntersectionTuple.west.__doc__ = (
    """LaneMsg: Lane markings on the left side when coming from south."""
)
IntersectionTuple.east.__doc__ = (
    """LaneMsg: Lane markings on the right side when coming from south."""
)
IntersectionTuple.north.__doc__ = (
    """LaneMsg: Lane markings straight ahead when coming from south."""
)


@dataclass
class Groundtruth:
    """Provide functionality to extract groundtruth information as ROS messages."""

    road: Road
    """Store the current road."""

    def get_section_msgs(self) -> List[SectionMsg]:
        """Section message for all sections."""
        messages = []
        for section in self.road.sections:
            msg = SectionMsg()
            msg.id = section.id
            msg.type = section.__class__.TYPE
            messages.append(msg)
        return messages

    def get_lane_msg(self, id: int) -> LaneMsg:
        """Lane message for requested road section.

        Args:
            id: section id
        """
        section = self.road.sections[id]
        return _lane_msg_from_lines(
            section.left_line, section.middle_line, section.right_line
        )

    def get_intersection_msg(self, id: int) -> IntersectionTuple:
        """Intersection tuple for requested road section.

        Args:
            id: section id
        """
        intersection: Intersection = self.road.sections[id]
        assert intersection.__class__.TYPE == Intersection.TYPE

        south = _lane_msg_from_lines(
            intersection.left_line_south,
            intersection.middle_line_south,
            intersection.right_line_south,
        )
        west = _lane_msg_from_lines(
            intersection.left_line_west,
            intersection.middle_line_west,
            intersection.right_line_west,
        )
        east = _lane_msg_from_lines(
            intersection.left_line_east,
            intersection.middle_line_east,
            intersection.right_line_east,
        )
        north = _lane_msg_from_lines(
            intersection.left_line_north,
            intersection.middle_line_north,
            intersection.right_line_north,
        )

        return IntersectionTuple(
            intersection.turn, intersection.rule, south, west, east, north
        )

    def get_parking_msg(self, id: int) -> Tuple[ParkingMsg, ParkingMsg]:
        """Parking message for the left and right side of the requested road section.

        Args:
            id: section id
        """

        def msg_from(lots):
            borders = [lot.border for lot in lots]
            spots = sum((lot.spots for lot in lots), [])
            msg = ParkingMsg()
            msg.borders = [LineMsg(border.to_geometry_msgs()) for border in borders]
            for spot in spots:
                spot_msg = LabeledPolygonMsg()
                spot_msg.frame = spot.frame.to_geometry_msg()
                spot_msg.type = spot.kind
                msg.spots.append(spot_msg)

            return msg

        left_msg = msg_from(self.road.sections[id].left_lots)
        right_msg = msg_from(self.road.sections[id].right_lots)

        return left_msg, right_msg

    def get_obstacle_msgs(self, id: int) -> List[LabeledPolygonMsg]:
        """Labeled polygon msg for each obstacle in the requested road section.

        Args:
            id: section id
        """
        obstacles = self.road.sections[id].obstacles
        if obstacles is None:
            return []
        msgs = []

        for obstacle in obstacles:
            assert isinstance(obstacle.frame, Polygon)
            assert isinstance(obstacle.height, float)
            assert isinstance(obstacle.id_, int)
            msg = LabeledPolygonMsg()
            msg.frame = obstacle.frame.to_geometry_msg()
            msg.height = obstacle.height
            msg.type = obstacle.id_
            msg.desc = obstacle.desc
            msgs.append(msg)

        return msgs

    def get_surface_marking_msgs(self, id: int) -> List[LabeledPolygonMsg]:
        """Labeled polygon msg for each surface marking in the requested road section.

        Args:
            id: section id
        """
        surface_markings = self.road.sections[id].surface_markings
        if surface_markings is None:
            return []
        msgs = []
        for surface_marking in surface_markings:
            msg = LabeledPolygonMsg()
            msg.frame = surface_marking.frame.to_geometry_msg()
            msg.type = surface_marking.kind[0]
            msg.desc = surface_marking.kind[1]
            msgs.append(msg)
        return msgs

    def get_traffic_sign_msgs(self, id: int) -> List[LabeledPolygonMsg]:
        """Labeled polygon msg for each traffic sign in the requested road section.

        Args:
            id: section id
        """
        signs = self.road.sections[id].traffic_signs
        if signs is None:
            return []
        msgs = []
        for sign in signs:
            msg = LabeledPolygonMsg()
            msg.frame = sign.frame.to_geometry_msg()
            msg.type = sign.kind.id_
            msg.desc = sign.kind.mesh
            msg.height = sign.kind.collision_box_size[2]
            msgs.append(msg)
        return msgs
