"""Definition of the EventSpeaker class."""
import functools
from typing import Any, Callable, List

from simulation_evaluation.msg import Speaker as SpeakerMsg
from simulation_groundtruth.msg import LabeledPolygon as LabeledPolygonMsg
from simulation_groundtruth.msg import Lane as LaneMsg
from simulation_groundtruth.msg import Parking as ParkingMsg
from simulation_groundtruth.msg import Section as SectionMsg

import simulation.utils.road.sections.type as road_section_type
from simulation.utils.geometry import Polygon
from simulation.utils.road.sections import SurfaceMarking

from .speaker import Speaker

# Used to simplify the polygons
BUFFER = 0.0001


class EventSpeaker(Speaker):
    """Find events that happen during a drive e.g collision, parked in spot."""

    def __init__(
        self,
        *,
        section_proxy: Callable[[], List[SectionMsg]],
        lane_proxy: Callable[[int], LaneMsg],
        obstacle_proxy: Callable[[int], List[LabeledPolygonMsg]],
        surface_marking_proxy: Callable[[int], List[LabeledPolygonMsg]],
        parking_proxy: Callable[[int], Any],
        parking_spot_buffer: float,
        min_parking_wheels: int
    ):
        """Event speaker with funtions that can be queried for groundtruth information and
        parameters.

        Args:
            section_proxy: Returns all sections when called.
            lane_proxy: Returns a LaneMsg for each section.
            obstacle_proxy: function which returns obstacles in a section.
            parking_proxy: function which returns parking msg in a section.
            parking_spot_buffer: buffer making accepted parking spots larger
            min_parking_wheels: Minimum amount of wheels which must be inside
                a parking spot when parking.
        """
        super().__init__(
            section_proxy=section_proxy,
            lane_proxy=lane_proxy,
            obstacle_proxy=obstacle_proxy,
            surface_marking_proxy=surface_marking_proxy,
        )

        self.get_parking_msgs = parking_proxy

        # Buffer is created around a parking spot,
        # being on the line qualifies as being inside!
        self.parking_spot_buffer = parking_spot_buffer
        self.min_parking_wheels = min_parking_wheels

    @functools.cached_property
    def parking_spots(self) -> List[Polygon]:
        """Return all parking spots as a list of polygons."""
        parking_spots = []

        def extend_spots(msg: ParkingMsg):
            """Extend the parking spots by the spots inside msg."""
            parking_spots.extend(
                [
                    Polygon(
                        Polygon(s.frame).buffer(self.parking_spot_buffer).exterior.coords
                    )
                    for s in msg.spots
                    if s.type == ParkingMsg.SPOT_FREE
                ]
            )

        for sec in self.sections:
            if not sec.type == road_section_type.PARKING_AREA:
                continue
            srv = self.get_parking_msgs(id=sec.id)

            if srv.right_msg:
                extend_spots(srv.right_msg)

            if srv.left_msg:
                extend_spots(srv.left_msg)

        return parking_spots

    @property
    def obstacles(self) -> List[Polygon]:
        """All obstacles as a list of polygons."""
        return [
            # Buffer the obstacle that is received from proxy because of some issues
            # with shapely thinking that the points are not a valid polygon!
            # And it further simplifies the polygons
            Polygon(lp.frame.buffer(BUFFER).exterior.coords)
            for sec in self.sections
            for lp in self.get_obstacles_in_section(sec.id)
        ]

    @functools.cached_property
    def blocked_areas(self) -> List[Polygon]:
        """All blocked_areas as a list of polygons."""
        return [
            # Buffer the blocked_area that is received from proxy because of some issues
            # with shapely thinking that the points are not a valid polygon!
            # And it further simplifies the polygons
            Polygon(sm.frame.buffer(BUFFER).exterior.coords)
            for sec in self.sections
            for sm in self.get_surface_markings_in_section(sec.id)
            if sm.id_ == SurfaceMarking.BLOCKED_AREA[0]
        ]

    def speak(self) -> List[SpeakerMsg]:
        """Return a list of SpeakerMsgs.

        Contents:
            * Collision with an obstacle -> :ref:`Speaker <speaker_msg>`.COLLISION,
            * inside a parking spot -> :ref:`Speaker <speaker_msg>`.PARKING_SPOT
        """
        msgs = super().speak()

        def append_with_type(t):
            """Append speaker msgs with a new msg of type t."""
            msg = SpeakerMsg()
            msg.type = t
            msgs.append(msg)

        if self.car_overlaps_with(*self.obstacles):
            append_with_type(SpeakerMsg.COLLISION)

        if self.car_overlaps_with(*self.blocked_areas):
            append_with_type(SpeakerMsg.BLOCKED_AREA)

        if self.wheel_count_inside(*self.parking_spots) >= self.min_parking_wheels:
            append_with_type(SpeakerMsg.PARKING_SPOT)

        return msgs
