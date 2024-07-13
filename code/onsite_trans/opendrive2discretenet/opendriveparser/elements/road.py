# -*- coding: utf-8 -*-

from onsite_trans.opendrive2discretenet.opendriveparser.elements.roadPlanView import PlanView
from onsite_trans.opendrive2discretenet.opendriveparser.elements.roadLink import Link
from onsite_trans.opendrive2discretenet.opendriveparser.elements.roadLanes import Lanes
from onsite_trans.opendrive2discretenet.opendriveparser.elements.roadElevationProfile import (
    ElevationProfile,
)
from onsite_trans.opendrive2discretenet.opendriveparser.elements.roadLateralProfile import LateralProfile
from onsite_trans.opendrive2discretenet.opendriveparser.elements.junction import Junction


class Road:
    """ """

    def __init__(self):
        self._id = None
        self._name = None
        self._junction = None  # The ID of the intersection. This road belongs to the intersection when it is used as a link road, otherwise it is -1
        self._length = None

        self._header = None  # TODO
        self._link = Link()
        self._types = []
        self._planView = PlanView()  #  Reference Line
        self._elevationProfile = ElevationProfile()  # road height(Section 8.4.1 in OpenD 1.6)
        self._lateralProfile = LateralProfile()  # Section 8.4.2 in OpenD 1.6
        self._lanes = Lanes()

    def __eq__(self, other):
        return self.__dict__ == other.__dict__

    @property
    def id(self):
        """ """
        return self._id

    @id.setter
    def id(self, value):
        self._id = int(value)

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, value):
        self._name = str(value)

    @property
    def junction(self):
        return self._junction

    @junction.setter
    def junction(self, value):

        if not isinstance(value, Junction) and value is not None:
            raise TypeError("Property must be a Junction or NoneType")

        if value == -1:
            value = None

        self._junction = value

    @property
    def link(self):
        return self._link

    @property
    def types(self):
        return self._types

    @property
    def planView(self):
        return self._planView

    @property
    def elevationProfile(self):
        return self._elevationProfile

    @property
    def lateralProfile(self):
        return self._lateralProfile

    @property
    def lanes(self):
        return self._lanes
