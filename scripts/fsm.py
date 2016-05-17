#!/usr/bin/env python

import rospy
import time

IDLE_STATE = 0
WAIT_PACKAGE_STATE = 1
TO_ROOM = 2
AT_ROOM = 3
TO_FRONT_DOOR = 4
TO_FRONT_DESK = 5

WAIT_PACKAGE_POSE = "wait_package_pose"

class RobotFSM:
    # Passed as listener to topic qrcode
    # Gets a Order object
    def validateQRCode(self, qrcode):
        # see if location is one we know about
        if self._state == IDLE_STATE and self._map.contains(qrcode.location):
            # Turn around
            self._state = WAIT_PACKAGE_STATE
            self._move_util.forwardThenTurn(self, WAIT_PACKAGE_POSE)
            # Wait for package (in future this will involve scale readings
            self._order = qrcode
            time.sleep(20)
            self.packageReceived()
        else:
            # TODO: Tell delivery person QRCode is invalid
            pass

    def packageReceived(self):
        if self._state == WAIT_PACKAGE_STATE:
            self._state = TO_ROOM
            # Note, this will hang until we reach the location or give up
            # TODO: Handle failure
            if self._move_util.goToPose(self._order.location, 300):
                self.reachedDoor()

    def reachedDoor(self):
        if self._state == TO_ROOM:
            self._state = AT_ROOM
            # TODO: Send message

    def __init__(self, myMap):
        self._state = IDLE_STATE
        self._map = myMap
        self._move_util = MoveUtil(myMap)
        self._order = None
