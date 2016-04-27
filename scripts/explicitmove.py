#/usr/bin/env python

from move_util import *

### This method will provide roomba like movement
# That means it will keep going in a straight line until it
# hits an obstacle, then adjust and move.
def roomba():
    pass #TODO

### This method provides an emotive dance to show it is stuck
# The steps of the dance are:
#   Forward and back
#   Turn 90 degrees right
#   Forward back
def stuck_dance(pub):
    forward_back(pub)
    twist_direction(pub, twist_right())
    forward_back(pub)
    twist_direction(pub, twist_left())
    forward_back(pub)
    twist_direction(pub, twist_left())
    forward_back(pub)
