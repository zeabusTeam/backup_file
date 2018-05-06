#!/usr/bin/python2

############################################################
# Fix xy mission <<MUST FIX THESE CONSTANTS BEFORE LAUNCH>>
############################################################

######################################################
# LEFT+ RIGHT- UP+ DOWN- COUNTER-CLOCKWISE+ CLOCKWISE-
######################################################

GATE_X = 4 #X
GATE_Y = 0 #Y

FLARE_DEGREE = -90 #DEGREE
FLARE_X = 15 #X
FLARE_Y = 6 #Y

DRUM_DEGREE = 0 #DEGREE
DRUM_X = 22.25 #X
DRUM_Y = 0 #Y

##################
# Missions
##################

GATE_DEPTH = -1.2
GATE_FRONT = 0

#GATE_POS = 'left'
GATE_POS = 'right'

FLARE_DEPTH = -1

#FLARE_POS = 'left'
FLARE_POS = 'right'

DRUM_DEPTH = -0.6
DRUM_DROP_BALL = 0.82

DRUM_POS = 'left'
#DRUM_POS = 'right'

############################################################
# RE-CHECK EVERYTIME BEFORE RUNNING MISSION
############################################################


############################################################
# Fixed constants <<DONT TOUCH THESE CONSTANTS>>
############################################################

################################
# AUV related
################################

##################
# move in x axis
##################

AUV_SPEED = 0.6
AUV_FULL_SPEED = 1

##################
# move in y axis
##################

AUV_SPEED_FIND_GATE = 0.5
AUV_SPEED_FIND_FLARE = 0.3
AUV_SPEED_FIND_DRUM = 0.5
AUV_SPEED_ADJUST = 0.4

##################
# Vision
##################

VISION_ERROR = 0.05
VISION_FRONT_FIX = 0.05
VISION_ERROR_FLARE = 0.10 ## 0.02
VISION_DRUM = 0.2
