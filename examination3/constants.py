# gCX are the world coordinates for the points on the chessboards which are later used to construct the h-matrix
gCX = [[-0.025, 0.125, 1.0], [-0.025, 0.075, 1.0], [-0.025, 0.025, 1.0], [-0.075, 0.125, 1.0],
       [-0.075, 0.075, 1.0], [-0.075, 0.025, 1.0],
       [-0.125, 0.125, 1.0], [-0.125, 0.075, 1.0], [-0.125, 0.025, 1.0], [-0.175, 0.125, 1.0],
       [-0.175, 0.075, 1.0], [-0.175, 0.025, 1.0]]

maxDistToBlock = 1.0        # distance to tell goToNextBlob when to stop

grabPosition = [355.0, 450.0]  # defines the position in a image 512x512 where the youBot should stay in order to grab it

explorePath = [(3.0, 1.0), (6.5, 3.0), (4.4, 5.9), (-4.0, 6.6), (-4.4, -1.0), (-3.6, -4.0), (-5.0, -4.6), (+1.5, -5.0), (3.5, -5.0), (3.5, -2.0), (-0.3, -2.0)]