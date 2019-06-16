# gCX are the world coordinates for the points on the chessboards which are later used to construct the h-matrix
gCX = [[-0.025, 0.125, 1.0], [-0.025, 0.075, 1.0], [-0.025, 0.025, 1.0], [-0.075, 0.125, 1.0],
       [-0.075, 0.075, 1.0], [-0.075, 0.025, 1.0],
       [-0.125, 0.125, 1.0], [-0.125, 0.075, 1.0], [-0.125, 0.025, 1.0], [-0.175, 0.125, 1.0],
       [-0.175, 0.075, 1.0], [-0.175, 0.025, 1.0]]

maxDistToBlock = 0.5        # distance to tell goToNextBlob when to stop

grabPosition = [355.0, 450.0] # defines the position in a image 512x512 where the youBot should stay in order to grab it

# TODO: Define the right explore paths
explorePaths =      [[(5.1, 1.6)],  # blob 0 r
                     [(2.2, 4.9)],  # blob 1 r
                     [(4.7, 4.8)],  # blob 2 b
                     [(-4.2, 2.8), (-3.9, -1.2)],   # blob 3 b
                     [(-3.5, -3.6), (-4.1, -4.8)],  # blob 4 r
                     [(6.2, 6.2), (6.5, 1.4), (-0.5, 0.8), (-0.4, -1.4)]]   # blob 5 b

exploreOrientation = [145.0,    # blob 0
                      -110.0,     # blob 1
                      -150.0,     # blob 2
                      170,      # blob 3
                      100,      # blob 4
                      165]      # blob 5

# TODO: Define the right basket paths
basketPaths =      [[(6.5, 3.8), (0.6, 4.6)],                               # basket red
                    [(0.6, 4.6)],                                           # basket red
                    [(3.6, 6.5), (-2.8, 6.5), (-3.0, 2.8)],                 # basket blue
                    [(-3.2, 1.3)],                                          # basket blue
                    [(-3.5, -3.6), (-5.5, 6.5), (-0.5, 6.3), (0.0, 5.8)],   # basket red
                    [(3.6, -2.7), (3.4, -4.6), (-3.5, -4.4), (-3.1, 1.2)]]  # basket blue
