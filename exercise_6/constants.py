# gCX are the world coordinates for the points on the chessboards which are later used to construct the h-matrix
gCX = [[-0.025, 0.125, 1.0], [-0.025, 0.075, 1.0], [-0.025, 0.025, 1.0], [-0.075, 0.125, 1.0],
       [-0.075, 0.075, 1.0], [-0.075, 0.025, 1.0],
       [-0.125, 0.125, 1.0], [-0.125, 0.075, 1.0], [-0.125, 0.025, 1.0], [-0.175, 0.125, 1.0],
       [-0.175, 0.075, 1.0], [-0.175, 0.025, 1.0]]

maxDistToBlock = 0.7        # distance to tell goToNextBlob when to stop

grabPosition = [355.0, 495.0] # defines the position in a image 512x512 where the youBot should stay in order to grab it

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
                      170.0,      # blob 3
                      80.0,      # blob 4
                      165.0]      # blob 5

# TODO: Define the right basket paths
basketPaths =      [[(6.5, 3.8), (1.4, 5.3)],                               # basket red
                    [(1.4, 5.3)],                                           # basket red
                    [(3.6, 6.5), (-2.8, 6.5)],                              # basket blue
                    [(-4.0, 0.0)],                                          # basket blue
                    [(-3.5, -3.6), (-5.5, 6.5), (-0.5, 6.3)],               # basket red
                    [(3.6, -2.7), (3.4, -4.6), (-3.5, -4.4), (-4.0, 0.0)]]  # basket blue

# 0 - RED | 1 - BLUE
blockColor =        [0,
                     0,
                     1,
                     1,
                     0,
                     1
                     ]

# coordinate and orientation to go to the basket (x, y, ori)
basketCoordinate = [(-0.1246, 5.85, -90.0),      # RED Basket
                     (-3.9246, 1.8249, 0.0)]      # BLUE Basket

# after driving to basketCoordinate step this value to the
sidewardDistToBasket = 0.7
