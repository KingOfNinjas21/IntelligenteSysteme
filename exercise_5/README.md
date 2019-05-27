Sebastian Horngacher (11722745), Martin Schnöller (11722749), Patrick Hofmann (11725141)

Our solution is constructed of two parts. The image recognition and the path finding algorithm.
The image recognition is stored in the colorDetection.py file and the path finding algorithm in the aStar.py file.
The entry point of our program is the main.py file.
At first we take an image of the chessboard in the scene. Then we calculate the corners of the chessboard in the picture and together with the coordinates in the real environment we calculate a matrix called H matrix (main.getH()).
With this H matrix we map points of an image (taken by the robot camera) to the real environment.
We are able to detect the boundary of an colored block. So we take a photo with the robot camera and detect all blocks in the picture (colorDetection.getContours()).
Then we calculate the distance from the robot to the block and with this distance the coordinates of the block in the environment and save the position.
To get all blocks in the scene we start to rotate the robot a bit and take another picture. But we add only new blocks to the list.
So after some rotations and pictures we get a map of all blocks in the scene colorDetection.findAllBlobs().

To find a path to the goal we put this list of obstacles together with the robot position and the goal position into an A* algorithm.
The output is a set of coordinates leading to the goal.