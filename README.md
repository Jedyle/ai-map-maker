# ai-map-maker

Robot software capable of navigating into a MRDS environment and constructing a map of it.

To run this program, you need MRDS installed and a map loaded.

Then just run ./mapper url x1 y1 x2 y2, where :
 - url is the adress and port to the machine running MRDS (ex : http://localhost:50000)
 - x1, y1, x2, y2 give the coordinates of the lower left and upper right corners respectively of the area the robot should explore and map, in meters.