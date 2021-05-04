define kinect ranger
(
  sensor
  (
    range_max 10.0 # LJM - 6.5
    fov 360 # LJM - 58.0
    samples 1800 # LJM - 640
    size [ 0.10 0.15 0.05 ]
    pose [0 0 0.1 0]
  )
  # generic model properties
  color_rgba [1.0 1.0 0.0 0.0]
  watts 2.0
)

define webcam camera
(
  # laser properties
  resolution [ 320 240 ]
  range [ 0.2 8.0 ]
  fov [ 70.0 40.0 ]
  pantilt [ 0.0 0.0 ]

  # model properties
  size [ 0.1 0.05 0.05 ]
  color "purple"
  watts 100.0 # TODO find watts for sony pan-tilt camera
)

define turtlebot position
(
  pose [ 0.0 0.0 0.0 -1.57 ]
  
  localization "odom"  # LJM - added

  odom_error [0.0 0.0 0.0 0.0]   # [x y z theta]

  size [ 0.2552 0.2552 0.40 ]
  origin [ 0.0 0.0 0.0 0.0 ]
  gui_nose 1
  drive "diff"
  color "gray"
  
  kinect(pose [ 0.0 0.0 0.05 0.0 ]) # LJM - kinect(pose [ -0.1 0.0 -0.11 0.0 ])
  webcam(pose [ 0.0 0.0 0.10 0.0 ])
)
