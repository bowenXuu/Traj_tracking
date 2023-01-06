# Optitrack use related:
1. Ensure that the motion capture coordinate system and Rviz in the coordinate system to meet the right-hand system, if not meet the need to adjust the RigidBody-transform option in Optitrack Y Axis to 180

# The use of remote control related: 
1. In case of emergency, first try the 6-cut, which will command the plane to land in place.
  If the 6 cut does not work, you can try 5 cut low manual control if you have hand flying experience (be careful to adjust the throttle to the right position before cutting manual to prevent the plane from falling or hitting the ceiling)
  If 5 cut low is still ineffective, or the aircraft threatens the personal safety of others, direct 7 cut high emergency stop
2. After takeoff, the 5 channel should always maintain the cut status, otherwise if the fixed point to stop sending, the aircraft will be forced to exit offboard mode

# Airframe configuration related:
1. To ensure the balance of the center of gravity of the frame, each time before takeoff should make the robot arm in a fixed preset position, and the battery placement position also needs to be fixed
2. Check and adjust the landing gear before each takeoff, so that its support direction is perpendicular to the ground, to ensure that it looks vertical, no need to be strictly vertical
3. Pay attention to the landing must let the robot arm aligned with the pit, to prevent the robot arm broken by mistake when landing

# Flight control related:
1. poor contact with the microusb connection port of the flight control, if the start phenomenon and "the light becomes blue flashing state" description does not match, re-plug can
2. ESC long time on the power will not turn the alarm, at this time there are two strategies
  If the aircraft in the field, you can remote control double rocker inside eight to arm the aircraft (note that the arm is not take off, must not give throttle)
  If the aircraft in the debugging, re-plug the new flight control power restart flight control can
  Implementation of any one of the above two, you can make the ESC to stop the alarm

