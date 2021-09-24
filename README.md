# MYO-Controller
Left hand: Use KNN to recognize four gestures of hand, combined with RPY to control a ground vehicle. Control message is published on /cmd_vel.
Right hand:Use the quaternion of armband to control a mechanical arm, position is calculated out by a sphere constraint with quaternion. world frame can be self-defined to adapt to various real world. Control message is published on /pose.

'''rosrun myo pymyolinux_example.py'''
