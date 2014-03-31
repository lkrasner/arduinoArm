arduinoArm
==========

An arduino powered robot arm controlled with a PS3 controller, over usb or bluetooth.  USB communication is recommended if wires aren't an issue, as it tends to connect better and is generally easier.  please use the included modified library, which adds double precision servo positions.  If you are using the ADK board instead of a USB host shield, you may also use the included usb host library modified to work with it.

The RobotDistance.ino sketch is designed to allow coordinated horizontal and vertical movement, however, due to the poor processing power of the Arduino, it does not work very well. 
