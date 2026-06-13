function [wLeft, wRight] = diffDriveWheelSpeeds(v, omega, wheelRadius, wheelSeparation)
%DIFFDRIVEWHEELSPEEDS Convert body velocity to wheel angular velocities.
%   v      forward velocity (m/s)
%   omega  yaw rate (rad/s)
%   Returns left/right wheel angular velocities (rad/s) for Set Joint
%   Velocity. Callable from a MATLAB Function block. If the robot drives
%   backward in Gazebo, negate both outputs (wheel joint axis sign).

wLeft  = (v - omega * wheelSeparation / 2) / wheelRadius;
wRight = (v + omega * wheelSeparation / 2) / wheelRadius;
end
