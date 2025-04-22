// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.subsystems.DriveTrain.SwerveDrive;

public class VisionFollowAprilTag 
{
  
  SwerveDrive swerveDrive;

  public VisionFollowAprilTag(SwerveDrive swerveDrive) 
  {
    this.swerveDrive = swerveDrive;
  }

  public void followWithOffset(Vector2 offset)
  {
    double turnSpeed = 1.2;
    double driveSpeed = 2;

    double rotationVelocity = VisionDriveCalculator.rotateTowardsTarget(0) * turnSpeed;
    Vector2 moveVelocity = VisionDriveCalculator.GetVelocityToAprilTagOffset(offset).velocity.times(driveSpeed);

    SmartDashboard.putString("AT Rotation velocity", rotationVelocity + "");
    SmartDashboard.putString("AT Move velocity", moveVelocity.toString(3));

    if (moveVelocity.magnitude() < .1) { moveVelocity = Vector2.zero; }
    swerveDrive.drive(moveVelocity.y, moveVelocity.x, rotationVelocity, false);
  }
}