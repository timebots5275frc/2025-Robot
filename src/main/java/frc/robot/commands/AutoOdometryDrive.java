// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.CustomTypes.Math.SillyMath;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.subsystems.DriveTrain.SwerveDrive;

public class AutoOdometryDrive extends Command {
  SwerveDrive swerveDrive;
  Vector2 relativeTargetPosition;
  double speed;

  Vector2 fieldSpaceStartingPos;
  Vector2 fieldSpaceTargetPos;
  boolean madeItToTarget = false;

  public AutoOdometryDrive(SwerveDrive swerveDrive, Vector2 relativeTargetPosition, double speed) {
    this.swerveDrive = swerveDrive;
    this.relativeTargetPosition = relativeTargetPosition;
    this.speed = SillyMath.clamp(speed, DriveConstants.AUTO_ODOMETRY_DRIVE_MIN_SPEED, DriveConstants.AUTO_ODOMETRY_DRIVE_MAX_SPEED);

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() 
  {
    fieldSpaceStartingPos = swerveDrive.getOdometryPosition();
    fieldSpaceTargetPos = fieldSpaceStartingPos.add(Vector2.rotate(relativeTargetPosition, Math.toRadians(swerveDrive.getGyroYawInDegrees())));
  }

  @Override
  public void execute() 
  {
    Vector2 currentFieldSpaceOdometryPos = swerveDrive.getOdometryPosition();
    double distanceToFieldSpaceTargetPos = Vector2.distance(currentFieldSpaceOdometryPos, fieldSpaceTargetPos);

    if (distanceToFieldSpaceTargetPos <= DriveConstants.AUTO_ODOMETRY_DRIVE_TARGET_ALLOWED_ERROR)
    {
      madeItToTarget = true;
      swerveDrive.drive(0, 0, 0, false);
    }
    else
    {
      double adjustedSpeed = speed;

      if (distanceToFieldSpaceTargetPos < DriveConstants.AUTO_ODOMETRY_DRIVE_SLOWDOWN_DISTANCE)
      {
        adjustedSpeed = speed * (distanceToFieldSpaceTargetPos / DriveConstants.AUTO_ODOMETRY_DRIVE_SLOWDOWN_DISTANCE);
        if (adjustedSpeed < DriveConstants.AUTO_ODOMETRY_DRIVE_MIN_SPEED) { adjustedSpeed = DriveConstants.AUTO_ODOMETRY_DRIVE_MIN_SPEED; }
      }

      Vector2 driveVelocity = fieldSpaceTargetPos.substract(currentFieldSpaceOdometryPos).normalized().times(adjustedSpeed);
      swerveDrive.drive(driveVelocity.x, driveVelocity.y, 0, true);
    }
  }

  @Override
  public void end(boolean interrupted)
  {
    swerveDrive.drive(0, 0, 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return madeItToTarget;
  }
}