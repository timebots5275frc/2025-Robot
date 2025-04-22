// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.Vision;

public class VisionOdometryCalculator extends SubsystemBase {
  Vision vision;
  SwerveDrive swerveDrive;

  Vector3 workingFieldSpacePosition;
  Vector3 workingFieldSpaceRotation;

  Vector3 lastValidLimelightPos;
  Vector3 lastValidLimelightRot;

  Vector2 odometryPosLastPeriodic = Vector2.zero;
  double odometryHeadingLastPeriodic = 0;

  public VisionOdometryCalculator(Vision vision, SwerveDrive swerveDrive) {
    this.vision = vision;
    this.swerveDrive = swerveDrive;
  }

  @Override
  public void periodic() {
    if (vision.hasValidData()) {
      workingFieldSpacePosition = vision.RobotPosInFieldSpace();
      workingFieldSpaceRotation = vision.RobotRotInFieldSpace();

      lastValidLimelightPos = workingFieldSpacePosition;
      lastValidLimelightRot = workingFieldSpaceRotation;
    }
    else {
      Vector2 deltaOdometryPos = odometryPosLastPeriodic.substract(odometryPosLastPeriodic);
    }

    odometryPosLastPeriodic = swerveDrive.getOdometryPosition();
    odometryHeadingLastPeriodic = swerveDrive.getHeading().getDegrees();
  }
}