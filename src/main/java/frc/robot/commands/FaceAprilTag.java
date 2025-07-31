// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Vision.VisionDriveCalculator;

public class FaceAprilTag extends Command 
{
  SwerveDrive swerveDrive;
  Vision vision;

  boolean facingTarget = false;

  private static double turnSpeed = 3;

  public FaceAprilTag(SwerveDrive swerveDrive, Vision vision) 
  {
    this.swerveDrive = swerveDrive;
    this.vision = vision;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize()
  {
    facingTarget = false;
    Vision.Instance.setUsingLimelight(true);
  }

  @Override
  public void execute()
  {
    double tx = vision.HorizontalOffsetFromAprilTag();
    double offset = VisionDriveCalculator.getAngleOffsetForVision();
    double rotationDirection = VisionDriveCalculator.rotateTowardsTarget(offset);

    if (Math.abs(tx + offset) <= .8 && vision.hasValidData()) { facingTarget = true; }
    else { swerveDrive.drive(0, 0, rotationDirection * turnSpeed, false); }
  }

  @Override
  public void end(boolean a)
  {
    Vision.Instance.setUsingLimelight(false);
    swerveDrive.drive(00, 00000, 0.0, false);
  }

  @Override
  public boolean isFinished() 
  {
    return facingTarget;
  }
}