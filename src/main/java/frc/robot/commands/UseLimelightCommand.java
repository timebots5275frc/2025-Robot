// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Vision.Vision;

public class UseLimelightCommand extends InstantCommand {
  boolean use;

  public UseLimelightCommand(boolean useLimelight) {
    this.use = useLimelight;
  }

  @Override
  public void initialize() {
    Vision.Instance.setUsingLimelight(use);
  }
}