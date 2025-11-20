// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem.CoralIntakeState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CSC extends Command {
  CoralIntakeSubsystem ci_sub;
  CoralIntakeState cis;
  /** Creates a new CSC. */
  public CSC(CoralIntakeSubsystem ci_sub, CoralIntakeState cis) {
    this.ci_sub = ci_sub;
    this.cis = cis;
    
    addRequirements(ci_sub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ci_sub.SetCoralIntakeState(cis);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {ci_sub.SetCoralIntakeState(CoralIntakeState.NONE);}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ci_sub.CoralOutOfWay();
  }
}
