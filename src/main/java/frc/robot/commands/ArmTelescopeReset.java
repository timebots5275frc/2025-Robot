// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.armTelescopeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmTelescopeReset extends Command {
  private ElevatorSubsystem as;
  public ArmTelescopeReset(ElevatorSubsystem as) {
    addRequirements(as);
    this.as=as;
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    as.SetTelescopeState(armTelescopeState.RESET);
  }
  public void end(boolean n) {
    System.out.println("End of subsystem reset");
    as.resetTelescopeEncoder();
    as.SetTelescopeState(armTelescopeState.NONE);
  }
}
