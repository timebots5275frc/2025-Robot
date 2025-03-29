// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberSet extends Command {
  private ClimberSubsystem cs;private ClimbState cst;
  public ClimberSet(ClimberSubsystem cs, ClimbState cst) {
    addRequirements(cs);
    this.cs= cs;
    this.cst= cst;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void execute() {
    cs.setClimbState(cst);
    // cs.setClimbState(ClimberSubsystem.ClimbState.CLIMB_TWO_MODE);
    // cs.setClimbState(ClimberSubsystem.ClimbState.RETRACT);
  }

  @Override
  public void initialize() {/*cs.setClimbState(cst);*/}

  @Override public void end(boolean inter) {
    //if (cst == ClimbState.CLIMB) cs.setClimbState(ClimbState.RETAIN);
  /*else*/ cs.setClimbState(ClimbState.NONE);
  }
}
