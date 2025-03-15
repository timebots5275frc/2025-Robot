// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ArmSubsystem.armIntakeState;
import frc.robot.subsystems.ArmSubsystem.armPivotState;
import frc.robot.subsystems.ArmSubsystem.armTelescopeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ArmTelescopeSet extends InstantCommand {
  ArmSubsystem as;
  armTelescopeState ats; 
  armPivotState ps = null; 
  armIntakeState is = null;
  public ArmTelescopeSet(ArmSubsystem as, armTelescopeState ats) {
    this.as=as;this.ats=ats;this.ps=armPivotState.OUTTAKE_ANGLE;
    addRequirements(as);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ArmTelescopeSet(ArmSubsystem as, armTelescopeState ats, armPivotState ps) {
    this.as=as;this.ats=ats;this.ps=ps;
    addRequirements(as);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public ArmTelescopeSet(ArmSubsystem as, armTelescopeState ats, armPivotState ps, armIntakeState is) {
    this.as=as;this.ats=ats;this.is=is;this.ps=ps;
    addRequirements(as);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(is != null)
    {as.SetIntakeState(is);}
    if(ps != null)
    {as.SetPivotState(ps);}
    as.SetTelescopeState(ats);
  }

}
