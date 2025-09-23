// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem.IntakePivotState;
import frc.robot.subsystems.AlgaeIntakeSubsystem.IntakeRunstate;
import frc.robot.subsystems.ElevatorSubsystem.armIntakeState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeIntakePivotCommand extends InstantCommand {
  private AlgaeIntakeSubsystem ais;
  private IntakePivotState ips;
  private IntakeRunstate irs=null;
  public AlgaeIntakePivotCommand(AlgaeIntakeSubsystem ais, IntakePivotState ips) {
    addRequirements(ais);
    this.ais=ais;this.ips=ips; 
  }
  public AlgaeIntakePivotCommand(AlgaeIntakeSubsystem ais, IntakePivotState ips, IntakeRunstate irs) {
    addRequirements(ais);
    this.ais=ais;this.ips=ips;this.irs=irs;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ais.SetIntakePivotState(ips);
    if (irs!=null) ais.SetIntakeRunState(irs);// i will never do my taxes
    
  }
}
