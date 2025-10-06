// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakePivotState;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakeRunState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorHeightState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlgaeState extends InstantCommand {
  AlgaeIntakeSubsystem ais;
  ElevatorSubsystem es;
  ElevatorHeightState ehs;
  AlgaeIntakeRunState airs;
  AlgaeIntakePivotState aips;
  public AlgaeState(AlgaeIntakeSubsystem ais, ElevatorSubsystem es, AlgaeIntakeRunState airs, AlgaeIntakePivotState aips, ElevatorHeightState ehs) {
    this.ais=ais;
    this.es=es;
    this.ehs=ehs;
    this.airs=airs;
    this.aips=aips;
  }
  public AlgaeState(AlgaeIntakeSubsystem ais, AlgaeIntakeRunState airs, AlgaeIntakePivotState aips) {
    this.ais=ais;
    this.airs=airs;
    this.aips=aips;
  } 
  public AlgaeState(AlgaeIntakeSubsystem ais, AlgaeIntakeRunState airs) {
    this.ais=ais;
    this.airs=airs;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ais.SetAlgaeIntakeRunState(airs);
    if (aips != null)ais.SetAlgaeIntakePivotState(aips);
    if (es!=null&&ehs!=null)es.SetHeightState(ehs);
  }
}
