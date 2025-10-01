// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem.CoralIntakeState;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorHeightState;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralState extends InstantCommand {
  ElevatorSubsystem es;
  ElevatorHeightState ehs;
  CoralIntakeState cis;
  CoralIntakeSubsystem ci_sub;
  public CoralState(CoralIntakeSubsystem ci_sub, ElevatorSubsystem es, CoralIntakeState cis,ElevatorHeightState ehs) {
    this.ci_sub=ci_sub;
    this.cis=cis;
    this.es=es;
    this.ehs=ehs;
  }
  public CoralState(CoralIntakeSubsystem ci_sub, CoralIntakeState cis) {
    this.ci_sub=ci_sub;
    this.cis=cis;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ci_sub.SetCoralIntakeState(cis);
    if (es!=null&&ehs!=null)es.SetHeightState(ehs);
  }
}
