// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmTelescopeSet;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberSet;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.NathansOuttakeCommand;
import frc.robot.commands.AlgaeIntakePivotCommand;
import frc.robot.commands.ArmIntakeCommand;
import frc.robot.commands.ArmPivotCommand;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem.IntakePivotState;
import frc.robot.subsystems.AlgaeIntakeSubsystem.IntakeRunstate;
import frc.robot.commands.ArmTelescopeReset;
import frc.robot.subsystems.ArmSubsystem.*;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Input.Input;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      //AlgaeIntakeSubsystem intake;
      Joystick joy;
      Input in;
      TeleopJoystickDrive tjd;
      SwerveDrive sd;
      GenericHID bBoard;
      ArmSubsystem as;
      AlgaeIntakeSubsystem ais;
  public RobotContainer() {
    joy = new Joystick(0);
    bBoard = new GenericHID(1);
    
    
    in = new Input(joy);
    sd = new SwerveDrive();
    ais = new AlgaeIntakeSubsystem();
    as = new ArmSubsystem();
    
    configureBindings();
    
    
  }
  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    tjd = new TeleopJoystickDrive(sd, in, true);
    sd.setDefaultCommand(tjd);
    sd.resetPigeon();
    /*tmp */
    new JoystickButton(joy, 8).onTrue(new InstantCommand(sd::resetPigeon, sd));
    new JoystickButton(bBoard, 3).onTrue(new ArmTelescopeSet(as, armTelescopeState.L1));
    new JoystickButton(bBoard, 11).onTrue(new ArmTelescopeSet(as, armTelescopeState.L2, armPivotState.L2_ANGLE));
    new JoystickButton(bBoard, 9).onTrue(new ArmTelescopeSet(as, armTelescopeState.L3));
    new JoystickButton(bBoard, 7).onTrue(new ArmTelescopeSet(as, armTelescopeState.L4));
    new JoystickButton(joy, 7).whileTrue(new ArmTelescopeReset(as));
    new JoystickButton(bBoard, 10).onTrue(new AlgaeIntakePivotCommand(ais, IntakePivotState.PICKUP,IntakeRunstate.INTAKE));
    new JoystickButton(bBoard, 12).onTrue(new SequentialCommandGroup(new AlgaeIntakePivotCommand(ais, IntakePivotState.DRIVE,IntakeRunstate.OUTTAKE),new WaitCommand(.75), new AlgaeIntakePivotCommand(ais, IntakePivotState.DRIVE,IntakeRunstate.NONE)));
    new JoystickButton(bBoard, 8).onTrue(new ArmTelescopeSet(as, armTelescopeState.DRIVE));
    new JoystickButton(joy, 5).onTrue(new SequentialCommandGroup(new ArmIntakeCommand(as, armIntakeState.OUTTAKE),new WaitCommand(.75), new ArmIntakeCommand(as, armIntakeState.NONE)));
    new JoystickButton(joy, 4).onTrue(new ArmTelescopeSet(as, armTelescopeState.INTAKE, armPivotState.INTAKE_ANGLE, armIntakeState.INTAKE));
    new JoystickButton(bBoard, 11).onTrue(new ArmPivotCommand(as,armPivotState.OUTTAKE_ANGLE));
    new JoystickButton(bBoard, 5).onTrue(new ArmPivotCommand(as,armPivotState.NONE));
  }
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
