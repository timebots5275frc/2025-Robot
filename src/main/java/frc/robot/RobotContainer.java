// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmTelescopeSet;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimberSet;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.ArmPivotCommand;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.commands.ArmTelescopeReset;
import frc.robot.subsystems.ArmSubsystem.*;
import frc.robot.subsystems.ClimberSubsystem.ClimbState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Input.Input;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  public RobotContainer() {
    joy = new Joystick(0);
    bBoard = new GenericHID(1);
    
    
    in = new Input(joy);
    sd = new SwerveDrive();
    //intake = new AlgaeIntakeSubsystem();
    as = new ArmSubsystem();
    
    configureBindings();
    
    
  }
  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    
    tjd = new TeleopJoystickDrive(sd, in, true);
    sd.setDefaultCommand(tjd);
    sd.resetPigeon();
    /*tmp */
    new JoystickButton(joy, 8).onTrue(new InstantCommand(sd::resetPigeon, sd));
    new JoystickButton(bBoard, 3).onTrue(new ArmTelescopeSet(as, armTelescopeState.L1));
    new JoystickButton(bBoard, 4).onTrue(new ArmTelescopeSet(as, armTelescopeState.L2));
    new JoystickButton(bBoard, 5).onTrue(new ArmTelescopeSet(as, armTelescopeState.L3));
    new JoystickButton(bBoard, 6).onTrue(new ArmTelescopeSet(as, armTelescopeState.L4));
    new JoystickButton(bBoard, 7).whileTrue(new ArmTelescopeReset(as));
    new JoystickButton(bBoard, 8).onTrue(new ArmTelescopeSet(as, armTelescopeState.DRIVE));
    // 9
    new JoystickButton(bBoard, 9).onTrue(new ArmPivotCommand(as,armPivotState.INTAKE_ANGLE));
  }
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
