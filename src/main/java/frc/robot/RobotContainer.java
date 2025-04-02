// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MathConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ArmTelescopeSet;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.ClimberSet;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Adrian;
import frc.robot.commands.AlgaeIntakePivotCommand;
import frc.robot.commands.AlgaeIntakeRunCommand;
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
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
      ClimberSubsystem cs;
      Adrian adrian;

  public RobotContainer(SendableChooser<Command> autonChooser) {
    joy = new Joystick(0);
    bBoard = new GenericHID(1);
    cs=new ClimberSubsystem();
    in = new Input(joy);
    sd = new SwerveDrive();
    ais = new AlgaeIntakeSubsystem();
    as = new ArmSubsystem();

    autonChooser.setDefaultOption("Drive Score L4", AutoCommands.MiddleCoralL4());
    autonChooser.addOption("arm LS test", AutoCommands.LSTesterMaBobThing());
    autonChooser.addOption("L4 and Processor", AutoCommands.L4AndL3Algae());
    // autonChooser.addOption("2 Piece L4", AutoCommands.TwoCoralL4());

    SmartDashboard.putData(autonChooser);
    
    configureBindings();
    
    
  }
  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
   
    tjd = new TeleopJoystickDrive(sd, in, true);
    sd.setDefaultCommand(tjd);
    sd.resetPigeon();
    new JoystickButton(joy, 2).onTrue(new InstantCommand(sd::flipFieldRelative ,sd));
    /*tmp */
    //pigeon
    new JoystickButton(joy, 7).onTrue(new InstantCommand(sd::resetPigeon, sd));

    //arm
    // new JoystickButton(bBoard, 3).onTrue(new ArmTelescopeSet(as, armTelescopeState.L1));
    new JoystickButton(joy, 5).onTrue(new ArmTelescopeSet(as, armTelescopeState.L2, armPivotState.L2_ANGLE));
    new JoystickButton(joy, 6).onTrue(new ArmTelescopeSet(as, armTelescopeState.L3));
    new JoystickButton(joy, 3).onTrue(new ArmTelescopeSet(as, armTelescopeState.L4));
    new JoystickButton(joy, 8).whileTrue(new ArmTelescopeReset(as));
    new JoystickButton(bBoard, 5).onTrue(new SequentialCommandGroup(new ArmPivotCommand(as,armPivotState.NONE),new AlgaeIntakePivotCommand(ais, IntakePivotState.DRIVE)));
    new JoystickButton(joy, 1).onTrue(new SequentialCommandGroup(new ArmIntakeCommand(as, armIntakeState.OUTTAKE),new WaitCommand(.75), new ArmIntakeCommand(as, armIntakeState.NONE)));
    new JoystickButton(joy, 4).onTrue(new ArmTelescopeSet(as, armTelescopeState.INTAKE, armPivotState.INTAKE_ANGLE, armIntakeState.INTAKE));

    new JoystickButton(joy, 10).onTrue(new ArmTelescopeSet(as,armTelescopeState.REMOVE_ALGAE2,armPivotState.L2BALLREMOVAL));
    new JoystickButton(joy, 9).onTrue(new ArmTelescopeSet(as,armTelescopeState.REMOVE_ALGAE,armPivotState.L2BALLREMOVAL,armIntakeState.INTAKE));
    new JoystickButton(bBoard, 3).onTrue(new SequentialCommandGroup(new ArmPivotCommand(as,armPivotState.L2BALLREMOVAL),new ArmIntakeCommand(as, armIntakeState.OUTTAKE))).onFalse(new ArmIntakeCommand(as, armIntakeState.NONE));
    new JoystickButton(bBoard, 8).onTrue(new ArmTelescopeSet(as, armTelescopeState.DRIVE));
    
    //algae
    new JoystickButton(bBoard, 10).onTrue(new AlgaeIntakePivotCommand(ais, IntakePivotState.PICKUP,IntakeRunstate.INTAKE));
    new JoystickButton(bBoard, 12).onTrue(new SequentialCommandGroup(new AlgaeIntakePivotCommand(ais, IntakePivotState.DRIVE,IntakeRunstate.OUTTAKE),new WaitCommand(.75), new AlgaeIntakePivotCommand(ais, IntakePivotState.DRIVE,IntakeRunstate.NONE)));
    new JoystickButton(bBoard, 11).onTrue(new AlgaeIntakeRunCommand(ais, IntakeRunstate.NONE));
    //climber
    new JoystickButton(bBoard,6).whileTrue(new ClimberSet(cs, ClimbState.RETRACT));
    new JoystickButton(bBoard, 7).whileTrue(new ClimberSet(cs, ClimbState.CLIMB_ONE_MODE));
    new JoystickButton(bBoard, 8).whileTrue(new ClimberSet(cs, ClimbState.CLIMB_TWO_MODE));
  }
  public Command getAutonomousCommand(SendableChooser<Command> autonChooser) 
  {
    return autonChooser.getSelected();
  }
}
