// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MathConstants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.ArmTelescopeSet;
// import frc.robot.commands.AutoCommands;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.AlgaeIntakePivotCommand;
import frc.robot.commands.AlgaeIntakeRunCommand;
// import frc.robot.commands.ArmIntakeCommand;
// import frc.robot.commands.ArmPivotCommand;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem.IntakePivotState;
import frc.robot.subsystems.AlgaeIntakeSubsystem.IntakeRunstate;
import frc.robot.subsystems.ElevatorSubsystem.*;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Input.Input;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      Joystick joy;
      Input in;
      TeleopJoystickDrive tjd;
      SwerveDrive sd;
      GenericHID bBoard;
      ElevatorSubsystem as;
      AlgaeIntakeSubsystem ais;
  public RobotContainer(SendableChooser<Command> autonChooser) {
    joy = new Joystick(0);
    bBoard = new GenericHID(1);
    
    in = new Input(joy);
    sd = new SwerveDrive();
    ais = new AlgaeIntakeSubsystem();
    as = new ElevatorSubsystem();

    // autonChooser.setDefaultOption("Drive Score L4", new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new AutoDrive(MathConstants.INCH_TO_METER*53,.5,sd).withTimeout(7),
    //     new ArmTelescopeSet(as, armTelescopeState.L4)), 
    //     new WaitCommand(.5), 
    //     new ArmIntakeCommand(as, armIntakeState.OUTTAKE),
    //     new WaitCommand(.5),
    //     new ArmIntakeCommand(as, armIntakeState.NONE),
    //     new AutoDrive(MathConstants.INCH_TO_METER*-10,-.5,sd).withTimeout(7),
    //     new ArmTelescopeSet(as,armTelescopeState.INTAKE,armPivotState.INTAKE_ANGLE)
    // ));

    autonChooser.addOption("Drive ONLY", new SequentialCommandGroup(
      new AutoDrive(MathConstants.INCH_TO_METER*53,.5,sd)
    ));
    // autonChooser.setDefaultOption("Drive Score L4", AutoCommands.MiddleCoralL4());
    // autonChooser.addOption("arm LS test", AutoCommands.LSTesterMaBobThing());

    SmartDashboard.putData(autonChooser);
    
    configureBindings();
    
    
  }
  private void configureBindings() {
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
   
    tjd = new TeleopJoystickDrive(sd, in, true);
    sd.setDefaultCommand(tjd);
    sd.resetPigeon();
    new JoystickButton(joy, 2).onTrue(new InstantCommand(sd::flipFieldRelative ,sd));
    /*tmp */
    //pigeon
    new JoystickButton(joy, 7).onTrue(new InstantCommand(sd::resetPigeon, sd));

    // //arm
    // new JoystickButton(joy, 5).onTrue(new ArmTelescopeSet(as, armTelescopeState.L2, armPivotState.OUTTAKE_ANGLE));
    // new JoystickButton(joy, 6).onTrue(new ArmTelescopeSet(as, armTelescopeState.L3, armPivotState.OUTTAKE_ANGLE));
    // new JoystickButton(joy, 3).onTrue(new ArmTelescopeSet(as, armTelescopeState.L4, armPivotState.OUTTAKE_ANGLE));
    // new JoystickButton(joy, 1).onTrue(new SequentialCommandGroup(new ArmIntakeCommand(as, armIntakeState.OUTTAKE),new WaitCommand(.75), new ArmIntakeCommand(as, armIntakeState.NONE)));
    // new JoystickButton(joy, 4).onTrue(new ArmTelescopeSet(as, armTelescopeState.INTAKE, armPivotState.INTAKE_ANGLE, armIntakeState.INTAKE)/*.until(ArmSubsystem.limitSwitchisPressed())*/); // working on this
    // new JoystickButton(joy, 4).onTrue(new ArmTelescopeSet(as, armTelescopeState.INTAKE, armPivotState.INTAKE_ANGLE, armIntakeState.INTAKE));

    // new JoystickButton(bBoard, 8).onTrue(new ArmTelescopeSet(as, armTelescopeState.DRIVE));
    
    //algae
    new JoystickButton(bBoard, 10).onTrue(new AlgaeIntakePivotCommand(ais, IntakePivotState.PICKUP,IntakeRunstate.INTAKE));
    new JoystickButton(bBoard, 12).onTrue(new SequentialCommandGroup(new AlgaeIntakePivotCommand(ais, IntakePivotState.DRIVE,IntakeRunstate.OUTTAKE),new WaitCommand(.75), new AlgaeIntakePivotCommand(ais, IntakePivotState.DRIVE,IntakeRunstate.NONE)));
    new JoystickButton(bBoard, 11).onTrue(new AlgaeIntakeRunCommand(ais, IntakeRunstate.NONE));

  }
  public Command getAutonomousCommand(SendableChooser<Command> autonChooser) 
  {
    return autonChooser.getSelected(); 
  }
}
