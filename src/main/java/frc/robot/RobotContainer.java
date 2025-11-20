// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


// change notes:
// the algae system for disabling algae motor must be changed. it should be done by two different commands.
// elevator use two motors.
// change motor ids to match the constants.
// remove algae limit switch code


import frc.robot.Constants.MathConstants;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.ArmTelescopeSet;
// import frc.robot.commands.AutoCommands;
import frc.robot.commands.auto.AutoDrive;
import frc.robot.commands.AlgaeState;
import frc.robot.commands.COSC;
import frc.robot.commands.CSC;
import frc.robot.commands.ElevatorState;
// import frc.robot.commands.AlgaeState;
// import frc.robot.commands.ArmIntakeCommand;
// import frc.robot.commands.ArmPivotCommand;
import frc.robot.commands.TeleopJoystickDrive;
//import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakePivotState;
// import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakeRunState;
// import frc.robot.subsystems.ElevatorSubsystem.*;
// import frc.robot.subsystems.AlgaeIntakeSubsystem;
// import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.CoralIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LaserCANSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakePivotState;
import frc.robot.subsystems.AlgaeIntakeSubsystem.AlgaeIntakeRunState;
import frc.robot.subsystems.CoralIntakeSubsystem.CoralIntakeState;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorHeightState;
import frc.robot.subsystems.Input.Input;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class RobotContainer {
  //private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  //private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
      Joystick joy;
      Input in;
      TeleopJoystickDrive tjd;
      SwerveDrive sd;
      GenericHID bBoard;
      ElevatorSubsystem es;
      AlgaeIntakeSubsystem ais;
      CoralIntakeSubsystem cis;
      LaserCANSubsystem lcs;
      
  public RobotContainer(SendableChooser<Command> autonChooser) {
    joy = new Joystick(0);
    bBoard = new GenericHID(1);
    
    in = new Input(joy);
    sd = new SwerveDrive();
    ais = new AlgaeIntakeSubsystem();
    es = new ElevatorSubsystem();
    cis = new CoralIntakeSubsystem();
    lcs = new LaserCANSubsystem();
    

    // autonChooser.setDefaultOption("Drive", new SequentialCommandGroup(
    //   new ParallelCommandGroup(
    //     new AutoDrive(MathConstants.INCH_TO_METER*53,.5,sd).withTimeout(7)
    // )));

    autonChooser.setDefaultOption("Drive ONLY", new SequentialCommandGroup(
      new AutoDrive(MathConstants.INCH_TO_METER*53,.5,sd)
    ));
    // autonChooser.setDefaultOption("Drive Score L4", AutoCommands.MiddleCoralL4());
    // autonChooser.addOption("arm LS test", AutoCommands.LSTesterMaBobThing());

    SmartDashboard.putData(autonChooser);
    
    configureBindings();
    
  }
  private void configureBindings() {
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
   
    tjd = new TeleopJoystickDrive(sd, in, true);
    sd.setDefaultCommand(tjd);
    sd.resetPigeon();
    new JoystickButton(joy, 7).onTrue(new InstantCommand(sd::flipFieldRelative ,sd));
    /*tmp */
    //pigeon
    new JoystickButton(joy, 8).onTrue(new InstantCommand(sd::resetPigeon, sd));
  
    //Elevator
    // new JoystickButton(bBoard, Constants.ButtonConstants.ELEVATOR_L1).onTrue(new ElevatorState(es, ElevatorHeightState.L1));
    new JoystickButton(bBoard, Constants.ButtonConstants.ELEVATOR_L2).onTrue(new ElevatorState(es, ElevatorHeightState.L2));
    new JoystickButton(bBoard, Constants.ButtonConstants.ELEVATOR_L3).onTrue(new ElevatorState(es, ElevatorHeightState.L3));
    new JoystickButton(bBoard, Constants.ButtonConstants.ELEVATOR_L4).onTrue(new ElevatorState(es, ElevatorHeightState.L4));
    // new JoystickButton(bBoard, Constants.ButtonConstants.ELEVATOR_DRIVE).onTrue(new ElevatorState(es, ElevatorHeightState.DRIVE));
    new JoystickButton(bBoard, Constants.ButtonConstants.ELEVATOR_INTAKE).onTrue(new ElevatorState(es, ElevatorHeightState.INTAKE));

    //Coral Intake
    new JoystickButton(joy, Constants.ButtonConstants.CORAL_NONE).onTrue(new COSC(cis, CoralIntakeState.NONE));
    new JoystickButton(joy, Constants.ButtonConstants.CORAL_INTAKE).onTrue(new CSC(cis, CoralIntakeState.INTAKE).until(cis.CoralOutOfWay));
    new JoystickButton(joy, Constants.ButtonConstants.CORAL_OUTTAKE_L1).onTrue(new COSC(cis, CoralIntakeState.OUTTAKE_L1));
    new JoystickButton(joy, Constants.ButtonConstants.CORAL_OUTTAKE_L2_TO_L4).onTrue(new COSC(cis, CoralIntakeState.OUTTAKE_L2_TO_4)); 

    //Algae Intake
    new JoystickButton(bBoard, Constants.ButtonConstants.ALGAE_INTAKE_INTAKE).onTrue(new AlgaeState(ais, AlgaeIntakeRunState.INTAKE));
    new JoystickButton(bBoard, Constants.ButtonConstants.ALGAE_INTAKE_OUTTAKE).onTrue(new AlgaeState(ais, AlgaeIntakeRunState.OUTTAKE));
    new JoystickButton(bBoard, Constants.ButtonConstants.ALGAE_INTAKE_NONE).onTrue(new AlgaeState(ais, AlgaeIntakeRunState.NONE));

    //Algae Pivot
    new JoystickButton(bBoard, Constants.ButtonConstants.ALGAE_PIVOT_DRIVE).onTrue(new AlgaeState(ais, AlgaeIntakePivotState.DRIVE));
    new JoystickButton(bBoard, Constants.ButtonConstants.ALGAE_PIVOT_GROUND).onTrue(new AlgaeState(ais, AlgaeIntakePivotState.GROUND));
    new JoystickButton(bBoard, Constants.ButtonConstants.ALGAE_PIVOT_REEF).onTrue(new AlgaeState(ais, AlgaeIntakePivotState.REEF));
    new JoystickButton(bBoard, Constants.ButtonConstants.ALGAE_PIVOT_SHOOT).onTrue(new AlgaeState(ais, AlgaeIntakePivotState.SHOOT));
  }
  public Command getAutonomousCommand(SendableChooser<Command> autonChooser) 
  {
    return autonChooser.getSelected(); 
  }
}
