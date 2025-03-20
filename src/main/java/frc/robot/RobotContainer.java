// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.TeleopJoystickDrive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.DriveTrain.SwerveDrive;
import frc.robot.subsystems.Input.Input;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
      AlgaeIntakeSubsystem intake;
      Joystick joy;
      Input in;
      TeleopJoystickDrive tjd;
      SwerveDrive sd;

  public RobotContainer() {
    sd = new SwerveDrive();
    intake = new AlgaeIntakeSubsystem();
    configureBindings();
  }
  private void configureBindings() {
    new Trigger(m_exampleSubsystem::exampleCondition).onTrue(new ExampleCommand(m_exampleSubsystem));
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    joy = new Joystick(0);
    tjd = new TeleopJoystickDrive(sd, in, true);
    sd.setDefaultCommand(tjd);

    //none of the buttons are done but just a mere skeleton of what they could be
    //consider adding buttonboard
    new JoystickButton(joy, 0); //arm t
    new JoystickButton(joy, 0); //arm L1
    new JoystickButton(joy, 0); //arm L2
    new JoystickButton(joy, 0); //arm L3
    new JoystickButton(joy, 0); //arm L4
    new JoystickButton(joy, 0); //arm p
    new JoystickButton(joy, 0); //arm i

    new JoystickButton(joy, 0); //algae piv drive
    new JoystickButton(joy, 0); //algae piv pick up
    new JoystickButton(joy, 0); //algae piv in
    new JoystickButton(joy, 0); //algae intake
    new JoystickButton(joy, 0); //algae outtake
    new JoystickButton(joy, 0); //algae stop

    new JoystickButton(joy, 0); //climb
    new JoystickButton(joy, 0); //climb none

  }
  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
