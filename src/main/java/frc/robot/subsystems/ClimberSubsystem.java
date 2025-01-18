// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase 
{

  private SparkMax climberMotor;
  private RelativeEncoder climberEncoder;
  private SparkClosedLoopController climberPID;
  

  /** Creates a new Climber. */
  public ClimberSubsystem()
  {
    climberMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberPID = climberMotor.getClosedLoopController();

    climberMotor.set(Constants.ClimberConstants.ClimberPIDs.P);
    climberMotor.set(Constants.ClimberConstants.ClimberPIDs.I);
    climberMotor.set(Constants.ClimberConstants.ClimberPIDs.D);
    climberMotor.set(Constants.ClimberConstants.ClimberPIDs.kFF);
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
