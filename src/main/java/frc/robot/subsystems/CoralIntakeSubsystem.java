// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralIntakeSubsystem extends SubsystemBase {

  CoralIntakeState cisc;

  private SparkMax IntakeMotorOne;
  private RelativeEncoder IntakeEncoderOne;
  private SparkClosedLoopController IntakePIDOne;

  private SparkMax IntakeMotorTwo;
  private RelativeEncoder IntakeEncoderTwo;
  private SparkClosedLoopController IntakePIDTwo;

  public enum CoralIntakeState
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  /** Creates a new CoralIntakeSubsystem. */
  public CoralIntakeSubsystem() 
  {
    IntakeMotorOne = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    IntakeEncoderOne = IntakeMotorOne.getEncoder();
    IntakePIDOne = IntakeMotorOne.getClosedLoopController();
    
    IntakeMotorTwo = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    IntakeEncoderTwo = IntakeMotorTwo.getEncoder();
    IntakePIDTwo = IntakeMotorTwo.getClosedLoopController();

    
  }

  public void setIntakeState(CoralIntakeState state)
  {
    cisc = state;
    CoralIntakeState();
  }

  public void CoralIntakeState()
  {
    switch(cisc)
    {
      
      case NONE: IntakePIDOne.setReference(0, ControlType.kVelocity); 
      break;
      case INTAKE:
      break;
      case OUTTAKE:
      break;
    }
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
