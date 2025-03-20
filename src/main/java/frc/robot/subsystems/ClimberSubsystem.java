// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase 
{

  private SparkMax climberLeftMotor;
  private CANcoder climberEncoder;
  private SparkClosedLoopController climberLeftPID;

  private SparkMax climberRightMotor;
  private SparkClosedLoopController climberRightPID;

  public double lClimberPose;
  public double rClimberPose;

  ClimbState state;
  

  /** Creates a new Climber. */

  public enum ClimbState
  {
    NONE,
    CLIMB,
    RETRACT,
    RESET;
  }

  public ClimberSubsystem()
  {
    climberLeftMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberEncoder = new CANcoder(ClimberConstants.CLIMBER_ENCODER_ID);
    climberLeftPID = climberLeftMotor.getClosedLoopController();

    Constants.ClimberConstants.CLIMBER_LEFT_PID.setSparkMaxPID(climberLeftMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberRightMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    //climberRightEncoder = climberRightMotor.getEncoder();
    climberRightPID = climberRightMotor.getClosedLoopController();

    Constants.ClimberConstants.CLIMBER_RIGHT_PID.setSparkMaxPID(climberRightMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);   
  }
  public void setClimbState(ClimbState climb) {
    state=climb;
  }
  public void Climb()
  {
    switch(state)
    {
      //NONE
      case NONE: 
        climberLeftPID.setReference(0, ControlType.kCurrent);
        climberRightPID.setReference(0, ControlType.kCurrent);
      break;
      //CLIMB
      case CLIMB:
        if (rClimberPose > ClimberConstants.MAX_CLIMBER_POSE || lClimberPose > ClimberConstants.MAX_CLIMBER_POSE)  {
          climberLeftPID.setReference(0, ControlType.kVelocity);
          climberRightPID.setReference(0, ControlType.kVelocity);
          state = ClimbState.NONE;

        } else {
          climberLeftPID.setReference(ClimberConstants.climberSpeed, ControlType.kVelocity);
          climberLeftPID.setReference(-ClimberConstants.climberSpeed, ControlType.kVelocity);
        }
      break;

      case RETRACT: 
      if (rClimberPose < ClimberConstants.MIN_CLIMBER_POSE || lClimberPose < ClimberConstants.MIN_CLIMBER_POSE) {
          climberLeftPID.setReference(0, ControlType.kVelocity);
          climberRightPID.setReference(0, ControlType.kVelocity);
          state = ClimbState.NONE;
      }
            climberRightPID.setReference(-Constants.ClimberConstants.climberSpeed, ControlType.kVelocity);
            climberLeftPID.setReference(Constants.ClimberConstants.climberSpeed, ControlType.kVelocity);
      break;

      case RESET: 
            climberRightPID.setReference(-8, ControlType.kCurrent);
            climberLeftPID.setReference(8, ControlType.kCurrent);
            climberEncoder.setPosition(0);
      break;
    }
    
  }

  @Override
  public void periodic() 
  {
    lClimberPose = climberEncoder.getPosition().getValueAsDouble();
    //rClimberPose = climberRightEncoder.getPosition();

    //System.out.println("CRM Position: " + climberRightEncoder.getPosition() + "CRM Velocity: " + climberRightEncoder.getVelocity());
    System.out.println("CLM Position: " + climberEncoder.getPosition() + "CLM Velocity: " + climberEncoder.getVelocity());
  }

    public ClimbState climbState() { return state; }
    public double leftClimberRotations() { return lClimberPose; }
    public double rightClimberRotations() { return rClimberPose; }
}
