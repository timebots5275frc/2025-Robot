// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
  private RelativeEncoder climberLeftEncoder;
  private SparkClosedLoopController climberLeftPID;

  private SparkMax climberRightMotor;
  private RelativeEncoder climberRightEncoder;
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
    climberLeftEncoder = climberLeftMotor.getEncoder();
    climberLeftPID = climberLeftMotor.getClosedLoopController();

    Constants.ClimberConstants.CLIMBER_LEFT_PID.setSparkMaxPID(climberLeftMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    climberRightMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberRightEncoder = climberRightMotor.getEncoder();
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
        climberLeftPID.setReference(0, ControlType.kVelocity);
        climberRightPID.setReference(0, ControlType.kVelocity);
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
            climberRightEncoder.setPosition(0);
            climberLeftEncoder.setPosition(0);
      break;
    }
    
  }

  @Override
  public void periodic() 
  {
    lClimberPose = climberLeftEncoder.getPosition();
    rClimberPose = climberRightEncoder.getPosition();

    System.out.println("CRM Position: " + climberRightEncoder.getPosition() + "CRM Velocity: " + climberRightEncoder.getVelocity());
    System.out.println("CLM Position: " + climberLeftEncoder.getPosition() + "CLM Velocity: " + climberLeftEncoder.getVelocity());
  }

    public ClimbState climbState() { return state; }
    public double leftClimberRotations() { return lClimberPose; }
    public double rightClimberRotations() { return rClimberPose; }
}
