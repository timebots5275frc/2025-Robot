// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
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
    RETRACT;
  }

  public ClimberSubsystem()
  {
    climberLeftMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberLeftEncoder = climberLeftMotor.getEncoder();
    climberLeftPID = climberLeftMotor.getClosedLoopController();
    ClosedLoopConfig climberLCLC = new ClosedLoopConfig();
    SparkMaxConfig climberLSMC = new SparkMaxConfig();
    climberLCLC.pidf(ClimberConstants.ClimberLeftPIDs.P, ClimberConstants.ClimberLeftPIDs.I, ClimberConstants.ClimberLeftPIDs.D, ClimberConstants.ClimberLeftPIDs.kFF);
    climberLCLC.iZone(ClimberConstants.ClimberLeftPIDs.IZ);
    climberLSMC.apply(climberLCLC);
    climberLeftMotor.configure(climberLSMC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    

    climberRightMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberRightEncoder = climberRightMotor.getEncoder();
    climberRightPID = climberRightMotor.getClosedLoopController();
    ClosedLoopConfig climberRCLC = new ClosedLoopConfig();
    SparkMaxConfig climberRSMC = new SparkMaxConfig();
    climberRCLC.pidf(ClimberConstants.ClimberRightPIDs.P, ClimberConstants.ClimberRightPIDs.I, ClimberConstants.ClimberRightPIDs.D, ClimberConstants.ClimberRightPIDs.kFF);
    climberRCLC.iZone(ClimberConstants.ClimberRightPIDs.IZ);
    climberRSMC.apply(climberRCLC);
    climberRightMotor.configure(climberRSMC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);   
  }

  public void Climb(ClimbState state)
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
        rClimberPose = climberRightEncoder.getPosition();
        if(rClimberPose < Constants.ClimberConstants.MAX_CLIMBER_POSE)
        {
          climberRightPID.setReference(Constants.ClimberConstants.climberSpeed, ControlType.kVelocity);
          climberLeftPID.setReference(-Constants.ClimberConstants.climberSpeed, ControlType.kVelocity); 
        }
        else
        {
          climberLeftPID.setReference(0, ControlType.kVelocity);
          climberRightPID.setReference(0, ControlType.kVelocity);
        }
      break;

      case RETRACT: 
            climberRightPID.setReference(-Constants.ClimberConstants.climberSpeed, ControlType.kCurrent);
            climberLeftPID.setReference(Constants.ClimberConstants.climberSpeed, ControlType.kCurrent);
      break;
    }
    
  }

  @Override
  public void periodic() 
  {
    System.out.println("CRM Position: " + climberRightEncoder.getPosition() + "CRM Velocity: " + climberRightEncoder.getVelocity());
    System.out.println("CLM Position: " + climberLeftEncoder.getPosition() + "CLM Velocity: " + climberLeftEncoder.getVelocity());

    // public ClimbState state() { return state; }
    // public double leftClimberRotations() { return lClimberPose; }
    // public double rightClimberRotations() { return rClimberPose; }
  }
}
