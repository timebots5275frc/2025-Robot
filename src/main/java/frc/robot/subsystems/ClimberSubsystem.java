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

  private double lClimberPose;
  private double rClimberPose;

  ClimbState state;
  

  /** Creates a new Climber. */

  public enum ClimbState
  {
    NONE,
    CLIMB;
  }

  public ClimberSubsystem()
  {
    climberLeftMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberLeftEncoder = climberLeftMotor.getEncoder();
    climberLeftPID = climberLeftMotor.getClosedLoopController();

    climberRightMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberRightEncoder = climberRightMotor.getEncoder();
    climberRightPID = climberRightMotor.getClosedLoopController();

    
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
    }
    ClosedLoopConfig climberCLC = new ClosedLoopConfig();
    SparkMaxConfig climberSMC = new SparkMaxConfig();
    climberCLC.pidf(ClimberConstants.ClimberPIDs.P, ClimberConstants.ClimberPIDs.I, ClimberConstants.ClimberPIDs.D, ClimberConstants.ClimberPIDs.kFF);
    climberCLC.iZone(ClimberConstants.ClimberPIDs.IZ);
    climberSMC.apply(climberCLC);
    climberMotor.configure(climberSMC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
