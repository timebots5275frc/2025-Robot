// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase 
{

  private SparkMax climberLeftMotor;
  private CANcoder climberEncoder;
  private SparkClosedLoopController climberLeftPID;
  private RelativeEncoder climberLeftMotorEncoder;
  private RelativeEncoder climberRightMotorEncoder;
  private SparkMax climberRightMotor;
  private SparkClosedLoopController climberRightPID;

  public double lClimberPose;
  public double rClimberPose;

  ClimbState state;
 
  public enum ClimbState
  {
    NONE,
    CLIMB_ONE_MODE,
    CLIMB_TWO_MODE,
    RETRACT;
  }

  public ClimberSubsystem()
  {
    climberLeftMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_LEFT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberLeftMotorEncoder = climberLeftMotor.getEncoder();
    climberLeftPID = climberLeftMotor.getClosedLoopController();
    Constants.ClimberConstants.CLIMBER_LEFT_PID.setSparkMaxPID(climberLeftMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    climberEncoder = new CANcoder(ClimberConstants.CLIMBER_ENCODER_ID);

    climberRightMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_RIGHT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberRightMotorEncoder = climberRightMotor.getEncoder();
    climberRightPID = climberRightMotor.getClosedLoopController();
    ClosedLoopConfig clRight;
    clRight = new ClosedLoopConfig();
    SparkMaxConfig smcRight;
    smcRight = new SparkMaxConfig();
    smcRight.follow(ClimberConstants.CLIMBER_LEFT_MOTOR_ID,true);
    smcRight.apply(clRight);
    climberRightMotor.configure(smcRight, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    state = ClimbState.NONE;
  }
  public void setClimbState(ClimbState climb) {
    state=climb;
    Climb();
  }
  public void Climb()
  {
  
    System.out.println(state);

    if(lClimberPose > 140 || lClimberPose < 0){climberLeftPID.setReference(0, ControlType.kCurrent);}
    
    else{
    switch(state)
    {
      //NONE
      case NONE: 
        climberLeftPID.setReference(0, ControlType.kCurrent);
      break;
      //CLIMB ONE MODE
      case CLIMB_ONE_MODE:
        climberLeftPID.setReference(-4, ControlType.kCurrent);
      break;
      //CLIMB TWO MODE
      case CLIMB_TWO_MODE:
        if(lClimberPose <= 60)
          {climberLeftPID.setReference(-4, ControlType.kCurrent);}
        
        else if(lClimberPose > 60)
          {climberLeftPID.setReference(-8, ControlType.kCurrent);}
      break;
      //RETRACT
      case RETRACT: 
        climberLeftPID.setReference(/*ClimberConstants.CLIMBER_UP_POS*/-4, ControlType.kCurrent);        
      break;
    }
  }
    
  }

  @Override
  public void periodic() 
  {
    climberLeftMotorEncoder.setPosition(climberEncoder.getAbsolutePosition().getValueAsDouble()*360*Constants.CLIMBER_ROTATIONS_PER_DEGREE);
    
    lClimberPose = climberEncoder.getPosition().getValueAsDouble();
    //rClimberPose = climberRightEncoder.getPosition();

    //System.out.println("CRM Position: " + climberRightMotorEncoder.getPosition() + "CRM Velocity: " + climberRightMotorEncoder.getVelocity());
    //System.out.println("E Position: " + climberEncoder.getPosition() + "CLM Velocity: " + climberEncoder.getVelocity());
  }

    public ClimbState climbState() { return state; }
    public double leftClimberRotations() { return lClimberPose; }
    public double rightClimberRotations() { return rClimberPose; }
}
