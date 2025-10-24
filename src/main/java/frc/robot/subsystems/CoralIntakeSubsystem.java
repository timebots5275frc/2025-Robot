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
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase {

  LaserCANSubsystem lcs = new LaserCANSubsystem();

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
    IntakeMotorOne = new SparkMax(Constants.CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID1, SparkLowLevel.MotorType.kBrushless);
    Constants.CoralIntakeConstants.CORAL_INTAKE_PID.setSparkMaxPID(IntakeMotorOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakeEncoderOne = IntakeMotorOne.getEncoder();
    IntakePIDOne = IntakeMotorOne.getClosedLoopController();
    
    IntakeMotorTwo = new SparkMax(Constants.CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID2, SparkLowLevel.MotorType.kBrushless);
    Constants.CoralIntakeConstants.CORAL_INTAKE_PID.setSparkMaxPID(IntakeMotorOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakeEncoderTwo = IntakeMotorTwo.getEncoder();
    IntakePIDTwo = IntakeMotorTwo.getClosedLoopController();
  }

  public void SetCoralIntakeState(CoralIntakeState state)
  {
    cisc = state;
    UpdateCoralIntakeState();
  }

  private void UpdateCoralIntakeState()
  {
    switch(cisc)
    {
      case NONE:   IntakePIDOne.setReference(0,ControlType.kCurrent); 
      break;
      case INTAKE: if(lcs.LC1() == true){IntakePIDOne.setReference(CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED, ControlType.kVelocity);} 
                   else{IntakePIDOne.setReference(0,ControlType.kCurrent);}
      break;
      case OUTTAKE: IntakePIDOne.setReference(-CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED,ControlType.kVelocity);
      break;
    }
  }

  @Override
  public void periodic() 
  {
  
  }
}
