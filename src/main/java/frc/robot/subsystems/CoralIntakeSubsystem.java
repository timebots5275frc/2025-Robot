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

  CoralIntakeState cisc;

  private SparkMax IntakeMotorOne;
  private RelativeEncoder IntakeEncoderOne;
  private SparkClosedLoopController IntakePIDOne;
  private DigitalInput limswitch1;
  private DigitalInput limswitch2;
  private SparkMax IntakeMotorTwo;
  private RelativeEncoder IntakeEncoderTwo;
  private SparkClosedLoopController IntakePIDTwo;
  private LaserCan lc1;
  private LaserCan lc2;

  public enum CoralIntakeState
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  /** Creates a new CoralIntakeSubsystem. */
  public CoralIntakeSubsystem() 
  {
    lc1 = new LaserCan(Constants.CoralIntakeConstants.CORAL_INTAKE_LASERCAN_ID1);
    lc2 = new LaserCan(Constants.CoralIntakeConstants.CORAL_INTAKE_LASERCAN_ID2);

    IntakeMotorOne = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    Constants.CoralIntakeConstants.CORAL_INTAKE_PID.setSparkMaxPID(IntakeMotorOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakeEncoderOne = IntakeMotorOne.getEncoder();
    IntakePIDOne = IntakeMotorOne.getClosedLoopController();

    
    IntakeMotorTwo = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
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
      
      case NONE:   IntakePIDOne.setReference(0,ControlType.kVelocity); 
      break;
      case INTAKE: IntakePIDOne.setReference(CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;
      case OUTTAKE:IntakePIDOne.setReference(-CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED,ControlType.kVelocity);
      break;
    }
  }

  @Override
  public void periodic() 
  {
    lc1.getMeasurement();
    // System.out.println(lc1.getMeasurement());
    lc2.getMeasurement();
    // System.out.println(lc2.getMeasurement());
  }
}
