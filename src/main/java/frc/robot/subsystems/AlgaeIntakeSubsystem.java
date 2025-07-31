// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Queue;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeIntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase 
{
  IntakePivotState currentPivotState;
  IntakeRunstate   currentRunState;
  private DigitalInput limitswitch1,limitswitch2;
  private SparkMax intakeRunMotor;
  private RelativeEncoder intakeRunEncoder;
  private SparkClosedLoopController intakeRunPID;
  private CANcoder intakePivotEncoder;
  private SparkMax intakePivotMotor;
  private RelativeEncoder intakePivotMotorEncoder;
  private SparkClosedLoopController intakePivotPID;
  private SparkMaxConfig sparkmaxconfig;

  /** Creates a new Intake. */
  public enum IntakePivotState
  {
    DRIVE,
    PICKUP,
    SHOOT;
  }

  public enum IntakeRunstate
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  public AlgaeIntakeSubsystem() 
  {
    intakePivotEncoder = new CANcoder(AlgaeIntakeConstants.ALGAE_PIVOT_MOTOR_ENCODER_ID);
    limitswitch1 = new DigitalInput(AlgaeIntakeConstants.ALGAE_INTAKE_SWITCH1_PORT);
    limitswitch2 = new DigitalInput(AlgaeIntakeConstants.ALGAE_INTAKE_SWITCH2_PORT);
    intakePivotMotor = new SparkMax(Constants.AlgaeIntakeConstants.ALGAE_PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    intakePivotMotorEncoder = intakePivotMotor.getEncoder();
    intakePivotPID = intakePivotMotor.getClosedLoopController();

    sparkmaxconfig.idleMode(IdleMode.kBrake);
    sparkmaxconfig.smartCurrentLimit(20, 20, 2500);
    Constants.AlgaeIntakeConstants.ALGAE_INTAKE_PIVOT_PID.setSparkMaxPID(intakePivotMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters, IdleMode.kCoast);

    intakeRunMotor = new SparkMax(Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    intakeRunEncoder = intakeRunMotor.getEncoder();
    intakeRunPID = intakeRunMotor.getClosedLoopController();

    Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_PID.setSparkMaxPID(intakeRunMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    currentPivotState = IntakePivotState.DRIVE;
    currentRunState   = IntakeRunstate.NONE;
  }

  public void SetIntakePivotState(IntakePivotState state) 
  {
    currentPivotState = state;
    IntakePivotState();
  }
  public void IntakePivotState()
  {
    switch(currentPivotState)
    {
      case DRIVE: intakePivotPID.setReference(Constants.AlgaeIntakeConstants.DRIVE_HEIGHT, ControlType.kPosition);
      break;

      case PICKUP: intakePivotPID.setReference(Constants.AlgaeIntakeConstants.GROUND, ControlType.kPosition);
      break;
      
      case SHOOT: intakePivotPID.setReference(Constants.AlgaeIntakeConstants.PROCESSOR_HEIGHT, ControlType.kPosition);
      break;
    }
  }
  public void SetIntakeRunState(IntakeRunstate state) {
    currentRunState = state;
    IntakeRunState();
  }
  public void IntakeRunState()
  {
    switch(currentRunState)
    {
      case NONE: intakeRunPID.setReference(0, ControlType.kVelocity);
      break;

      case INTAKE: intakeRunPID.setReference(Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;

      case OUTTAKE: intakeRunPID.setReference(-Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;

    }
  }

  public void AutoFlip()
  {
    if(currentRunState == IntakeRunstate.INTAKE &&(!limitswitch1.get()||!limitswitch2.get())) 
    {
      SetIntakePivotState(IntakePivotState.SHOOT);
      SetIntakeRunState(IntakeRunstate.NONE);
    }
  }

  @Override
  public void periodic() 
  {
    AutoFlip();
    SmartDashboard.putNumber("Algae Pos", intakePivotMotorEncoder.getPosition());
    intakePivotMotorEncoder.setPosition(intakePivotEncoder.getAbsolutePosition().getValueAsDouble()*-360*Constants.ALGAE_INTAKE_PIVOT_ROTATIONS_PER_DEGREE);
    SmartDashboard.putBoolean("Algae LS1", limitswitch1.get());
    SmartDashboard.putBoolean("Algae LS2", limitswitch2.get());
    // SmartDashboard.putNumber("balls", intakeRunEncoder.getVelocity());
  }
}
