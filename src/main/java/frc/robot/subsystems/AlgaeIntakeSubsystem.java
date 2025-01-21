// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class AlgaeIntakeSubsystem extends SubsystemBase 
{
  IntakePivotState currentPivotState;

  private SparkMax intakeRunMotor;
  private RelativeEncoder intakeRunEncoder;
  private SparkClosedLoopController intakeRunPID;

  private SparkMax intakePivotMotor;
  private RelativeEncoder intakePivotEncoder;
  private SparkClosedLoopController intakePivotPID;

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
    intakePivotMotor = new SparkMax(Constants.IntakeConstants.PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    intakePivotEncoder = intakePivotMotor.getEncoder();
    intakePivotPID = intakePivotMotor.getClosedLoopController();
    ClosedLoopConfig intakePivotCLC;
    SparkMaxConfig intakePivotSMC;
    intakePivotCLC = new ClosedLoopConfig();
    intakePivotCLC.pidf(IntakeConstants.IntakePivotPIDs.P, IntakeConstants.IntakePivotPIDs.I, IntakeConstants.IntakePivotPIDs.D, IntakeConstants.IntakePivotPIDs.kFF);
    intakePivotCLC.iZone(IntakeConstants.IntakePivotPIDs.IZ);
    intakePivotSMC = new SparkMaxConfig();
    intakePivotSMC.apply(intakePivotCLC);
   
    intakePivotMotor.configure(intakePivotSMC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    // intakePivotPID.setOutputRange(-1, 1, 0);
    // intakePivotPID.setSmartMotionMaxVelocity(Constants.IntakeConstants.INTAKE_PIVOT_MAX_VELOCITY, 0);
    // intakePivotPID.setSmartMotionMaxAccel(Constants.IntakeConstants.INTAKE_PIVOT_MAX_ACCELERATION, 0);
    // intakePivotPID.setSmartMotionMinOutputVelocity(Constants.IntakeConstants.INTAKE_PIVOT_MIN_VELOCITY, 0);

    intakeRunMotor = new SparkMax(Constants.IntakeConstants.INTAKE_RUN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    intakeRunEncoder = intakeRunMotor.getEncoder();
    intakeRunPID = intakeRunMotor.getClosedLoopController();
    ClosedLoopConfig intakeRunCLC = new ClosedLoopConfig();
    SparkMaxConfig intakeRunSMC = new SparkMaxConfig();
    intakeRunCLC.pidf(Constants.IntakeConstants.IntakeRunPIDs.P,
    Constants.IntakeConstants.IntakeRunPIDs.I,
    Constants.IntakeConstants.IntakeRunPIDs.D,
    Constants.IntakeConstants.IntakeRunPIDs.kFF);
    intakeRunCLC.iZone(IntakeConstants.IntakeRunPIDs.IZ);
    intakeRunSMC.apply(intakeRunCLC);
    intakeRunMotor.configure(intakeRunSMC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    currentPivotState = IntakePivotState.DRIVE;
  }

  public void IntakePivotState(IntakePivotState state)
  {
    switch(state)
    {
      case DRIVE: intakePivotPID.setReference(0, ControlType.kVelocity);
      break;
      case PICKUP: intakePivotPID.setReference(0, ControlType.kVelocity);
      break;
      case SHOOT: intakePivotPID.setReference(0, ControlType.kVelocity);
      break;
    }
  }

  public void IntakeRunState(IntakeRunstate state)
  {
    switch(state)
    {
      case NONE: intakeRunPID.setReference(0, ControlType.kVelocity);
      break;

      case INTAKE: intakeRunPID.setReference(Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;

      case OUTTAKE: intakeRunPID.setReference(-Constants.IntakeConstants.INTAKE_RUN_SPEED * 0.5, ControlType.kVelocity);
      break;

    }
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
