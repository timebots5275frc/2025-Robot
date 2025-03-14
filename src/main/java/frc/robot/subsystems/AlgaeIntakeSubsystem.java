// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntakeSubsystem extends SubsystemBase 
{
  IntakePivotState currentPivotState;
  IntakeRunstate   currentRunState;

  private DigitalInput limitswitch1,limitswitch2;

  private SparkMax intakeRunMotor;
  private RelativeEncoder intakeRunEncoder;
  private SparkClosedLoopController intakeRunPID;

  private SparkMax intakePivotMotor;
  private RelativeEncoder intakePivotMotorEncoder;
  private CANcoder intakePivotMotorEncoder;
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
    limitswitch1 = new DigitalInput(AlgaeIntakeConstants.ALGAE_INTAKE_SWITCH1_PORT);
    limitswitch2 = new DigitalInput(AlgaeIntakeConstants.ALGAE_INTAKE_SWITCH2_PORT);
    intakePivotMotor = new SparkMax(Constants.AlgaeIntakeConstants.ALGAE_PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    intakePivotMotorEncoder = intakePivotMotor.getEncoder();
    intakePivotEncoder = new CANcoder(AlgaeIntakeConstants.ALGAE_PIVOT_MOTOR_ENCODER_ID); 
    intakePivotPID = intakePivotMotor.getClosedLoopController();


    Constants.AlgaeIntakeConstants.ALGAE_INTAKE_PIVOT_PID.setSparkMaxPID(intakePivotMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeRunMotor = new SparkMax(Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    intakeRunEncoder = intakeRunMotor.getEncoder();
    intakeRunPID = intakeRunMotor.getClosedLoopController();

    Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_PID.setSparkMaxPID(intakeRunMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    currentPivotState = IntakePivotState.DRIVE;
    currentRunState   = IntakeRunstate.NONE;
  }
  public void SetIntakePivotState(IntakePivotState state) {
    currentPivotState = state;
    IntakePivotState();
   
  }
  public void IntakePivotState()
  {
    intakePivotMotorEncoder.setPosition(
        intakePivotEncoder.getAbsolutePosition().getValueAsDouble()*360*Constants.ALGAE_INTAKE_ROTATIONS_PER_DEGREE);
    switch(currentPivotState)
    {
      case DRIVE: intakePivotPID.setReference(Constants.AlgaeIntakeConstants.DRIVE_HEIGHT, ControlType.kPosition);
      break;

      case PICKUP: intakePivotPID.setReference(Constants.AlgaeIntakeConstants.GROUND, ControlType.kPosition);
      break;
      
      case SHOOT: intakePivotPID.setReference(Constants.AlgaeIntakeConstants.PROCESSOR, ControlType.kPosition);
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

      case OUTTAKE: intakeRunPID.setReference(-Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_SPEED * 0.5, ControlType.kVelocity);
      break;

    }
  }

  public void AutoFlip()
  {
    if(limitswitch1.get()||limitswitch2.get())
    {
      SetIntakePivotState(IntakePivotState.DRIVE);
      SetIntakeRunState(IntakeRunstate.NONE);
    }
  }
  @Override public void teleopPeriodic() { 
  AutoFlip();
  // see arm subsystem teleopperiodic
  }
  @Override
  public void periodic() 
  {
    intakePivotMotorEncoder.setPosition(
        intakePivotEncoder.getAbsolutePosition().getValueAsDouble()*360*Constants.ALGAE_INTAKE_ROTATIONS_PER_DEGREE);
     }
}
