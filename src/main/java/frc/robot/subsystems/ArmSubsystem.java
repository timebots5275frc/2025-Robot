// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {

  armTelescopeState armTelescopeStateCurrent;
  armPivotState armPivotStateCurrent;
  armIntakeState armIntakeStateCurrent;

  private SparkMax armTelescopeMotor;
  private RelativeEncoder armTelescopeEncoder;
  private SparkClosedLoopController armTelescopePID;

  private SparkMax armPivotMotor;
  private RelativeEncoder armPivotEncoder;
  private SparkClosedLoopController armPivotPID;

  private SparkMax armIntakeMotor;
  private RelativeEncoder armIntakeEncoder;
  private SparkClosedLoopController armIntakePID;

  /** Creates a new ArmSubsystem. */
  public enum armTelescopeState
  {
    NONE,
    EXTEND,
    RETRACT,
    RESET;
  }

  public enum armPivotState
  {
    NONE,
    INTAKE_ANGLE,
    OUTTAKE_ANGLE;
  }

  public enum armIntakeState
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  public ArmSubsystem() 
  {
    
    armTelescopeMotor = new SparkMax(Constants.ArmConstants.ARM_PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    armTelescopeEncoder = armTelescopeMotor.getEncoder();
    armTelescopePID = armTelescopeMotor.getClosedLoopController();

    armPivotMotor = new SparkMax(Constants.ArmConstants.ARM_PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    armPivotEncoder = armPivotMotor.getEncoder();
    armPivotPID = armPivotMotor.getClosedLoopController();

    armIntakeMotor = new SparkMax(Constants.ArmConstants.ARM_INTAKE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    armIntakeEncoder = armIntakeMotor.getEncoder();
    armIntakePID = armIntakeMotor.getClosedLoopController();

    armTelescopeMotor.set(Constants.ArmConstants.ArmTelescopePIDs.P);
    armTelescopeMotor.set(Constants.ArmConstants.ArmTelescopePIDs.I);
    armTelescopeMotor.set(Constants.ArmConstants.ArmTelescopePIDs.D);
    armTelescopeMotor.set(Constants.ArmConstants.ArmTelescopePIDs.kFF);

    armPivotMotor.set(Constants.ArmConstants.ArmPivotPIDs.P);
    armPivotMotor.set(Constants.ArmConstants.ArmPivotPIDs.I);
    armPivotMotor.set(Constants.ArmConstants.ArmPivotPIDs.D);
    armPivotMotor.set(Constants.ArmConstants.ArmPivotPIDs.kFF);

    armIntakeMotor.set(Constants.ArmConstants.ArmIntakePIDs.P);
    armIntakeMotor.set(Constants.ArmConstants.ArmIntakePIDs.I);
    armIntakeMotor.set(Constants.ArmConstants.ArmIntakePIDs.D);
    armIntakeMotor.set(Constants.ArmConstants.ArmIntakePIDs.kFF);

    armTelescopeStateCurrent = armTelescopeState.NONE;
    armPivotStateCurrent = armPivotState.NONE;
    armIntakeStateCurrent = armIntakeState.NONE;
  
  }

  public void armTelescopeState(armTelescopeState armTelescopestateCurrent)
  {
    switch(armTelescopestateCurrent)
    {
      case NONE: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
      case EXTEND: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
      case RETRACT: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
      case RESET: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
    }
  }

  public void armPivotState(armPivotState armPivotStateCurrent)
  {
    switch(armPivotStateCurrent)
    {
      case NONE: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
      case INTAKE_ANGLE: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
      case OUTTAKE_ANGLE: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
    }
  }

  public void armIntakeState(armIntakeState armIntakeStateCurrent)
  {
    switch(armIntakeStateCurrent)
    {
      case NONE: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
      case INTAKE: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
      case OUTTAKE: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
    }
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
