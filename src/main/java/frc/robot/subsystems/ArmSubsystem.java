// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

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

    ClosedLoopConfig teleCLC = new ClosedLoopConfig();
    ClosedLoopConfig armPivCLC = new ClosedLoopConfig();
    ClosedLoopConfig armIntCLC = new ClosedLoopConfig();
    SparkMaxConfig teleSMC = new SparkMaxConfig();
    SparkMaxConfig armPivSMC = new SparkMaxConfig();
    SparkMaxConfig armIntSMC = new SparkMaxConfig();
    teleCLC.pidf(ArmConstants.ArmTelescopePIDs.P, ArmConstants.ArmTelescopePIDs.I, ArmConstants.ArmTelescopePIDs.D,ArmConstants.ArmTelescopePIDs.kFF);
    armPivCLC.pidf(ArmConstants.ArmPivotPIDs.P, ArmConstants.ArmPivotPIDs.I, ArmConstants.ArmPivotPIDs.D,ArmConstants.ArmPivotPIDs.kFF);
    armIntCLC.pidf(ArmConstants.ArmIntakePIDs.P, ArmConstants.ArmIntakePIDs.I, ArmConstants.ArmIntakePIDs.D,ArmConstants.ArmIntakePIDs.kFF);
    teleCLC.iZone(ArmConstants.ArmTelescopePIDs.IZ);
    armPivCLC.iZone(ArmConstants.ArmPivotPIDs.IZ);
    armIntCLC.iZone(ArmConstants.ArmIntakePIDs.IZ);
    teleSMC.apply(teleCLC);
    armPivSMC.apply(armPivCLC);
    armIntSMC.apply(armIntCLC);
    armTelescopeMotor.configure(teleSMC,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    armPivotMotor.configure(armPivSMC,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    armIntakeMotor.configure(armIntSMC,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
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
      case EXTEND: armTelescopePID.setReference(Constants.ArmConstants.ARM_SPEED, ControlType.kVelocity);
      break;
      case RETRACT: armTelescopePID.setReference(-Constants.ArmConstants.ARM_SPEED, ControlType.kVelocity);
      break;
      case RESET:  armTelescopePID.setReference(-1, ControlType.kVelocity);
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
