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
    L1,
    L2,
    L3,
    L4,
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

    //Arm Telesope
    Constants.ArmConstants.ARM_TELESCOPE_PID.setSparkMaxPID(armTelescopeMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Arm Pivot
    Constants.ArmConstants.ARM_PIVOT_PID.setSparkMaxPID(armPivotMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //Arm Intake
    Constants.ArmConstants.ARM_INTAKE_PID.setSparkMaxPID(armIntakeMotor, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armTelescopeStateCurrent = armTelescopeState.NONE;
    armPivotStateCurrent = armPivotState.NONE;
    armIntakeStateCurrent = armIntakeState.NONE;
  
  }
  public void SetTelescopeState( armTelescopeState state) {
    armTelescopeStateCurrent = state;
    armTelescopeState();
  }
  public void armTelescopeState()
  {
    switch(armTelescopeStateCurrent)
    {
      case NONE: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
      case L1: armTelescopePID.setReference(Constants.ArmConstants.LEVEL_ONE, ControlType.kPosition);
      break;
      case L2: armTelescopePID.setReference(-Constants.ArmConstants.LEVEL_TWO, ControlType.kPosition);
      break;
      case L3: armTelescopePID.setReference(Constants.ArmConstants.LEVEL_THREE, ControlType.kPosition);
      break;
      case L4: armTelescopePID.setReference(-Constants.ArmConstants.LEVEL_FOUR, ControlType.kPosition);
      break;
      case RESET:  armTelescopePID.setReference(-6, ControlType.kCurrent);
                   armTelescopeEncoder.setPosition(0);
      break;
    }
  }
  public void SetPivotState( armPivotState state) {
    armPivotStateCurrent = state;
    armPivotState();
  }
  public void armPivotState()
  {
    switch(armPivotStateCurrent)
    { 
      case NONE: armPivotPID.setReference(ArmConstants.NORMAL_ANGLE, ControlType.kPosition);
      break;
      case INTAKE_ANGLE: armPivotPID.setReference(Constants.ArmConstants.INTAKE_ANGLE, ControlType.kPosition);
      break;
      case OUTTAKE_ANGLE: armPivotPID.setReference(Constants.ArmConstants.OUTTAKE_ANGLE, ControlType.kPosition);
      break;
    }
  }
  public void SetIntakeState( armIntakeState state) {
    armIntakeStateCurrent = state;
    armIntakeState();
  }
  public void armIntakeState()
  {
    switch(armIntakeStateCurrent)
    {
      case NONE: armTelescopePID.setReference(0, ControlType.kVelocity);
      break;
      case INTAKE: armTelescopePID.setReference(Constants.ArmConstants.ARM_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;
      case OUTTAKE: armTelescopePID.setReference(-Constants.ArmConstants.ARM_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;
    }
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
