// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
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
  AlgaeIntakePivotState currentPivotState;
  AlgaeIntakeRunState   currentRunState;

  private SparkMax algaeIntakeRunMotor1;
  private RelativeEncoder algaeIntakeRunEncoder1;
  private SparkClosedLoopController algaeIntakeRunPID1;

  private CANcoder algaeIntakeRunEncoder2;
  private SparkMax algaeIntakeRunMotor2;
  // private RelativeEncoder intakePivotMotorEncoder;
  private SparkClosedLoopController algaeIntakeRunPID2;

  private SparkMax algaeIntakePivotmotor;
  private RelativeEncoder algaeIntakePivotEncoder;
  private SparkClosedLoopController algaeIntakePivotPID;

  private SparkMaxConfig sparkmaxconfig;

  public enum AlgaeIntakeRunState
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  public enum AlgaeIntakePivotState
  {
    DRIVE,
    REEF,
    GROUND;
  }

  public AlgaeIntakeSubsystem() 
  {
    //limitswitch1 = new DigitalInput(AlgaeIntakeConstants.ALGAE_INTAKE_SWITCH1_PORT);
    //limitswitch2 = new DigitalInput(AlgaeIntakeConstants.ALGAE_INTAKE_SWITCH2_PORT);

    sparkmaxconfig.smartCurrentLimit(20, 20, 2500);
    Constants.AlgaeIntakeConstants.ALGAE_INTAKE_PIVOT_PID.setSparkMaxPID(algaeIntakeRunMotor2, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters, IdleMode.kCoast);

    algaeIntakeRunMotor1 = new SparkMax(Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    algaeIntakeRunEncoder1 = algaeIntakeRunMotor1.getEncoder();
    algaeIntakeRunPID1 = algaeIntakeRunMotor1.getClosedLoopController();
    sparkmaxconfig.idleMode(IdleMode.kBrake);

    Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_PID.setSparkMaxPID(algaeIntakeRunMotor1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    currentPivotState = AlgaeIntakePivotState.DRIVE;
    currentRunState   = AlgaeIntakeRunState.NONE;
  }

  public void SetAlgaeIntakeRunState(AlgaeIntakeRunState state) 
  {
    currentRunState = state;
    IntakeRunState();
  }

  public void SetAlgaeIntakePivotState(AlgaeIntakePivotState aips) 
  {
    currentPivotState = aips;
    AlgaeIntakePivotState();
  }

  public void AlgaeIntakePivotState()
  {
    switch (currentPivotState) 
    {
      case DRIVE: algaeIntakePivotPID.setReference(0, ControlType.kPosition);
      break;
      case REEF: algaeIntakePivotPID.setReference(0, ControlType.kPosition);
      break;
      case GROUND: algaeIntakePivotPID.setReference(0, ControlType.kPosition);
      break;
      default: algaeIntakePivotPID.setReference(0,ControlType.kPosition); //same as drive
      break;
    }
  }

  public void IntakeRunState()
  {
    switch(currentRunState)
    {
      case NONE: algaeIntakeRunPID1.setReference(0, ControlType.kVelocity);
      break;

      case INTAKE: algaeIntakeRunPID1.setReference(Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;

      case OUTTAKE: algaeIntakeRunPID1.setReference(-Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;

    }
  }


  @Override
  public void periodic() 
  {
    algaeIntakePivotEncoder.getPosition();
    SmartDashboard.putNumber("APP", algaeIntakePivotEncoder.getPosition());
    // SmartDashboard.putNumber("Algae Pos", intakeRunMotorEncoder2.getPosition());
    // intakePivotMotorEncoder.setPosition(intakeRunEncoder2.getAbsolutePosition().getValueAsDouble()*-360*Constants.ALGAE_INTAKE_PIVOT_ROTATIONS_PER_DEGREE);
    // SmartDashboard.putNumber("balls", intakeRunEncoder.getVelocity());
  }
}
