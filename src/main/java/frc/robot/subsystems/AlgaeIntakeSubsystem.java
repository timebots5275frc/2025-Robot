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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
//import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
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

  private SparkMax algaeIntakeRunMotor;
  private RelativeEncoder algaeIntakeRunEncoder;
  private SparkClosedLoopController algaeIntakeRunPID;

  // private CANcoder algaeIntakeRunEncoder2;
  // private SparkMax algaeIntakeRunMotor2;
  // // private RelativeEncoder intakePivotMotorEncoder;
  // private SparkClosedLoopController algaeIntakeRunPID2;

  private SparkMax algaePivotMotor;
  private RelativeEncoder algaePivotEncoder;
  private SparkClosedLoopController algaePivotPID;

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
    SHOOT,
    REEF,
    GROUND;
  }

  public AlgaeIntakeSubsystem() 
  {
    //limitswitch1 = new DigitalInput(AlgaeIntakeConstants.ALGAE_INTAKE_SWITCH1_PORT);
    //limitswitch2 = new DigitalInput(AlgaeIntakeConstants.ALGAE_INTAKE_SWITCH2_PORT);

    // sparkmaxconfig.smartCurrentLimit(20, 20, 2500);

    algaeIntakeRunMotor = new SparkMax(Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_PID.setSparkMaxPID(algaeIntakeRunMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    algaeIntakeRunPID = algaeIntakeRunMotor.getClosedLoopController();
    // sparkmaxconfig.idleMode(IdleMode.kCoast);
    Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_PID.setSparkMaxPID(algaeIntakeRunMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters, IdleMode.kCoast);

    algaePivotMotor = new SparkMax(Constants.AlgaeIntakeConstants.ALGAE_PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    algaePivotEncoder = algaePivotMotor.getEncoder();
    algaePivotPID = algaePivotMotor.getClosedLoopController();
   // sparkmaxconfig.idleMode(IdleMode.kCoast);
   Constants.AlgaeIntakeConstants.ALGAE_INTAKE_PIVOT_PID.setSparkMaxPID(algaePivotMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters, IdleMode.kCoast);

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
      /*
       * Things To Look Out For
       *    - Motor trying to go too far in one direction/Stalling
       *    - Button should be held until algae is intaked
       *    - What is the zero position
       *    - What direction is positive vs negative
       *    - Would it be more efficient to keep algae intake out while driving or should we keep it in to be safe
       */
      case DRIVE: algaePivotPID.setReference(Constants.AlgaeIntakeConstants.DRIVE_ANGLE, ControlType.kPosition);
      break;
      case SHOOT: algaePivotPID.setReference(Constants.AlgaeIntakeConstants.SHOOT_ANGLE, ControlType.kPosition);
      break;
      case REEF: algaePivotPID.setReference(Constants.AlgaeIntakeConstants.ALGAE_REEF_ANGLE, ControlType.kPosition);
      break;
      case GROUND: algaePivotPID.setReference(Constants.AlgaeIntakeConstants.ALGAE_GROUND_ANGLE, ControlType.kPosition);
      break;
      default: algaePivotPID.setReference(0,ControlType.kPosition); //same as drive
      break;
    }
  }

  public void IntakeRunState()
  {
    switch(currentRunState)
    {
      case NONE: algaeIntakeRunPID.setReference(0, ControlType.kCurrent);
      break;

      case INTAKE: algaeIntakeRunPID.setReference(Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;

      case OUTTAKE: algaeIntakeRunPID.setReference(-Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;

    }
  }


  @Override
  public void periodic() 
  {
    algaePivotEncoder.getPosition();
    SmartDashboard.putNumber("APP", algaePivotEncoder.getPosition());
    SmartDashboard.putNumber("AIRS", Constants.AlgaeIntakeConstants.ALGAE_INTAKE_RUN_SPEED);
    // SmartDashboard.putNumber("Algae Pos", intakeRunMotorEncoder2.getPosition());
    // intakePivotMotorEncoder.setPosition(intakeRunEncoder2.getAbsolutePosition().getValueAsDouble()*-360*Constants.ALGAE_INTAKE_PIVOT_ROTATIONS_PER_DEGREE);
    // SmartDashboard.putNumber("balls", intakeRunEncoder.getVelocity());
  }
}
