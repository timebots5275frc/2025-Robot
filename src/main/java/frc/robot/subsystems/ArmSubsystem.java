// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem.IntakePivotState;
import frc.robot.subsystems.AlgaeIntakeSubsystem.IntakeRunstate;

public class ArmSubsystem extends SubsystemBase {

  DigitalInput limitswitch;
  // DigitalInput limitswitchtwo = new DigitalInput(2);

  armTelescopeState armTelescopeStateCurrent;
  //armPivotState //armPivotStateCurrent;
  armIntakeState armIntakeStateCurrent;

  private SparkMax armTelescopeMotor;
  private RelativeEncoder armTelescopeEncoder;
  private SparkClosedLoopController armTelescopePID;

  private SparkMax armPivotMotor;
  private CANcoder armPivotEncoder;
  private RelativeEncoder armPivotMotorEncoder;
  private SparkClosedLoopController armPivotPID;

  private SparkMax armIntakeMotor;
  private RelativeEncoder armIntakeEncoder;
  private SparkClosedLoopController armIntakePID;
  private armPivotState armPivotStateCurrent;
  private SparkMaxConfig sparkmaxconfig;

  /** Creates a new ArmSubsystem. */
  public enum armTelescopeState
  {
    NONE,
    REMOVE_ALGAE,
    REMOVE_ALGAE_2,
    REMOVE_ALGAE_3,
    // L1,
    L2,
    L3,
    L4,
    DRIVE,
    INTAKE,
    L2BALL,
    L3BALL,
    RESET;
  }

  public enum armPivotState
  {
    NONE,
    L2BALLREMOVAL,
    L3BALLREMOVAL,
    INTAKE_ANGLE,
    L2_ANGLE,
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
    limitswitch = new DigitalInput(ArmConstants.ARM_INTAKE_SWITCH_PORT);

    armTelescopeMotor = new SparkMax(Constants.ArmConstants.ARM_TELESCOPE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    armTelescopeEncoder = armTelescopeMotor.getEncoder();
    armTelescopePID = armTelescopeMotor.getClosedLoopController();
    
    armPivotMotor = new SparkMax(Constants.ArmConstants.ARM_PIVOT_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    
    armPivotEncoder = new CANcoder(ArmConstants.ARM_PIVOT_ENCODER_ID);
    armPivotPID = armPivotMotor.getClosedLoopController();
    armPivotMotorEncoder = armPivotMotor.getEncoder();
    armIntakeMotor = new SparkMax(Constants.ArmConstants.ARM_INTAKE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    armIntakeEncoder = armIntakeMotor.getEncoder();
    armIntakePID = armIntakeMotor.getClosedLoopController();

    //Arm Telesope
    sparkmaxconfig.smartCurrentLimit(20, 20, 2500);
    Constants.ArmConstants.ARM_TELESCOPE_PID.setSparkMaxPID(armTelescopeMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    //Arm Pivot
    sparkmaxconfig.smartCurrentLimit(20, 20, 2500);
    Constants.ArmConstants.ARM_PIVOT_PID.setSparkMaxPID(armPivotMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters,IdleMode.kCoast);
    //Arm Intake
    Constants.ArmConstants.ARM_INTAKE_PID.setSparkMaxPID(armIntakeMotor, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    armTelescopeStateCurrent = armTelescopeState.NONE;
    armPivotStateCurrent = armPivotState.NONE;
    armIntakeStateCurrent = armIntakeState.NONE;
  
  }

  public boolean limitSwitchPressed(){return limitswitch.get();}
  
  public BooleanSupplier limitSwitchIsPressed = new BooleanSupplier(){public boolean getAsBoolean() {return limitSwitchPressed();};};
  
  // public boolean CoralPickedUp()
  // {
  //   if(limitswitch.get() == true){return true;}
  //   else{return false;}
  // }

  public void SetTelescopeState( armTelescopeState state) 
  {
    armTelescopeStateCurrent = state;
    armTelescopeState();
  }

  public void resetTelescopeEncoder() {armTelescopeEncoder.setPosition(0);}

  public void armTelescopeState()
  {
    //System.out.println("Telescope state: "+armTelescopeStateCurrent);
    switch(armTelescopeStateCurrent)
    {
      case NONE: armTelescopePID.setReference(0, ControlType.kCurrent);
      break;
      case REMOVE_ALGAE: armTelescopePID.setReference(ArmConstants.ALGAE_REMOVE, ControlType.kPosition);
      break;
      case REMOVE_ALGAE_2: armTelescopePID.setReference(ArmConstants.ALGAE_REMOVE2, ControlType.kPosition);
      break;
      case REMOVE_ALGAE_3: armTelescopePID.setReference(ArmConstants.ALGAE_REMOVE3, ControlType.kPosition);
      break;
      case L2: armTelescopePID.setReference(Constants.ArmConstants.LEVEL_TWO, ControlType.kPosition);
      break;
      case L3: armTelescopePID.setReference(Constants.ArmConstants.LEVEL_THREE, ControlType.kPosition);
      break;
      case L4: armTelescopePID.setReference(Constants.ArmConstants.LEVEL_FOUR, ControlType.kPosition);
      break;
      case DRIVE: armTelescopePID.setReference(Constants.ArmConstants.DRIVE, ControlType.kPosition);
      break;
      case INTAKE: armTelescopePID.setReference(Constants.ArmConstants.INTAKE, ControlType.kPosition);
      break;
      case L2BALL: armTelescopePID.setReference(Constants.AlgaeIntakeConstants.ALGAE_REEF_HEIGHT_L2, ControlType.kPosition);
      break;
      case L3BALL: armTelescopePID.setReference(Constants.AlgaeIntakeConstants.ALGAE_REEF_HEIGHT_L3, ControlType.kPosition);
      break;
      case RESET:  armTelescopePID.setReference(-1.5, ControlType.kCurrent); resetTelescopeEncoder();   
      break;
      default: armTelescopePID.setReference(0, ControlType.kCurrent); System.out.println("Default");
    }
  }

  public void SetPivotState( armPivotState state) 
  {
    armPivotStateCurrent = state;
    armPivotState();
  }

  public void armPivotState()
  {
    switch(armPivotStateCurrent)
    { 
      case NONE: armPivotPID.setReference(ArmConstants.NORMAL_ANGLE, ControlType.kPosition);
      break;
      case L2BALLREMOVAL: armPivotPID.setReference(ArmConstants.BALL_REMOVAL_SERVICE, ControlType.kPosition);
      break;
      case L3BALLREMOVAL: armPivotPID.setReference(ArmConstants.BALL_REMOVAL_SERVICE2, ControlType.kPosition);
      break;
      case INTAKE_ANGLE: armPivotPID.setReference(Constants.ArmConstants.INTAKE_ANGLE, ControlType.kPosition);
      break;
      case L2_ANGLE: armPivotPID.setReference(Constants.ArmConstants.L2_ANGLE, ControlType.kPosition);
      break;
      case OUTTAKE_ANGLE: armPivotPID.setReference(Constants.ArmConstants.OUTTAKE_ANGLE, ControlType.kPosition);
      break;
    }
  }

  public void SetIntakeState( armIntakeState state) 
  {
    armIntakeStateCurrent = state;
    armIntakeState();
  }

  public void armIntakeState()
  {
    //System.out.println("Intake state" +armIntakeStateCurrent);
    switch(armIntakeStateCurrent)
    {
      case NONE: armIntakePID.setReference(0, ControlType.kVelocity);
      break;
      case INTAKE: armIntakePID.setReference(-Constants.ArmConstants.ARM_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;
      case OUTTAKE: armIntakePID.setReference(Constants.ArmConstants.ARM_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;
    }
  }

  public void AutoFlip()
  {
    if(!limitswitch.get()&&armIntakeStateCurrent == armIntakeState.INTAKE)
    {
      SetIntakeState(armIntakeState.NONE);
    } //else if (limitswitch.get()&&armed==false){armed=true;}
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putNumber("Coral RPM", armIntakeEncoder.getVelocity());
    AutoFlip();
    armPivotMotorEncoder.setPosition(armPivotEncoder.getAbsolutePosition().getValueAsDouble()*360*Constants.INTAKE_PIVOT_ROTATIONS_PER_DEGREE);
    SmartDashboard.putNumber("Relative Encoder", armPivotMotorEncoder.getPosition());
    SmartDashboard.putNumber("Absolute Encoder", armPivotEncoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putBoolean("Arm LS", limitswitch.get());
    // This method will be called once per scheduler run
  }
}
