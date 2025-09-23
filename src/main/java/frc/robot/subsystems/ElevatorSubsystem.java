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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {

  DigitalInput limitswitch;
  // DigitalInput limitswitchtwo = new DigitalInput(2);

  armTelescopeState armTelescopeStateCurrent;
  //armPivotState //armPivotStateCurrent;
  armIntakeState armIntakeStateCurrent;

  private SparkMax armTelescopeMotorOne;
  private SparkMax armTelescopeMotorTwo;
  private RelativeEncoder armTelescopeEncoderOne;
  private RelativeEncoder armTelescopeEncoderTwo;
  private SparkClosedLoopController armTelescopePIDOne;
  private SparkClosedLoopController armTelescopePIDTwo;

  /** Creates a new ArmSubsystem. */
  public enum armTelescopeState
  {
    NONE,
    // L1,
    L2,
    L3,
    L4,
    DRIVE,
    INTAKE,
    RESET;
  }

  public enum armIntakeState
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  public ElevatorSubsystem() 
  {
    limitswitch = new DigitalInput(ElevatorConstants.ARM_INTAKE_SWITCH_PORT);

    armTelescopeMotorOne = new SparkMax(Constants.ElevatorConstants.ARM_TELESCOPE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    armTelescopeMotorTwo = new SparkMax(Constants.ElevatorConstants.ARM_TELESCOPE_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    armTelescopeEncoderOne = armTelescopeMotorOne.getEncoder();
    armTelescopePIDOne = armTelescopeMotorOne.getClosedLoopController();

    //Arm Telesope
    // sparkmaxconfig.smartCurrentLimit(20, 20, 2500);
    Constants.ElevatorConstants.ARM_TELESCOPE_PID.setSparkMaxPID(armTelescopeMotorOne, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    armTelescopeStateCurrent = armTelescopeState.NONE;
  
  }
  public boolean limitSwitchPressed(){return limitswitch.get();}
  
  public BooleanSupplier limitSwitchIsPressed = new BooleanSupplier(){public boolean getAsBoolean() {return limitSwitchPressed();};};
  
  // public boolean CoralPickedUp()
  // {
  //   if(limitswitch.get() == true){return true;}
  //   else{return false;}
  // }

  public void SetTelescopeState(armTelescopeState state) 
  {
    armTelescopeStateCurrent = state;
    armTelescopeState();
  }


  public void resetTelescopeEncoder() {armTelescopeEncoderOne.setPosition(0);}


  public void armTelescopeState()
  {
    
    //System.out.println("Telescope state: "+armTelescopeStateCurrent);
    switch(armTelescopeStateCurrent)
    {
      case NONE: armTelescopePIDOne.setReference(0, ControlType.kCurrent); armTelescopePIDTwo.setReference(0, ControlType.kCurrent);
      break;
      case L2: armTelescopePIDOne.setReference(Constants.ElevatorConstants.LEVEL_TWO, ControlType.kPosition);
      break;
      case L3: armTelescopePIDOne.setReference(Constants.ElevatorConstants.LEVEL_THREE, ControlType.kPosition);
      break;
      case L4: armTelescopePIDOne.setReference(Constants.ElevatorConstants.LEVEL_FOUR, ControlType.kPosition);
      break;
      case DRIVE: armTelescopePIDOne.setReference(Constants.ElevatorConstants.DRIVE, ControlType.kPosition);
      break;
      case INTAKE: armTelescopePIDOne.setReference(Constants.ElevatorConstants.INTAKE, ControlType.kPosition);
      break;
      case RESET:  armTelescopePIDOne.setReference(-1.5, ControlType.kCurrent); resetTelescopeEncoder();   
      break;
      default: armTelescopePIDOne.setReference(0, ControlType.kCurrent); System.out.println("Default");
    }
  }

  @Override
  public void periodic() 
  {
    // SmartDashboard.putNumber("Tele RPM", armTelescopeEncoder.getVelocity());
    // SmartDashboard.putNumber("Tele Current", armTelescopeMotor.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}
