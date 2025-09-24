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

  ElevatorHeightState elevatorHeight;
  //armPivotState //armPivotStateCurrent;
  ElevatorIntakeState armIntakeStateCurrent;

  private SparkMax elevatorHeightMotor1;
  private SparkMax elevatorHeightMotor2;
  private RelativeEncoder elevatorHeightEncoder1;
  private RelativeEncoder armTelescopeEncoderTwo;
  private SparkClosedLoopController elevatorHeightEncoder2;
  private SparkClosedLoopController armTelescopePIDTwo;

  /** Creates a new ArmSubsystem. */
  public enum ElevatorHeightState
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

  public enum ElevatorIntakeState
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  public ElevatorSubsystem() 
  {
    limitswitch = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH_PORT);

    elevatorHeightMotor1 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_HEIGHT_MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);
    elevatorHeightMotor2 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_HEIGHT_MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);
    elevatorHeightEncoder1 = elevatorHeightMotor1.getEncoder();
    elevatorHeightEncoder2 = elevatorHeightMotor1.getClosedLoopController();

    //Arm Telesope
    // sparkmaxconfig.smartCurrentLimit(20, 20, 2500);
    Constants.ElevatorConstants.ELEVATOR_HEIGHT_PID.setSparkMaxPID(elevatorHeightMotor1, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    elevatorHeight = ElevatorHeightState.NONE;
  
  }
  public boolean limitSwitchPressed(){return limitswitch.get();}
  
  public BooleanSupplier limitSwitchIsPressed = new BooleanSupplier(){public boolean getAsBoolean() {return limitSwitchPressed();};};
  
  // public boolean CoralPickedUp()
  // {
  //   if(limitswitch.get() == true){return true;}
  //   else{return false;}
  // }

  public void SetHeightState(ElevatorHeightState state) 
  {
    elevatorHeight = state;
    armTelescopeState();
  }


  public void resetTelescopeEncoder() {elevatorHeightEncoder1.setPosition(0);}


  public void armTelescopeState()
  {
    
    //System.out.println("Telescope state: "+armTelescopeStateCurrent);
    switch(elevatorHeight)
    {
      case NONE: elevatorHeightEncoder2.setReference(0, ControlType.kCurrent); armTelescopePIDTwo.setReference(0, ControlType.kCurrent);
      break;
      case L2: elevatorHeightEncoder2.setReference(Constants.ElevatorConstants.LEVEL_TWO, ControlType.kPosition);
      break;
      case L3: elevatorHeightEncoder2.setReference(Constants.ElevatorConstants.LEVEL_THREE, ControlType.kPosition);
      break;
      case L4: elevatorHeightEncoder2.setReference(Constants.ElevatorConstants.LEVEL_FOUR, ControlType.kPosition);
      break;
      case DRIVE: elevatorHeightEncoder2.setReference(Constants.ElevatorConstants.DRIVE, ControlType.kPosition);
      break;
      case INTAKE: elevatorHeightEncoder2.setReference(Constants.ElevatorConstants.INTAKE, ControlType.kPosition);
      break;
      case RESET:  elevatorHeightEncoder2.setReference(-1.5, ControlType.kCurrent); resetTelescopeEncoder();   
      break;
      default: elevatorHeightEncoder2.setReference(0, ControlType.kCurrent); System.out.println("Default");
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
