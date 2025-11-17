// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.AbsoluteEncoder;
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

  DigitalInput limitswitch1;
  DigitalInput limitswitch2;
  // DigitalInput limitswitchtwo = new DigitalInput(2);

  LaserCANSubsystem lcs = new LaserCANSubsystem();

  ElevatorHeightState elevatorHeight;
  //armPivotState //armPivotStateCurrent;
  ElevatorIntakeState armIntakeStateCurrent;

  private SparkMax elevatorHeightMotor1;
  private SparkMax elevatorHeightMotor2;
  private AbsoluteEncoder elevatorHeightEncoder;
  private SparkClosedLoopController elevatorPID1;
  private SparkClosedLoopController elevatorPID2;

  /** Creates a new ArmSubsystem.  */
  public enum ElevatorHeightState
  {
    /*
     * Things To Think Of
     *    - Algae should have 3 different levels to intake it in
     *    - Coral Should have 4 different levels
     *    - Drive level should just be the same as level 1 for parking at the end
     */
    NONE,
    L1,
    L2,
    L3,
    L4,
    DRIVE,
    ALGAE,
    INTAKE;
    // RESET;
  }

  public enum ElevatorIntakeState
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  public ElevatorSubsystem() 
  {
    limitswitch1 = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH_PORT1);
    limitswitch2 = new DigitalInput(ElevatorConstants.ELEVATOR_LIMIT_SWITCH_PORT2);

    elevatorHeightMotor1 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_HEIGHT_MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig ehm1 = ElevatorConstants.ELEVATOR_HEIGHT_PID.setSparkMaxPID(elevatorHeightMotor1, IdleMode.kBrake);
    elevatorPID1 = elevatorHeightMotor1.getClosedLoopController();

    elevatorHeightMotor2 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_HEIGHT_MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig ehm2 = ElevatorConstants.ELEVATOR_HEIGHT_PID.setSparkMaxPID(elevatorHeightMotor1, IdleMode.kBrake);
    ehm2.follow(elevatorHeightMotor1);
    elevatorHeightMotor2.configure(ehm2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorPID2 = elevatorHeightMotor1.getClosedLoopController();

    elevatorHeight = ElevatorHeightState.NONE;
  
  }

  //limit switch top
  public boolean limitSwitchPressed1(){return limitswitch1.get();}
  public BooleanSupplier limitSwitchIsPressed1 = new BooleanSupplier(){public boolean getAsBoolean() {return limitSwitchPressed1();};};

  //limit switch bottom
  public boolean limitSwitchPressed2(){return limitswitch1.get();}
  public BooleanSupplier limitSwitchIsPressed2 = new BooleanSupplier(){public boolean getAsBoolean() {return limitSwitchPressed2();};};
  
  public boolean CoralPickedUp()
  {
    if(lcs.LC2() == true){return true;}
    else{return false;}
  }

  public void SetHeightState(ElevatorHeightState state) 
  {
    elevatorHeight = state;
    UpdateElevatorHeightState();
  }

  // public void resetTelescopeEncoder() {elevatorHeightEncoder1.setPosition(0);}

  public void UpdateElevatorHeightState()
  {
    
    //System.out.println("Telescope state: "+armTelescopeStateCurrent);
    switch(elevatorHeight)
    {
      /*
       * Things To Think Of
       *    - What conditions should be true vs false in order to switch what level we are in
       *    - When setting something to not move do NOT set the control type as a position as it WILL move to the position that you set it at
       */
      //none
      case NONE: elevatorPID1.setReference(0, ControlType.kCurrent); elevatorPID1.setReference(0, ControlType.kCurrent);
      break;
      //L1
      case L1: if(lcs.LC2() == true && lcs.LC1() == false && limitSwitchPressed2() == false){elevatorPID1.setReference(Constants.ElevatorConstants.LEVEL_ONE, ControlType.kPosition);}
               else{elevatorPID1.setReference(0, ControlType.kCurrent);}
      break;
      //L2
      case L2: if(lcs.LC2() == true && lcs.LC1() == false){elevatorPID1.setReference(Constants.ElevatorConstants.LEVEL_TWO, ControlType.kPosition);}
               else{elevatorPID1.setReference(0, ControlType.kCurrent);}
      break;
      //L3
      case L3: if(lcs.LC2() == true && lcs.LC1() == false){elevatorPID1.setReference(Constants.ElevatorConstants.LEVEL_THREE, ControlType.kPosition);}
               else{elevatorPID1.setReference(0, ControlType.kCurrent);}
      break;
      //L4
      case L4: if(lcs.LC2() == true && lcs.LC1() == false && limitSwitchPressed1() == false){elevatorPID1.setReference(Constants.ElevatorConstants.LEVEL_FOUR, ControlType.kPosition);}
               else{elevatorPID1.setReference(0, ControlType.kCurrent);}
      break;
      //drive
      case DRIVE: if(lcs.LC2() == true && lcs.LC1() == false){elevatorPID1.setReference(Constants.ElevatorConstants.DRIVE, ControlType.kPosition);}
                  else{elevatorPID1.setReference(0, ControlType.kCurrent);}
      break;
      //intake
      case INTAKE: if(lcs.LC2() == true && lcs.LC1() == false){elevatorPID1.setReference(Constants.ElevatorConstants.INTAKE, ControlType.kPosition);}
                   else{elevatorPID1.setReference(0, ControlType.kCurrent);}
      break;
      //algae
      case ALGAE: if(lcs.LC2() == true && lcs.LC1() == false){elevatorPID1.setReference(Constants.ElevatorConstants.ALGAE, ControlType.kPosition);}
                  else{elevatorPID1.setReference(0, ControlType.kCurrent);}
      break;
      // case RESET:  elevatorHeightEncoder2.setReference(-1.5, ControlType.kCurrent); resetTelescopeEncoder();   
      // break;
      default: elevatorPID1.setReference(0, ControlType.kCurrent); System.out.println("Default");
    }
  }

  @Override
  public void periodic() 
  {
    SmartDashboard.putBoolean("CPU", CoralPickedUp());
  }
}
