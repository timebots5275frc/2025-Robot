// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CoralIntakeConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorConstants;

public class CoralIntakeSubsystem extends SubsystemBase {

  LaserCANSubsystem lcs; //= new LaserCANSubsystem();
  //2 times instantiated ^

  CoralIntakeState cisc;
  ElevatorSubsystem es;

  private SparkMax IntakeMotorOne;
  private SparkClosedLoopController IntakeEncoderOne;
  private SparkClosedLoopController IntakePIDOne;
  private SparkMax IntakeMotorTwo;
  private SparkClosedLoopController IntakeEncoderTwo;
  private SparkClosedLoopController IntakePIDTwo;

  public static boolean coralOutOfWay = false;
  
    //enum coral intake state
    public enum CoralIntakeState
    {
      NONE,
      INTAKE,
      OUTTAKE_L1,
      OUTTAKE_L2_TO_3,
      OUTTAKE_L4;
    }
  
    /** Creates a new CoralIntakeSubsystem. */
    public CoralIntakeSubsystem(LaserCANSubsystem lcs, ElevatorSubsystem es) 
    {
      this.lcs = lcs;
      IntakeMotorOne = new SparkMax(Constants.CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID1, SparkLowLevel.MotorType.kBrushless);
      Constants.CoralIntakeConstants.CORAL_INTAKE_PID.setSparkMaxPID(IntakeMotorOne, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      IntakePIDOne = IntakeMotorOne.getClosedLoopController();
  
      IntakeMotorTwo = new SparkMax(Constants.CoralIntakeConstants.CORAL_INTAKE_MOTOR_ID2, SparkLowLevel.MotorType.kBrushless);
      Constants.CoralIntakeConstants.CORAL_INTAKE_PID.setSparkMaxPID(IntakeMotorTwo, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      // IntakeEncoderTwo = IntakeMotorTwo.getEncoder();
      IntakePIDTwo = IntakeMotorTwo.getClosedLoopController();
    }
  
    public void SetCoralIntakeState(CoralIntakeState state)
    {
      cisc = state;
      CoralIntakeState();
    }
  
    //coral is/isnt out of way
    // public BooleanSupplier CoralNotOutOfWay = new BooleanSupplier() {
    //   public boolean getAsBoolean() {return !CoralOutOfWay();}
    // };
  
    public boolean CoralOutOfWay(){
      if(lcs.LC2() == false && lcs.LC1() == true){return true;}
      else{return false;}
    }
  
    public BooleanSupplier CoralOutOfWay = new BooleanSupplier() {
        public boolean getAsBoolean() {return CoralOutOfWay();}
      };
    
      //coral intake state
      private void CoralIntakeState()
      {
        
        switch(cisc)
        {
          case NONE: IntakePIDOne.setReference(0,ControlType.kCurrent); 
                     IntakePIDTwo.setReference(0, ControlType.kCurrent);
          break;
          case INTAKE: IntakePIDOne.setReference(CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED_NORMAL, ControlType.kVelocity);
                       IntakePIDTwo.setReference(-CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED_NORMAL, ControlType.kVelocity);
          break;
          case OUTTAKE_L1: IntakePIDOne.setReference(CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED_NORMAL, ControlType.kVelocity);
                           IntakePIDTwo.setReference(-CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED_L1, ControlType.kVelocity);
    
          break;
          case OUTTAKE_L2_TO_3:   IntakePIDOne .setReference(CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED_NORMAL,ControlType.kVelocity);
                                  IntakePIDTwo.setReference(-CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED_NORMAL, ControlType.kVelocity);
          break;
          case OUTTAKE_L4: IntakePIDOne .setReference(CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED_NORMAL-1000,ControlType.kVelocity);
                           IntakePIDTwo.setReference(-CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED_NORMAL-1000, ControlType.kVelocity);
          break;
        }
        
      }
    
  //periodic
  @Override
  public void periodic() {

    CoralIntakeSubsystem.coralOutOfWay = CoralOutOfWay();

    SmartDashboard.putNumber("CIRS", CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED_NORMAL);
    // SmartDashboard.putBoolean("coow", CoralOutOfWay());
    // CoralOutOfWay();
    // System.out.println(CoralOutOfWay());
    
    // System.out.println(coralOutOfWay);
  }
}
