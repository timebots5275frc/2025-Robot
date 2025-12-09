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

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.CustomTypes.PID;

public class ElevatorSubsystem extends SubsystemBase {

  LaserCANSubsystem lcs = new LaserCANSubsystem();

  ElevatorHeightState elevatorHeight;
  //armPivotState //armPivotStateCurrent;
  ElevatorIntakeState armIntakeStateCurrent;

  private SparkMax elevatorHeightMotor1;
  private SparkMax elevatorHeightMotor2;
  private AbsoluteEncoder elevatorHeightEncoder;
  private SparkClosedLoopController elevatorPID1;
  private SlewRateLimiter srl = new SlewRateLimiter(18);
  private double targetpose = 0.0;
  // private double targetPosReached;

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
    //slew rate limiter
    //  - a slew rate limiter is used to prevent sudden jerks of movement to protect the elevator from damage
    // SlewRateLimiter srl = new SlewRateLimiter(.5);

    elevatorHeightMotor1 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_HEIGHT_MOTOR1_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig ehm1 = ElevatorConstants.ELEVATOR_HEIGHT_PID.setSparkMaxPID(elevatorHeightMotor1, IdleMode.kBrake);
    elevatorHeightMotor1.configure(ehm1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorPID1 = elevatorHeightMotor1.getClosedLoopController();
    
    elevatorHeightMotor2 = new SparkMax(Constants.ElevatorConstants.ELEVATOR_HEIGHT_MOTOR2_ID, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig ehm2 = ElevatorConstants.ELEVATOR_HEIGHT_PID.setSparkMaxPID(elevatorHeightMotor1, IdleMode.kBrake);
    ehm2.follow(elevatorHeightMotor1, true);
    elevatorHeightMotor2.configure(ehm2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    elevatorHeight = ElevatorHeightState.NONE;
  
  }

  public void ElevatorHeightReset(){
    
  }

  public void SetHeightState(ElevatorHeightState state) 
  {
    elevatorHeight = state;
    UpdateElevatorHeightState();
  }

  public void HeightCurrent(double hc){SetHeightState(elevatorHeight);}

  public void SetTargetPose(double target){targetpose = target;}

  public void UpdateElevatorHeightState()
  {
    
    // if(CoralIntakeSubsystem.coralOutOfWay == true){
      switch(elevatorHeight)
      {
        case NONE: elevatorPID1.setReference(0, ControlType.kCurrent);
        break;
        //L1
        case L1:  SetTargetPose(-Constants.ElevatorConstants.INTAKE);
        break;
        //L2
        case L2: SetTargetPose(-Constants.ElevatorConstants.LEVEL_TWO);
        break;
        //L3
        case L3: SetTargetPose(-Constants.ElevatorConstants.LEVEL_THREE);
        break;
        //L4
        case L4: SetTargetPose(-Constants.ElevatorConstants.LEVEL_FOUR);
        break;
        //drive
        case DRIVE: SetTargetPose(-Constants.ElevatorConstants.DRIVE);
        break;
        //intake
        case INTAKE: SetTargetPose(-Constants.ElevatorConstants.INTAKE);
        break;
        //algae
        case ALGAE: SetTargetPose(-Constants.ElevatorConstants.ALGAE);
        break;
        case RESET: //reset stuff here i spose
        break;
        default:
      }
    }
  // else{System.out.println("coral is in the way");};
  // }

  @Override
    public void periodic() {
      // Read the current encoder position
      double currentPosition = elevatorHeightMotor1.getEncoder().getPosition();
      SmartDashboard.putNumber("ElevatorCurrentPosition", currentPosition);
      SmartDashboard.putNumber("ElevatorTargetPosition", targetpose);

      // Smooth the target position with SlewRateLimiter
      double smoothedTarget = srl.calculate(targetpose);

      // If NONE state, hold current position using encoder
      if (elevatorHeight != ElevatorHeightState.NONE) {
        elevatorPID1.setReference(smoothedTarget, ControlType.kPosition);
        System.out.println("st"+smoothedTarget);
      } else {
          // Normal motion: move toward the smoothed target
          System.out.println("currentpose"+currentPosition);
          elevatorPID1.setReference(currentPosition, ControlType.kPosition);
      }
    } 
}
