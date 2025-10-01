// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralIntakeConstants;

public class CoralIntakeSubsystem extends SubsystemBase {

  CoralIntakeState cisc;

  private SparkMax IntakeMotorOne;
  private RelativeEncoder IntakeEncoderOne;
  private SparkClosedLoopController IntakePIDOne;
  private DigitalInput limswitch1;
  private DigitalInput limswitch2;
  private SparkMax IntakeMotorTwo;
  private RelativeEncoder IntakeEncoderTwo;
  private SparkClosedLoopController IntakePIDTwo;

  public enum CoralIntakeState
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  /** Creates a new CoralIntakeSubsystem. */
  public CoralIntakeSubsystem() 
  {
    IntakeMotorOne = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig cfg1 = CoralIntakeConstants.CORAL_INTAKE_PID.setSparkMaxPID(IntakeMotorOne,IdleMode.kBrake);
    IntakeEncoderOne = IntakeMotorOne.getEncoder();
    IntakePIDOne = IntakeMotorOne.getClosedLoopController();
    
    IntakeMotorTwo = new SparkMax(0, SparkLowLevel.MotorType.kBrushless);
    SparkMaxConfig cfg2 = CoralIntakeConstants.CORAL_INTAKE_PID.setSparkMaxPID(IntakeMotorTwo,IdleMode.kBrake);
    cfg2.follow(IntakeMotorOne,true); // take default settings applied by our PID configure function and add the follow.
    // this may not work, idk, i have yet to try it
    IntakeMotorTwo.configure(cfg2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakeEncoderTwo = IntakeMotorTwo.getEncoder();
    IntakePIDTwo = IntakeMotorTwo.getClosedLoopController();

    limswitch1=new DigitalInput(CoralIntakeConstants.CORAL_INTAKE_LIMSWITCH1_PORT);
    limswitch2=new DigitalInput(CoralIntakeConstants.CORAL_INTAKE_LIMSWITCH2_PORT);
  }

  public void SetCoralIntakeState(CoralIntakeState state)
  {
    cisc = state;
    UpdateCoralIntakeState();
  }

  private void UpdateCoralIntakeState()
  {
    switch(cisc)
    {
      
      case NONE:   IntakePIDOne.setReference(0,ControlType.kVelocity); 
      break;
      case INTAKE: IntakePIDOne.setReference(CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;
      case OUTTAKE:IntakePIDOne.setReference(-CoralIntakeConstants.CORAL_INTAKE_RUN_SPEED,ControlType.kVelocity);
      break;
    }
  }

  @Override
  public void periodic() 
  {
    if (
      cisc == CoralIntakeState.INTAKE && (
        !limswitch1.get() ||
        !limswitch2.get()
      )
    ) {
      SetCoralIntakeState(CoralIntakeState.NONE);
      // we captured our coral. ideally we execute a command here which tells us to move to some L# but we cant gurantee we dont get stuck on the top of the dispenser and i dont think its a good idea to execute commands in subsystems. maybe ill change it tho idk
    }
  }
}
