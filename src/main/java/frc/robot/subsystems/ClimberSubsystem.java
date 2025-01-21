// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase 
{

  private SparkMax climberMotor;
  private RelativeEncoder climberEncoder;
  private SparkClosedLoopController climberPID;
  

  /** Creates a new Climber. */
  public ClimberSubsystem()
  {
    climberMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();
    climberPID = climberMotor.getClosedLoopController();

    ClosedLoopConfig climberCLC = new ClosedLoopConfig();
    SparkMaxConfig climberSMC = new SparkMaxConfig();
    climberCLC.pidf(ClimberConstants.ClimberPIDs.P, ClimberConstants.ClimberPIDs.I, ClimberConstants.ClimberPIDs.D, ClimberConstants.ClimberPIDs.kFF);
    climberCLC.iZone(ClimberConstants.ClimberPIDs.IZ);
    climberSMC.apply(climberCLC);
    climberMotor.configure(climberSMC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
