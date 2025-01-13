// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkJNI;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkRelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake extends SubsystemBase 
{

  private CANSparkMax intakeRunMotor;
  private Spark intakeRunEncoder;
  private SparkPIDController intakeRunPID;

  /** Creates a new Intake. */

  public enum IntakeRunstate
  {
    NONE,
    INTAKE,
    OUTTAKE;
  }

  public Intake() 
  {
    intakeRunMotor = new CANSpakMax();
    intakeRunEncoder = intakeRunMotor.getEncoder();
    intakeRunPID = intakeRunMotor.getPIDController();

    intakeRunMotor.setP(Constants.IntakeConstants.IntakeRunPIDs.P);
    intakeRunMotor.setI(Constants.IntakeConstants.IntakeRunPIDs.I);
    intakeRunMotor.setD(Constants.IntakeConstants.IntakeRunPIDs.D);
    intakeRunMotor.setFF(Constants.IntakeConstants.IntakeRunPIDs.kFF);
  }

  public void IntakeRunState(IntakeRunstate state)
  {
    switch(state)
    {
      case NONE: intakeRunPID.setReference(0, ControlType.kVelocity);
      break;

      case INTAKE: intakeRunPID.setReference(Constants.IntakeConstants.INTAKE_RUN_SPEED, ControlType.kVelocity);
      break;

      case OUTTAKE: intakeRunPID.setReference(-Constants.IntakeConstants.INTAKE_RUN_SPEED * 0.5, ControlType.kVelocity);
      break;

    }
  }

  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run
  }
}
