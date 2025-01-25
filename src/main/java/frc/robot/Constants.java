// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import CustomTypes.PID_Values;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{

  public static final double INTAKE_PIVOT_GEAR_RATIO = 90;
  public static final double INTAKE_PIVOT_ROTATIONS_PER_DEGREE = INTAKE_PIVOT_GEAR_RATIO / 360;

  //Operator Constants
  public static class OperatorConstants 
  {
    public static final int kDriverControllerPort = 0;
  }

  //Arm Constants
  public final class ArmConstants
  {
    public static final int ARM_TELESCOPE_MOTOR_ID = 20;
    public static final PID_Values ARM_TELESCOPE_PID = new PID_Values(0,0,0,0,0);

    public static final int ARM_PIVOT_MOTOR_ID = 21;
    public static final PID_Values ARM_PIVOT_PID = new PID_Values(0,0,0,0,0);

    public static final int ARM_INTAKE_MOTOR_ID = 22;
    public static final PID_Values ARM_INTAKE_PID = new PID_Values(0,0,0,0,0);

    public static final double ARM_TELESCOPE_SPEED = 10.0;

    public static final double ARM_INTAKE_RUN_SPEED = 5000.0;

    public static final double INTAKE_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 55); //all mathed up
    public static final double OUTTAKE_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 35); //all mathed up

  }

  //Intake Constants
  public final class AlgaeIntakeConstants
  {
    public static final int ALGAE_INTAKE_RUN_MOTOR_ID = 30;
    public static final PID_Values ALGAE_INTAKE_RUN_PID = new PID_Values(0,0,0,0,0);

    public static final int ALGAE_PIVOT_MOTOR_ID = 31;
    public static final PID_Values ALGAE_PIVOT_PID = new PID_Values(0,0,0,0,0);

    public static final int ALGAE_INTAKE_PIVOT_MAX_ACCELERATION = 10000;
    public static final int ALGAE_INTAKE_PIVOT_MAX_VELOCITY = 5000;
    public static final int ALGAE_INTAKE_PIVOT_MIN_VELOCITY = 2000;

     public static final int ALGAE_INTAKE_RUN_SPEED = 500;
     public static final int ALGAE_INTAKE_RUN_SPEED_MAX = 2000;

     public static final int ALGAE_INTAKE_PIVOT_SPEED = 3000;

     public static final double PROCESSOR = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 90);

     public static final double GROUND = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 120);

     public static final double DRIVE_HEIGHT = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 90);
     
  }

  //Climber Constants
  public final class ClimberConstants
  {
    public static final int CLIMBER_LEFT_MOTOR_ID = 40;
    public static final PID_Values CLIMBER_LEFT_PID = new PID_Values(0,0,0,0,0);

    public static final int CLIMBER_RIGHT_MOTOR_ID = 41;
    public static final PID_Values CLIMBER_RIGHT_PID = new PID_Values(0,0,0,0,0);

    public static final int MAX_CLIMBER_POSE = 100;

    public static final double climberSpeed = 5000.0;

  }
}
