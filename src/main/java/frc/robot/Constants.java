// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.CustomTypes.PID;
import frc.robot.CustomTypes.SwerveCanIDs;
import frc.robot.CustomTypes.SwerveModuleLocations;

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
    public static final int ARM_TELESCOPE_MOTOR_ID = 32;
    public static final PID ARM_TELESCOPE_PID = new PID(0,0,0,0,0);

    public static final int ARM_PIVOT_MOTOR_ID = 31;
    public static final PID ARM_PIVOT_PID = new PID(0,0,0,0,0);

    public static final int ARM_INTAKE_MOTOR_ID = 33;
    public static final PID ARM_INTAKE_PID = new PID(0,0,0,0,0);

    public static final double ARM_TELESCOPE_SPEED = 10.0;

    public static final double ARM_INTAKE_RUN_SPEED = 5000.0;

    public static final double INTAKE_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 55); //all mathed up
    public static final double OUTTAKE_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 35); //all mathed up
    public static final double NORMAL_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 25); // idle spot

    public static final double LEVEL_ONE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 40);
    public static final double LEVEL_TWO = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 60);
    public static final double LEVEL_THREE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 80);
    public static final double LEVEL_FOUR = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 100);

  }

  //Intake Constants
  public final class AlgaeIntakeConstants
  {
    public static final int ALGAE_INTAKE_RUN_MOTOR_ID = 51;
    public static final PID ALGAE_INTAKE_RUN_PID = new PID(0,0,0,0,0);

    public static final int ALGAE_PIVOT_MOTOR_ID = 50;
    public static final PID ALGAE_INTAKE_PIVOT_PID = new PID(0,0,0,0,0);

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
    public static final PID CLIMBER_LEFT_PID = new PID(0,0,0,0,0);

    public static final int CLIMBER_RIGHT_MOTOR_ID = 41;
    public static final PID CLIMBER_RIGHT_PID = new PID(0,0,0,0,0);

    public static final int MAX_CLIMBER_POSE = 100;
    public static final int MIN_CLIMBER_POSE = 0;

    public static final double climberSpeed = 5000.0;

  }
   public static final class ControllerConstants {
      public static final int DRIVER_STICK_CHANNEL = 0;
      public static final int AUX_STICK_CHANNEL    = 1;
      public static final double DEADZONE_DRIVE    = 0.1;
      public static final double DEADZONE_STEER    = 0.3;
    }
    public static final class DriveConstants {
      // Final Robot Constants
        public static final SwerveModuleLocations Robot2025SwerveLocations = new SwerveModuleLocations(
            12.375   * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
            9.375  * MathConstants.INCH_TO_METER,   // LEFT_FRONT_WHEEL_Y
            12.375   * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
            -9.375 * MathConstants.INCH_TO_METER,   // RIGHT_FRONT_WHEEL_Y
            -12.375  * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
            -9.375 * MathConstants.INCH_TO_METER,   // RIGHT_REAR_WHEEL_Y
            -12.375  * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
            9.375  * MathConstants.INCH_TO_METER    // LEFT_REAR_WHEEL_Y
        ); 
        // in case the autofill doesnt show, the can ids go as follows.
        // L/R F/B D/S M for left/right front/back drive/steer motor. it goes in order of lf,rf,lr,rr with drive first 
        public static final SwerveCanIDs Robot2025SwerveCAN = new SwerveCanIDs(
            13, 
            12, 
            11, 
            10, 
            17, 
            16, 
            15, 
            14, 
            60, //hmm
            61, //hmm
            63, //hmm
            62  //hmm
        );


          // Test Robot Constants
          public static final SwerveCanIDs AdrianBotSwerveCAN = new SwerveCanIDs(
            10, 
            20, 
            11, 
            21, 
            13, 
            23, 
            12, 
            22, 
            30, 
            31, 
            33, 
            32
           ); 

        public static final SwerveModuleLocations AdrianBotSwerveLocations = new SwerveModuleLocations(
            12.375   * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
            9.375  * MathConstants.INCH_TO_METER,   // LEFT_FRONT_WHEEL_Y
            12.375   * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
            -9.375 * MathConstants.INCH_TO_METER,   // RIGHT_FRONT_WHEEL_Y
            -12.375  * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
            -9.375 * MathConstants.INCH_TO_METER,   // RIGHT_REAR_WHEEL_Y
            -12.375  * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
            9.375  * MathConstants.INCH_TO_METER    // LEFT_REAR_WHEEL_Y
          ); 
      public static final SwerveCanIDs ROBOT_SWERVE_CAN = Robot2025SwerveCAN;
      public static final SwerveModuleLocations ROBOT_SWERVE_LOCATIONS = Robot2025SwerveLocations;
      public static final double WHEEL_RADIUS = 2.0 * 0.0254; // meters * 0.98
      public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution
      public static final double MAX_DRIVE_SPEED = 3.5; // meters/second
      public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
      public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.
      public static final double CONTROLLER_TWIST_RATE = 2; // constant turn rate for using controller
      public static final int PIGEON_2_ID = 29;
      public static final double DRIVE_GEAR_RATIO = .169;
      public static final double STEER_GEAR_RATIO = .05333333333;
      public static final PID PID_SparkMax_Steer = new PID(0.0003,0.0000018,0,0,0.0001);
      public static final PID PID_Encoder_Steer = new PID(15, 10, .1);
      public static final PID PID_SparkFlex_Drive = new PID(0.00018,0.0000005,0,0,0.00013);
      public static final double AUTO_ODOMETRY_DRIVE_MIN_SPEED = .1;
      public static final double AUTO_ODOMETRY_DRIVE_MAX_SPEED = 2;
      public static final double AUTO_ODOMETRY_DRIVE_TARGET_ALLOWED_ERROR = .1; // in meters
      public static final double AUTO_ODOMETRY_DRIVE_SLOWDOWN_DISTANCE = .6; // in meters
  }
  public static final class MathConstants
  {
    public static final double INCH_TO_METER = 0.0254;
  }
}