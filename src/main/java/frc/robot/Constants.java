// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.CustomTypes.PID;
import frc.robot.CustomTypes.SwerveCanIDs;
import frc.robot.CustomTypes.SwerveModuleLocations;
import frc.robot.CustomTypes.Math.Vector2;

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

  public static final double TELESCOPE_PIVOT_GEAR_RATIO = 81;
  public static final double INTAKE_PIVOT_ROTATIONS_PER_DEGREE = (270.0/360.0)/*(270/(double)360)*/;
  // 300*(36/24) = gear ratio
  public static final double ALGAE_INTAKE_PIVOT_ROTATIONS_PER_DEGREE =1;//((double)(150*36/(double)24)/360);
  public static final double CLIMBER_ROTATIONS_PER_DEGREE = 125/(double)360;

  //Operator Constants
  public static class OperatorConstants 
  {
    public static final int kDriverControllerPort = 0;
  }

  public static class LaserCanConstants
  {
    public static final int LASERCAN_ID1 = 55;
    public static final int LASERCAN_ID2 = 56;

    public static final int LASERCAN_DISTANCE_CORAL_IN1 = 50; //its mm
    public static final int LASERCAN_DISTANCE_CORAL_OUT1 = 0; //its mm
    public static final int LASERCAN_DISTANCE_CORAL_IN2 = 50; //its mm
    public static final int LASERCAN_DISTANCE_CORAL_OUT2 = 0; //its mm
  }
//guh
  public static class ButtonConstants
  {
    //elevator
    // public static final int ELEVATOR_L1 = 1;
    public static final int ELEVATOR_RESET = 1;
    public static final int ELEVATOR_INTAKE = 6;
    public static final int ELEVATOR_L2 = 7;
    public static final int ELEVATOR_L3 = 8;
    public static final int ELEVATOR_L4 = 3;
    // public static final int ELEVATOR_DRIVE = 1;

    //coral intake
    public static final int CORAL_NONE = 6;
    public static final int CORAL_INTAKE = 5;
    public static final int CORAL_OUTTAKE_L1 = 3;
    public static final int CORAL_OUTTAKE_L2_TO_L3 = 4;
    public static final int CORAL_OUTTAKE_L4 = 9;

    //algae intake
    public static final int ALGAE_INTAKE_INTAKE = 1;
    public static final int ALGAE_INTAKE_OUTTAKE = 1;
    public static final int ALGAE_INTAKE_NONE = 1;

    //algae pivot
    public static final int ALGAE_PIVOT_DRIVE = 1;
    public static final int ALGAE_PIVOT_GROUND = 1;
    public static final int ALGAE_PIVOT_REEF = 1;
    public static final int ALGAE_PIVOT_SHOOT = 1;
  }

  //Elevator Constants
  public final class ElevatorConstants
  {
    //IDs
    public static final int ELEVATOR_HEIGHT_MOTOR1_ID = 41;
    public static final int ELEVATOR_HEIGHT_MOTOR2_ID = 42;

    //switch ports
    public static final int ELEVATOR_LIMIT_SWITCH_PORT1 = 0;
    public static final int ELEVATOR_LIMIT_SWITCH_PORT2 = 1;

    //PIDs

    //P = gain, tells motor to accelerate towards target velocity
    //i = error, starts to remove error that occurs when over/under-shooting target velocity
    //d = change, tells motor it needs to start to slow as it reaches target velocity

    public static final PID ELEVATOR_HEIGHT_PID = new PID(0.03,0.000008,0.00005,0,0); //
    //public static final PID ARM_TELESCOPE_VELOCITY_PID = new PID(0,0.0,0.0,.00001,0);
    // public static final PID ELEVATOR_INTAKE_PID = new PID(0,0,0,0.0001,0);

    //speeds
    public static final double ELEVATOR_HEIGHT_SPEED = 10.0;
    public static final double ELEVATOR_INTAKE_SPEED = 2500.0;    

    // public static final double LEVEL_ONE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*.04));
    public static final double LEVEL_TWO = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*.07));
    public static final double LEVEL_THREE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*.14));
    public static final double LEVEL_FOUR = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*.27));
    public static final double DRIVE      = 0;
    public static final double INTAKE     = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*.0233));
    public static final double ALGAE =INTAKE_PIVOT_ROTATIONS_PER_DEGREE*360*.2;
  }

  //Intake Constants
  public final class AlgaeIntakeConstants
  {

    //encoder ids
    //public static final int ALGAE_PIVOT_MOTOR_ENCODER_ID = 61;
    public static final int ALGAE_INTAKE_RUN_MOTOR_ID = 51;
    public static final int ALGAE_PIVOT_MOTOR_ID = 52;

    //PID's
    public static final PID ALGAE_INTAKE_RUN_PID = new PID(0.0,0,0,0.000085,0);
    public static final PID ALGAE_INTAKE_PIVOT_PID = new PID(0.003,0,0,0,0);

    public static final double ALGAE_INTAKE_HOLD_CURRENT = 1.5f;

    //speeds
     public static final int ALGAE_INTAKE_RUN_SPEED = 3000;

    //Pivot Angle
    public static final double ALGAE_GROUND_ANGLE = 0;
    public static final double ALGAE_REEF_ANGLE = 0;
    public static final double SHOOT_ANGLE = 0;
    public static final double DRIVE_ANGLE = 0;

     //Heights
    //  public static final double PROCESSOR_HEIGHT = ;//(ALGAE_INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 10);
    //  public static final double ALGAE_ON_REEF = ;

     public static final double GROUND = 50;
    //  public static final double DRIVE_HEIGHT = PROCESSOR_HEIGHT;//(ALGAE_INTAKE_PIVOT_ROTATIONS_PER_DEGREE *80);
     
  }
  public static final class CoralIntakeConstants {

    public static final int CORAL_INTAKE_LASERCAN_ID1 = 55;
    public static final int CORAL_INTAKE_LASERCAN_ID2 = 56;

    public static final int CORAL_INTAKE_MOTOR_ID1 = 61;
    public static final int CORAL_INTAKE_MOTOR_ID2 = 62;

    public static final PID CORAL_INTAKE_PID = new PID(0,0,0,0.000085, 0); // ripped from algae intake constant. might not be good.
    public static final int CORAL_INTAKE_RUN_SPEED_NORMAL = 2000;
    public static final int CORAL_INTAKE_RUN_SPEED_L1 = CORAL_INTAKE_RUN_SPEED_NORMAL - 1000;
  }
   public static final class ControllerConstants 
   {
      public static final int DRIVER_STICK_CHANNEL = 0;
      public static final int AUX_STICK_CHANNEL    = 1;
      public static final double DEADZONE_DRIVE    = 0.1;
      public static final double DEADZONE_STEER    = 0.3;
    }

    public static final class DriveConstants {
      // Final Robot Constants
      // 11.875 for 29" side (front)
      // 12.375 for 30" side (side)
        public static final SwerveModuleLocations Robot2025SwerveLocations = new SwerveModuleLocations(
            11.875  * MathConstants.INCH_TO_METER, // LEFT_FRONT_WHEEL_X
            12.375  * MathConstants.INCH_TO_METER,   // LEFT_FRONT_WHEEL_Y
            11.875   * MathConstants.INCH_TO_METER, // RIGHT_FRONT_WHEEL_X
            -12.375 * MathConstants.INCH_TO_METER,   // RIGHT_FRONT_WHEEL_Y
            -11.875  * MathConstants.INCH_TO_METER, // RIGHT_REAR_WHEEL_X
            -12.375 * MathConstants.INCH_TO_METER,   // RIGHT_REAR_WHEEL_Y
            -11.875  * MathConstants.INCH_TO_METER, // LEFT_REAR_WHEEL_X
            11.375  * MathConstants.INCH_TO_METER    // LEFT_REAR_WHEEL_Y
        ); 
        // in case the autofill doesnt show, the can ids go as follows.
        // L/R F/B D/S M for left/right front/back drive/steer motor. it goes in order of lf,rf,lr,rr with drive first 
    
        public static final SwerveCanIDs Robot2025SwerveCAN = new SwerveCanIDs(
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
      public static final double WHEEL_RADIUS = 2.0 * MathConstants.INCH_TO_METER; // meters * 0.98
      public static final double WHEEL_CIRCUMFERENCE = 2.0 * Math.PI * WHEEL_RADIUS; // meters/revolution
      public static final double MAX_DRIVE_SPEED = 5.5; // meters/second
      public static final double MAX_STEER_RATE = .5; // rotations/second of a wheel for steer.
      public static final double MAX_TWIST_RATE = .6 * 2.0 * Math.PI; // radians/second of the robot rotation.
      public static final double CONTROLLER_TWIST_RATE = 2; // constant turn rate for using controller
      public static final int PIGEON_2_ID = 9;
      public static final double DRIVE_GEAR_RATIO = 1.0/5.9;//.169;
      public static final double STEER_GEAR_RATIO = .05333333333;
      public static final PID PID_SparkMax_Steer = new PID(0.0004,0,0.0001,0,0.0001);
      //public static final PID PID_SparkMax_Steer = new PID(0.0005,0.00000018,0.001,0,0.0001);
      public static final PID PID_Encoder_Steer = new PID(15, 10, .1);
      public static final PID PID_SparkFlex_Drive = new PID(0.0003,0.0000001,0.005,0,0.0002);
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