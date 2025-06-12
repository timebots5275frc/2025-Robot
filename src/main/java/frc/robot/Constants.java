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

  //Arm Constants
  public final class ArmConstants
  {
    //ID's
    public static final int ARM_INTAKE_MOTOR_ID = 43;
    public static final int ARM_PIVOT_MOTOR_ID = 41;
    public static final int ARM_TELESCOPE_MOTOR_ID = 42;
    public static final int ARM_PIVOT_ENCODER_ID = 62;

    //switch ports
    public static final int ARM_INTAKE_SWITCH_PORT = 0;

    //PID's
    public static final PID ARM_TELESCOPE_PID = new PID(0.05,0.0,0.0,0,0);
    //public static final PID ARM_TELESCOPE_VELOCITY_PID = new PID(0,0.0,0.0,.00001,0);
    public static final PID ARM_PIVOT_PID = new PID(0.008,0,0,0,0);
    public static final PID ARM_INTAKE_PID = new PID(0,0,0,0.0001,0);

    //speeds
    public static final double ARM_TELESCOPE_SPEED = 10.0;
    public static final double ARM_INTAKE_RUN_SPEED = 2500.0;


    public static final double L2_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * -90);
    public static final double L3_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * -90);
    public static final double L4_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * -90);

    public static final double INTAKE_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * -60); 
    public static final double OUTTAKE_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * -90); 
    public static final double NORMAL_ANGLE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * -25); // idle spot

    public static final double BALL_REMOVAL_SERVICE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE *-90);
    public static final double BALL_REMOVAL_SERVICE2= OUTTAKE_ANGLE;//(INTAKE_PIVOT_ROTATIONS_PER_DEGREE *)
    public static final double ALGAE_REMOVE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE*360*.6);
    public static final double ALGAE_REMOVE2= (INTAKE_PIVOT_ROTATIONS_PER_DEGREE*360*.15);
    public static final double ALGAE_REMOVE3= (INTAKE_PIVOT_ROTATIONS_PER_DEGREE*360*69420);
    

    public static final double LEVEL_ONE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*.25));
    public static final double LEVEL_TWO = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*.10));
    public static final double LEVEL_THREE = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*1.35));
    public static final double LEVEL_FOUR = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*2.55));
    public static final double DRIVE      = 0;
    public static final double INTAKE     = (INTAKE_PIVOT_ROTATIONS_PER_DEGREE * (360*0));
  }

  //Intake Constants
  public final class AlgaeIntakeConstants
  {

    //encoder id's
    public static final int ALGAE_PIVOT_MOTOR_ENCODER_ID = 61;
    public static final int ALGAE_INTAKE_RUN_MOTOR_ID = 45;
    public static final int ALGAE_PIVOT_MOTOR_ID = 44;

    //PID's
    public static final PID ALGAE_INTAKE_RUN_PID = new PID(0.0,0,0,0.000085,0);
    public static final PID ALGAE_INTAKE_PIVOT_PID = new PID(0.003,0,0,0,0);

    //switch ports
    public static final int ALGAE_INTAKE_SWITCH1_PORT = 1;
    public static final int ALGAE_INTAKE_SWITCH2_PORT = 2;

    //speeds
     public static final int ALGAE_INTAKE_RUN_SPEED =3000;
     public static final int ALGAE_INTAKE_RUN_SPEED_MAX = 2000;
     public static final int ALGAE_INTAKE_PIVOT_SPEED = 3000;
     public static final int ALGAE_INTAKE_PIVOT_MAX_ACCELERATION = 10000;
    public static final int ALGAE_INTAKE_PIVOT_MAX_VELOCITY = 5000;
    public static final int ALGAE_INTAKE_PIVOT_MIN_VELOCITY = 2000;

     //Heights
     public static final double PROCESSOR_HEIGHT = 5;//(ALGAE_INTAKE_PIVOT_ROTATIONS_PER_DEGREE * 10);
     public static final double ALGAE_REEF_HEIGHT_L2 = 0;
     public static final double ALGAE_REEF_HEIGHT_L3 = 0;

     public static final double GROUND = 50;
     public static final double DRIVE_HEIGHT = PROCESSOR_HEIGHT;//(ALGAE_INTAKE_PIVOT_ROTATIONS_PER_DEGREE *80);
     
  }

  //Climber Constants
  public final class ClimberConstants
  {
    //ID's
    public static final int CLIMBER_LEFT_MOTOR_ID = 50;
    public static final int CLIMBER_RIGHT_MOTOR_ID = 51;
    public static final int CLIMBER_ENCODER_ID = 60;

    //PID's
    public static final PID CLIMBER_LEFT_PID = new PID(0.02,0.000004,0.0033,0,0);
    //public static final PID CLIMBER_RIGHT_PID = new PID(0.001,0.00001,0,0,0);

    //positions
    public static final double CLIMBER_DOWN_CURRENT = 10;
    public static final double CLIMBER_DOWN_POS = -145 * CLIMBER_ROTATIONS_PER_DEGREE;
    public static final double CLIMBER_UP_POS = -10 * CLIMBER_ROTATIONS_PER_DEGREE;
  }

  public static final class VisionConstants {
    public static final boolean ENABLE_LIMELIGHT_LIGHT_ON_ENABLE = true;
    public static final int VALUES_TO_AVERAGE = 3;
    public static final double TARGET_POSITION_ALLOWED_ERROR = .1; // meters
    public static final double LIMELIGHT_X_OFFSET = 0.31773; // meters

    public static final double LIMELIGHT_DATA_WAIT_TIME = .5; // seconds

    public static final double MAX_AMP_TARGET_DISTANCE = 3;
    public static final Vector2 AMP_VISION_DRIVE_TARGET = new Vector2(.07, .47);

    public static enum AprilTagData
    {
      //id's are not accurate
      CORAL_STATION_LEFT(1, "CS Left", DriverStation.Alliance.Blue,0,0),
      CORAL_STATION_RIGHT(2, "CS Right", DriverStation.Alliance.Blue,0,0),
      REEF_1(3, "Reef 1", DriverStation.Alliance.Red), //side closest to driver then it goes left around
      REEF_2(4, "Reef 2", DriverStation.Alliance.Red, 0, 16.6193978),
      REEF_3(5, "Reef 3", DriverStation.Alliance.Red, -2.7389074, 14.778355),
      REEF_4(6, "Reef 4", DriverStation.Alliance.Blue, -2.7389074, 1.858645),
      REEF_5(7, "Reef 5", DriverStation.Alliance.Blue, 0, 0),
      REEF_6(8, "Reef 6", DriverStation.Alliance.Blue,0,0),
      REEF_7(9, "Reef 7", DriverStation.Alliance.Red,0,0),
      REEF_8(10, "Reef 8", DriverStation.Alliance.Red,0,0),
      PROCESSOR_RED(11, "Processor", DriverStation.Alliance.Red,0,0),
      PROCESSOR_BLUE(12, "Processor", DriverStation.Alliance.Red,0,0);

      public final int id;
      public final String name;
      public final DriverStation.Alliance alliance;

      private AprilTagData(int id, String name, DriverStation.Alliance alliance)
      {
        this.id = id;
        this.name = name;
        this.alliance = alliance;
      }

      private AprilTagData(int id, String name, DriverStation.Alliance alliance, double x, double y)
      {
        this.id = id;
        this.name = name;
        this.alliance = alliance;
      }  
    }

    public static AprilTagData getTag(int id) {
      switch(id){
        case 1: return AprilTagData.CORAL_STATION_LEFT;
        case 2: return AprilTagData.CORAL_STATION_RIGHT;
        case 3: return AprilTagData.REEF_1;
        case 4: return AprilTagData.REEF_2;
        case 5: return AprilTagData.REEF_3;
        case 6: return AprilTagData.REEF_4;
        case 7: return AprilTagData.REEF_5;
        case 8: return AprilTagData.REEF_6;
        case 9: return AprilTagData.REEF_7;
        case 10: return AprilTagData.REEF_8;
        case 11: return AprilTagData.PROCESSOR_BLUE;
        case 12: return AprilTagData.PROCESSOR_RED;
        default: return null;
      }
    }
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
            11, //good
            21, //good
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
      public static final PID PID_SparkMax_Steer = new PID(0.0003,0.0000018,0.001,0,0.0001);
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