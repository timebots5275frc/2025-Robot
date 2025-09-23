// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveTrain;


import java.util.List;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

import frc.robot.CustomTypes.Math.Vector2;

public class SwerveDrive extends SubsystemBase {
  private final Translation2d leftFrontWheelLocation  = new Translation2d(DriveConstants.ROBOT_SWERVE_LOCATIONS.LEFT_FRONT_WHEEL_X,  DriveConstants.ROBOT_SWERVE_LOCATIONS.LEFT_FRONT_WHEEL_Y);
  private final Translation2d rightFrontWheelLocation = new Translation2d(DriveConstants.ROBOT_SWERVE_LOCATIONS.RIGHT_FRONT_WHEEL_X, DriveConstants.ROBOT_SWERVE_LOCATIONS.RIGHT_FRONT_WHEEL_Y);    
  private final Translation2d rightRearWheelLocation  = new Translation2d(DriveConstants.ROBOT_SWERVE_LOCATIONS.RIGHT_REAR_WHEEL_X,  DriveConstants.ROBOT_SWERVE_LOCATIONS.RIGHT_REAR_WHEEL_Y);
  private final Translation2d leftRearWheelLocation   = new Translation2d(DriveConstants.ROBOT_SWERVE_LOCATIONS.LEFT_REAR_WHEEL_X,   DriveConstants.ROBOT_SWERVE_LOCATIONS.LEFT_REAR_WHEEL_Y);

  private final SwerveModule leftFrontSwerveModule  = new SwerveModule(DriveConstants.ROBOT_SWERVE_CAN.LEFT_FRONT_DRIVE_MOTOR_ID,  DriveConstants.ROBOT_SWERVE_CAN.LEFT_FRONT_STEER_MOTOR_ID, DriveConstants.ROBOT_SWERVE_CAN.LEFT_FRONT_STEER_ENCODER_ID,"LF");
  private final SwerveModule rightFrontSwerveModule = new SwerveModule(DriveConstants.ROBOT_SWERVE_CAN.RIGHT_FRONT_DRIVE_MOTOR_ID, DriveConstants.ROBOT_SWERVE_CAN.RIGHT_FRONT_STEER_MOTOR_ID, DriveConstants.ROBOT_SWERVE_CAN.RIGHT_FRONT_STEER_ENCODER_ID,"RF");
  private final SwerveModule rightRearSwerveModule  = new SwerveModule(DriveConstants.ROBOT_SWERVE_CAN.RIGHT_REAR_DRIVE_MOTOR_ID,  DriveConstants.ROBOT_SWERVE_CAN.RIGHT_REAR_STEER_MOTOR_ID, DriveConstants.ROBOT_SWERVE_CAN.RIGHT_REAR_STEER_ENCODER_ID,"RR");
  private final SwerveModule leftRearSwerveModule   = new SwerveModule(DriveConstants.ROBOT_SWERVE_CAN.LEFT_REAR_DRIVE_MOTOR_ID,   DriveConstants.ROBOT_SWERVE_CAN.LEFT_REAR_STEER_MOTOR_ID, DriveConstants.ROBOT_SWERVE_CAN.LEFT_REAR_STEER_ENCODER_ID,"LR");
  
  Pigeon2 pigeon2Gyro = new Pigeon2(DriveConstants.PIGEON_2_ID);

    public final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftFrontWheelLocation, rightFrontWheelLocation, rightRearWheelLocation, leftRearWheelLocation);
    
    public final SwerveModulePosition[] startingSwerveModulePositions = new SwerveModulePosition[] {leftFrontSwerveModule.getPosition(), rightFrontSwerveModule.getPosition(), rightRearSwerveModule.getPosition(), leftRearSwerveModule.getPosition()};
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(kinematics, this.getHeading(), startingSwerveModulePositions);

    SwerveModulePosition[] currentSwerveModulePositions = startingSwerveModulePositions;
    
    public void Drivetrain() {
        flip = false;
        System.out.println("SwerveDrive.java started...");
    }
    
    @Override
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Heading", getHeading().getDegrees());
        SmartDashboard.putNumber("LF Drive Speed", leftFrontSwerveModule.driveNEOVortexMotorEncoder.getVelocity());
        SmartDashboard.putNumber("RF Drive Speed", rightFrontSwerveModule.driveNEOVortexMotorEncoder.getVelocity());
        SmartDashboard.putNumber("LR Drive Speed", leftRearSwerveModule.driveNEOVortexMotorEncoder.getVelocity());
        SmartDashboard.putNumber("RR Drive Speed", rightRearSwerveModule.driveNEOVortexMotorEncoder.getVelocity());
    }
    private boolean flip;
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates((fieldRelative && !flip )? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, this.getHeading()) : new ChassisSpeeds(-ySpeed, xSpeed, rot));
        SmartDashboard.putNumber("RRDesiredM/S", swerveModuleStates[3].speedMetersPerSecond);
        SmartDashboard.putNumber("RRRotDeg", swerveModuleStates[3].angle.getDegrees());
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.MAX_DRIVE_SPEED);
        SmartDashboard.putBoolean("Field Relative", fieldRelative && !flip);
        
        leftFrontSwerveModule.setDesiredState(swerveModuleStates[0], true);
        rightFrontSwerveModule.setDesiredState(swerveModuleStates[1], true);
        rightRearSwerveModule.setDesiredState(swerveModuleStates[2], true);
        leftRearSwerveModule.setDesiredState(swerveModuleStates[3], true);

        currentSwerveModulePositions = new SwerveModulePosition[] { leftFrontSwerveModule.getPosition(), rightFrontSwerveModule.getPosition(), rightRearSwerveModule.getPosition(), leftRearSwerveModule.getPosition(), };

        SmartDashboard.putString("positions", currentSwerveModulePositions[0].distanceMeters+" "+currentSwerveModulePositions[1].distanceMeters+" "+currentSwerveModulePositions[2].distanceMeters+" "+currentSwerveModulePositions[3].distanceMeters+" ");
        SmartDashboard.putNumber("left front pos", leftFrontSwerveModule.getPosition().distanceMeters);
        SmartDashboard.putString("Odometry Pos", this.getOdometryPosition().toString());
        SmartDashboard.putString("Odometry Rot", m_odometry.getPoseMeters().getRotation().toString());
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    public Vector2 getOdometryPosition()
    {
        Pose2d pose = m_odometry.getPoseMeters();
        return new Vector2(pose.getX(), pose.getY());
    }

    /** Updates the field relative position of the robot. */
    public void updateOdometry() {
        m_odometry.update(this.getHeading(), currentSwerveModulePositions);
    }

    /**
     * Resets the odometry Position and Angle to 0.
     */
    public void resetOdometry() {
        System.out.println("resetOdometry");
        m_odometry.resetPosition(this.getHeading(), currentSwerveModulePositions, new Pose2d(0,0, new Rotation2d(0)));
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometryWithPose2d(Pose2d pose) {
        System.out.println("resetOdometryWithPose2d");
        m_odometry.resetPosition(pose.getRotation(), startingSwerveModulePositions, pose); // imuADIS16470.getRotation2d()
    }

    public void resetPigeon() {
        pigeon2Gyro.setYaw(90);
    }

    public void setGyroYaw(double degrees) {
        pigeon2Gyro.setYaw(degrees);
    }

    /**
     * Returns the heading of the robot.
     * 
     * @return the robot's heading in degrees, from -180 to 180. // ! This comment
     *         was from last year.
     */
    public Rotation2d getHeading() {

        Rotation2d heading = Rotation2d.fromDegrees(pigeon2Gyro.getYaw().getValueAsDouble());

        // System.out.println( "getYComplementaryAngle = " + heading );
        // System.out.println( "getXComplementaryAngle = " +
        // imuADIS16470.getXComplementaryAngle() );

        return heading; // TODO Lucas //.minus(new Rotation2d(this.autoTurnOffsetRadians)); // radians

    }
    public void flipFieldRelative() {flip = !flip;}
    public double getGyroYawInDegrees() { return pigeon2Gyro.getYaw().getValueAsDouble(); }

    /**
     * Sets the swerve ModuleStates.
     * 
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.MAX_DRIVE_SPEED);

        leftFrontSwerveModule.setDesiredState(desiredStates[0], true);
        rightFrontSwerveModule.setDesiredState(desiredStates[1], true);
        rightRearSwerveModule.setDesiredState(desiredStates[2], true);
        leftRearSwerveModule.setDesiredState(desiredStates[3], true);
    }

    public void alignWheels() {
        SwerveModuleState desiredStates = new SwerveModuleState(0, new Rotation2d(0));

        leftFrontSwerveModule.setDesiredState(desiredStates, true);
        rightFrontSwerveModule.setDesiredState(desiredStates, true);
        rightRearSwerveModule.setDesiredState(desiredStates, true);
        leftRearSwerveModule.setDesiredState(desiredStates, true);
    }

    public static Trajectory generateTrajectory(TrajectoryConfig config, List<Pose2d> list) {
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(list, config);
        return exampleTrajectory;
    }
}