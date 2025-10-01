// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CustomTypes.Math.Vector2;
import frc.robot.subsystems.DriveTrain.SwerveDrive;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoDrive extends Command {
  /** Creates a new AutoDrive. */
  private static final double kP = .1;
	private static final double MAX_CORRECTION = 1;
  SwerveDrive sd;
  double distance;
  double power;
  double goalAngle;
  public AutoDrive(double distance, double power, SwerveDrive sd) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.distance = distance;
    this.power = power;
    this.sd = sd;
    addRequirements(sd);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sd.drive(0,0,0,false);
    goalAngle = sd.getGyroYawInDegrees();
    sd.resetOdometry();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double error = goalAngle - sd.getGyroYawInDegrees();
		
		double correction = kP * error;

		correction = Math.min(MAX_CORRECTION, correction);
		correction = Math.max(-MAX_CORRECTION, correction);
		
		sd.drive(power, 0, -1 * correction, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    sd.drive(0, 0, 0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    Vector2 pos = sd.getOdometryPosition();
		return Math.abs(pos.x)>=Math.abs(distance);
  }
}
