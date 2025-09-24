/* // Copyright (c) FIRST and other WPILib contributors.
 // Open Source Software; you can modify and/or share it under the terms of
 // the WPILib BSD license file in the root directory of this project.
 package frc.robot.commands;
 import static edu.wpi.first.units.Units.Rotation;
 import edu.wpi.first.math.geometry.Pose2d;
 import edu.wpi.first.math.geometry.Rotation2d;
 import edu.wpi.first.math.geometry.Translation2d;
 import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
 import edu.wpi.first.wpilibj.drive.DifferentialDrive;
 import edu.wpi.first.wpilibj2.command.Command;
 import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
 import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
 import edu.wpi.first.wpilibj2.command.WaitCommand;
 import frc.robot.Constants.MathConstants;
 import frc.robot.subsystems.ElevatorSubsystem;
 import frc.robot.subsystems.AlgaeIntakeSubsystem.IntakeRunstate;
 import frc.robot.subsystems.ElevatorSubsystem.armIntakeState;
 import frc.robot.subsystems.ElevatorSubsystem.armPivotState;
 import frc.robot.subsystems.ElevatorSubsystem.armTelescopeState;
 import frc.robot.subsystems.DriveTrain.SwerveDrive;

 public class AutoCommands
 {
     private static SwerveDrive sd;
     private static ElevatorSubsystem as = new ElevatorSubsystem();
         // most of these are assuming that we make the necessary changes to the coral intake as suggested
         //middle score l4 Coral
         public static Command MiddleCoralL4()
         { 
             return new SequentialCommandGroup(
                  new ParallelCommandGroup(
                     new AutoDrive(MathConstants.INCH_TO_METER*67,.5,sd).withTimeout(8.5),
                     new ArmIntakeCommand(as, armIntakeState.OUTTAKE),
                     new WaitCommand(.5),
                     new AutoDrive(MathConstants.INCH_TO_METER*10,.5,sd),
                     new ArmIntakeCommand(as, armIntakeState.INTAKE)
                  )
              );
         }
         //drive
         //outtake l4
         //drive to coral station
         //intake coral
         //drive to reef
         //outtake l4
         public static Command TwoCoralL4()
         { 
           return new SequentialCommandGroup(
             new ParallelCommandGroup(
                 new AutoDrive(MathConstants.INCH_TO_METER*67,2,sd),
                 new ArmIntakeCommand(as, armIntakeState.OUTTAKE),
                 new WaitCommand(.5),
                 //figure out rotation 2d/pose 2d
                 new AutoDrive(-MathConstants.INCH_TO_METER*10,.5,sd), 
                 new ArmIntakeCommand(as, armIntakeState.INTAKE).until(as.limitSwitchIsPressed)
           ));
         }
         //either spit algae out onto ground or big baller it into the barge
         //drive
         //outtake l4
         //algae l2 height
         //intake algae
         //drive
         //spit out algae onto ground
         //pick up algae in teleop
         //drive
         //outtake l4
         //algae l3 height
         //intake algae
         //drive
         //spit out algae onto ground
         //pick up algae in teleop
         //stinky limit switch test thing
         public static Command LSTesterMaBobThing()
         {
             return new SequentialCommandGroup(
                 new AutoDrive(MathConstants.INCH_TO_METER*10, .5, sd),
                 new ArmIntakeCommand(as, armIntakeState.INTAKE).until(as.limitSwitchIsPressed),
                 new AutoDrive(-MathConstants.INCH_TO_METER*15, .5, sd),
                 new ArmTelescopeSet(as, armTelescopeState.L2),
                 new ArmIntakeCommand(as, armIntakeState.OUTTAKE),
                 new WaitCommand(.5),
                 new ArmIntakeCommand(as, armIntakeState.OUTTAKE)
             );
         }
     }*/