// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.subsystems.ArmSubsystem;
// import frc.robot.subsystems.ArmSubsystem.armIntakeState;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class ArmIntakeCommand extends InstantCommand {
//   private armIntakeState ais;
//   private ArmSubsystem as;
//   public ArmIntakeCommand(ArmSubsystem as, armIntakeState ais) {
//     addRequirements(as);
//     this.as=as;
//     this.ais = ais;
    
//     // Use addRequirements() here to declare subsystem dependencies.
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {as.SetIntakeState(ais);}
// }
