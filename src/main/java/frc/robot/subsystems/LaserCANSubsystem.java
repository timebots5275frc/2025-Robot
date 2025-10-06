// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LaserCANSubsystem extends SubsystemBase {

  LaserCan lc1;
  LaserCan lc2;

  double lcm1;
  double lcm2;

  /** Creates a new LaserCANSubsystem. */
  public LaserCANSubsystem() 
  {

  }

  public boolean LC1()
  {
    if(lcm1 >= 4){return true;}
    else{return false;}
  }

  public boolean LC2()
  {
    if(lcm2 <= 4){return true;}
    else{return false;}
  }

  @Override
  public void periodic() 
  {
    lc1.getMeasurement();
    lc2.getMeasurement();
    // This method will be called once per scheduler run
  }
}
