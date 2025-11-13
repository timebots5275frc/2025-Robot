// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LaserCANSubsystem extends SubsystemBase {

  public static LaserCan lc1;
  public static LaserCan lc2;

  Measurement lcm1;
  Measurement lcm2;

  /** Creates a new LaserCANSubsystem. */
  public LaserCANSubsystem(){
   // idk 
  }
  
  //inside
  public boolean LC1()
  {
    if( lcm1 == null || lcm1.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("LC No Valid Data");
      return false;
    }
    System.out.println("triple ham");
    //if (lcm1 == null){return false};
    if(lcm1.distance_mm <= Constants.LaserCanConstants.LASERCAN_DISTANCE_CORAL_IN1){return true;} 
    else{return false;}
  }

  //outside
  public boolean LC2()
  {

    if ( lcm2 == null || lcm2.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      return false;
    }
    System.out.println("triple ham");
    //if (lcm2  == null) return false;
    if(lcm2.distance_mm <= Constants.LaserCanConstants.LASERCAN_DISTANCE_CORAL_IN2){return true;} 
    else{return false;}
  }

  @Override
  public void periodic() 
  {
    lcm1 = lc1.getMeasurement();
    lcm2 = lc2.getMeasurement();
    //SmartDashboard.putNumber("lcm1", lc1.getMeasurement().distance_mm);
    SmartDashboard.putBoolean("lcm1b", LC1());
    //SmartDashboard.putNumber("lcm2", lc2.getMeasurement().distance_mm);
    SmartDashboard.putBoolean("lcm2b", LC2());
    // This method will be called once per scheduler run
  }
}
