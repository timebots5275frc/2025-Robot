// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;

public class NewSensorTestClass extends SubsystemBase {
  /** Creates a new NewSensorTestClass. */
  private LaserCan lc;
  private RegionOfInterest roi;
  private RangingMode rm;
  // private int measurement;

  public NewSensorTestClass() 
  {
    
      // rm.setRangingMode(new RangingMode.()/*create some ranging modes*/);
      // roi.setRegionOfInterest(new RegionOfInterest(0,0,0,0));
  }

  @Override
  public void periodic() 
  {
    LaserCan.Measurement measurement = lc.getMeasurement();
    SmartDashboard.putString("LCM", measurement.toString());
    // This method will be called once per scheduler run
  }
}
