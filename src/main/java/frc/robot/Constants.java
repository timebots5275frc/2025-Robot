// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import CustomTypes.PID_Values;

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
  public static class OperatorConstants 
  {
    public static final int kDriverControllerPort = 0;
  }

  public final class IntakeConstants
  {
     public static final int INTAKE_RUN_SPEED = 1000;
     
     public static final PID_Values IntakeRunPIDs = new PID_Values(
      0.00014, /*P*/
      0.0, /*I*/
      0.0, /*D*/
      0.0, /*IZ*/
      0.0001 /*kFF*/
     );
  }
}
