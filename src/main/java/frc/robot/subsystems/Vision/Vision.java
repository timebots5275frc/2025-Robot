// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CustomTypes.Math.Vector3;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AprilTagData;

public class Vision extends SubsystemBase {

  private int aprilTagID = -1;
  private double horizontalOffsetFromAprilTag;

  private Vector3 avgAprilTagPosInRobotSpace;
  private ArrayList<Vector3> aprilTagPosInRobotSpaceValues = new ArrayList<Vector3>();

  private Vector3 avgAprilTagRotInRobotSpace;
  private ArrayList<Vector3> aprilTagRotInRobotSpaceValues = new ArrayList<Vector3>();

  private Vector3 avgRobotPosInFieldSpace;
  private ArrayList<Vector3> robotPosInFieldSpaceValues = new ArrayList<Vector3>();

  private Vector3 avgRobotRotInFieldSpace;
  private ArrayList<Vector3> robotRotInFieldSpaceValues = new ArrayList<Vector3>();

  public BooleanSupplier HasValidData = new BooleanSupplier() 
  {
    public boolean getAsBoolean() { return hasValidData();};
  };

  public static Vision Instance;
  public static boolean usingLimelight = false;

  int thingsUsingLimelight = 0;

  public Vision() 
  {
    VisionDriveCalculator.SetVisionReference(this);

    Instance = this;
  }

  @Override
  public void periodic() 
  {
    aprilTagID = (int)NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(-1.0);
    horizontalOffsetFromAprilTag = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    if (aprilTagID != -1)
    {
      CalculateTargetTransformInRobotSpace();
    }
    else
    {
      ClearAprilTagData();
    }

    LogData();
  }

  void CalculateTargetTransformInRobotSpace()
  {
    double[] vals = NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_robotspace").getDoubleArray(new double[6]);
    if(vals[0]!=0)
    {
      addVector3ToArrayList(new Vector3(vals[0], vals[1], vals[2]), aprilTagPosInRobotSpaceValues);
      addVector3ToArrayList(new Vector3(vals[3], vals[4], vals[5]), aprilTagRotInRobotSpaceValues);
      avgAprilTagPosInRobotSpace = getAverageOfArrayList(aprilTagPosInRobotSpaceValues);
      avgAprilTagRotInRobotSpace = getAverageOfArrayList(aprilTagRotInRobotSpaceValues);
    } 
    else 
    {
      System.out.println("Bad Data");
    }
  }

  void CalculateRobotPositionInFieldSpace()
  {
    double[] vals = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
    if (vals[0]!=0) {
      addVector3ToArrayList(new Vector3(vals[0], vals[1], vals[2]), robotPosInFieldSpaceValues);
      addVector3ToArrayList(new Vector3(vals[3], vals[4], vals[5]), robotRotInFieldSpaceValues);
      avgRobotPosInFieldSpace = getAverageOfArrayList(robotPosInFieldSpaceValues);
      avgAprilTagRotInRobotSpace = getAverageOfArrayList(robotRotInFieldSpaceValues);
    } 
    else 
    {
      System.out.println("Bad Data");
    }
    
  }

  void addVector3ToArrayList(Vector3 newVal, ArrayList<Vector3> arrayList)
  {
    arrayList.add(0, newVal);
    if (arrayList.size() > VisionConstants.VALUES_TO_AVERAGE) {arrayList.remove(arrayList.size() - 1); }
  }

  Vector3 getAverageOfArrayList(ArrayList<Vector3> arrayList)
  {
    Vector3 out = Vector3.zero;

    for (int i = 0; i < arrayList.size(); i++)
    {
      out = out.add(arrayList.get(i));
    }

    if (arrayList.size() == 0) { return Vector3.zero; }
    return out.divideBy(arrayList.size());
  }

  public boolean hasValidData()
  {
    return aprilTagID != -1;
  }

  void LogData()
  {
    SmartDashboard.putString("Detected AprilTagID", aprilTagID == -1 ? "None" : String.valueOf(aprilTagID));
    SmartDashboard.putString("AprilTag position in Robot Space", aprilTagID == -1 ? "N/A" : avgAprilTagPosInRobotSpace.toString(3));
    SmartDashboard.putString("AprilTag rotation in Robot Space", aprilTagID == -1 ? "N/A" : avgAprilTagRotInRobotSpace.toString(3));
    SmartDashboard.putString("AprilTag horizontal offset", aprilTagID == -1 ? "N/A" : String.valueOf(horizontalOffsetFromAprilTag));
    SmartDashboard.putNumber("Things using limelight", thingsUsingLimelight);
  }

  void ClearAprilTagData()
  {
    aprilTagPosInRobotSpaceValues.clear();
    aprilTagRotInRobotSpaceValues.clear();
  }

  public void OnTeleopDisable()
  {
    thingsUsingLimelight = 0;
    ToggleLimelightLight(false);
  }

  public void setUsingLimelight(boolean usingLimelight)
  {
    thingsUsingLimelight += usingLimelight ? 1 : -1;

    System.out.println(usingLimelight ? "New thing using limelight" : "Something stopped using limelight");

    if (thingsUsingLimelight <= 0) 
    { 
      thingsUsingLimelight = 0; 
      ToggleLimelightLight(false);
    }
    else if (thingsUsingLimelight > 0) { ToggleLimelightLight(true); } 
  }

  public void ToggleLimelightLight(boolean on)
  {
    System.out.println("Toggling Limelight " + (on ? "on" : "off"));

    usingLimelight = on;
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(on ? 3 : 1);
  }

  // Getter methods //
  public int AprilTagID() { return aprilTagID; }
  public Vector3 AprilTagPosInRobotSpace() { return avgAprilTagPosInRobotSpace; }
  public Vector3 AprilTagRotInRobotSpace() { return avgAprilTagRotInRobotSpace; }
  public Vector3 RobotPosInFieldSpace() { return avgRobotPosInFieldSpace; }
  public Vector3 RobotRotInFieldSpace() { return avgRobotRotInFieldSpace; }
  public double HorizontalOffsetFromAprilTag() { return horizontalOffsetFromAprilTag; }
}