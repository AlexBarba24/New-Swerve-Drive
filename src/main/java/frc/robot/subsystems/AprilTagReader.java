// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AprilTagReader extends SubsystemBase {

  private static PIDController PID = new PIDController(Constants.OperatorConstants.AimPID[0], Constants.OperatorConstants.AimPID[1], Constants.OperatorConstants.AimPID[2]);
  
  static PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  static private NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/Microsoft_LifeCam_HD-3000");
  static private NetworkTableEntry tx = table.getEntry("targetYaw");
  static private NetworkTableEntry ty = table.getEntry("targetPitch");
  static private NetworkTableEntry ta = table.getEntry("targetArea");

  static private double x = tx.getDouble(0.0);
  static private double y = ty.getDouble(0.0);
  static private double area = ta.getDouble(0.0);


  public static double getX() {
      tx = table.getEntry("targetYaw");
      x = tx.getDouble(0.0);
      return x;
  }

  public static double getY() {
      ty = table.getEntry("targetPitch");
      y = ty.getDouble(0.0);
      return y;
  }

  public static double getA() {
      ta = table.getEntry("targetArea");
      area = ta.getDouble(0.0);
      return area;
  }

  public static boolean getV() {
      tx = table.getEntry("targetYaw");
      x = tx.getDouble(0.0);
      ty = table.getEntry("targetPitch");
      y = ty.getDouble(0.0);
      if (x != 0 || y != 0)
          return true;
      else 
          return false;    
  }
  /**
   * 
   * @return a calculates meters per second for the robot to rotate in order to aim at an april tag
   */
  public static double aim(){
    return PID.calculate(getX(), 0)*1;
  }

  /** Creates a new AprilTagReader. */
  public AprilTagReader() {
    camera.setPipelineIndex(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSmartDashboard();

  }

  public void updateSmartDashboard(){
    SmartDashboard.putNumber("TURNING VALUE", aim());
  }
}
