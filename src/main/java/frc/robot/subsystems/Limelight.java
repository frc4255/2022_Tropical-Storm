// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  static TabData limelightData = Shuphlebord.limelightData;

  public static double tx;
  public static double ty;
  public static double ta;
  public static double thor;
  public static double tv;

  public static double limelightMountHeight = 39.5; //height of mount
  public static double targetHeight = 104.0; //height of target
  public static double limelightMountAngle = 50.0; //angle of limelight mount
  public static boolean ALIGNED = false;



  public Limelight() {
  }

  static void reset(){
    tx = 0.0;
    ty = 0.0;
    ta = 0.0;
  }

  public static double getTx() {
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    return tx;
  }

  public static double getTy(){
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    return ty;
  }


  /**
   * Will return the distance from the robot to the goal
   * 
   * @return Distance in inches
   */
  public static double getDistance(){

    if(hasTarget()){
      double angleToTarget = limelightMountAngle + getTy();

      double heightDiff = targetHeight - limelightMountHeight;

      double distance = heightDiff / (Math.tan(Math.toRadians(angleToTarget)) * Math.cos(Math.toRadians(getTx())));

      return distance;
    }
    return 0.0;
  }


  public static boolean hasTarget(){
    return tv == 1.0;
  }


  public static void update(){
    tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
    thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0.0);
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);

    limelightData.updateEntry("Distance Estimate", getDistance());
    limelightData.updateEntry("tx", tx);
    limelightData.updateEntry("ty", ty);
    limelightData.updateEntry("ALIGNED", ALIGNED);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    update();
  }
}