// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.TalonFXEncoder;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */

  public static enum STATES {FENDER_SHOOT, EXPEL, STOP, VISION_SHOOT, IDLE};

  public static STATES State = STATES.STOP;

  WPI_TalonFX leftMotor;
  WPI_TalonFX rightMotor;

  TalonFXEncoder leftEncoder;
  TalonFXEncoder rightEncoder;

  public static double shootSetpoint = 1700.0;
  public static double idleSetpoint = 2150.0;
  public static double expelSetpoint = 600.0;

  public Shooter() {

    leftMotor = new WPI_TalonFX(Constants.Shooter.leftMotor);
    rightMotor = new WPI_TalonFX(Constants.Shooter.rightMotor);

    rightMotor.follow(leftMotor);

    leftMotor.setInverted(TalonFXInvertType.Clockwise);
    rightMotor.setInverted(TalonFXInvertType.OpposeMaster);

    leftEncoder = new TalonFXEncoder(leftMotor);
    rightEncoder = new TalonFXEncoder(rightMotor);
  }

  public void set(double speed){
    double limit = 1.0;

    if(limit < speed){
      speed = limit;
    } else if(speed < 0.0){
      speed = 0.0;
    }

    leftMotor.set(speed);
  }

  public void setVoltage(double voltage){

    leftMotor.setVoltage(voltage);
    rightMotor.setVoltage(voltage);

  }

  public void stop(){
    set(0.0);
  }

  public double getRPM(){

    double rpm = 0.0;

    rpm += leftEncoder.getRPM();
    rpm += rightEncoder.getRPM();
    rpm /= 2.0;

    return rpm;

  }

  public double getTargetRPM(){

    Map<Double, Double> rpms = new HashMap<>();

    // Should be put as rpms.put(Distance in inches, rpm)
    // This is the data table for the distance v. rpm data
    rpms.put(29.0, 1700.0);
    rpms.put(51.0, 1830.0);
    rpms.put(75.0, 1980.0);
    rpms.put(100.0, 2150.0);
    rpms.put(125.0, 2210.0);


    // Linear Interpolation Stuff
    Double[] distances = rpms.keySet().toArray(Double[]::new);

    Arrays.sort(distances);

    // Distance from target
    double dist = Limelight.getDistance();

    if(dist == 0.0){
      return 0.0;
    }

    int index = distances.length;

    // Check to see which set of distances our current distance falls between
    for(int i = 0; i < distances.length; i++){
      if(dist < distances[i]){
        index = i;
        break;
      }
    }

    if(dist > distances[distances.length - 1]){
      return rpms.get(125.0);
    }

    // Calculate linear function with x: distance and y: RPM, then plug in LL distance
    double[] point1 = {distances[index - 1], rpms.get(distances[index - 1])};
    double[] point2 = {distances[index], rpms.get(distances[index])};

    double m = (point2[1] - point1[1]) / (point2[0] - point1[0]);
    double b = point1[1];

    double rpm = m * (dist - point1[0]) + b;

    return rpm;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
