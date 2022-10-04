// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.TalonFXEncoder;

import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */

  public WPI_TalonFX leftMotor;
  public WPI_TalonFX rightMotor;

  public TalonFXEncoder leftEncoder;
  public TalonFXEncoder rightEncoder;

  public DigitalInput leftLimitSensor;
  public DigitalInput rightLimitSensor;

  Solenoid arm;

  public double armUpSpeed = -0.8;
  public double armDownSpeed = 0.8;

  public double max = 142502.0;
  public double min = 0.0;

  boolean holding = false;
  
  public Climber() {
    leftMotor = new WPI_TalonFX(Constants.Climber.leftMotor);
    rightMotor = new WPI_TalonFX(Constants.Climber.rightMotor);

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    leftEncoder = new TalonFXEncoder(leftMotor);
    rightEncoder = new TalonFXEncoder(rightMotor);

    // Set distance per rotation of the motor here
    // factor should initially be set to meters / rotation
    double factor = (Math.PI / ((54.0 / 8.0) * (48.0 / 24.0))) * 0.0254;
    factor /= 2048.0;

    leftEncoder.setDistancePerPulse(factor);
    rightEncoder.setDistancePerPulse(factor);

    leftEncoder.reset();
    rightEncoder.reset();

    leftLimitSensor = new DigitalInput(0);
    rightLimitSensor = new DigitalInput(1);

    arm = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climber.arm);
  }
  
  public void setLeft(double power) {
    leftMotor.set(power);
  }

  public void setRight(double power){
    rightMotor.set(power*0.9);
  }

  public void stopLeft() {
    setLeft(0.0);
  }

  public void stopRight(){
    setRight(0.0);
  }

  public void setLeftVoltage(double power){
    leftMotor.setVoltage(power);
  }

  public void setRightVoltage(double power){
    rightMotor.setVoltage(power);
  }

  public double getLeftPower(){
    return leftMotor.get();
  }

  public double getRightPower(){
    return rightMotor.get();
  }

  public void extendArm(){
    arm.set(true);
  }

  public void retractArm(){
    arm.set(false);
  }

  /**
   * @return Distance traveled by motor in meters
   */
  public double getLeftDistance(){

    return leftEncoder.getDistance();
  
  }

  /**
   * @return Distance traveled by motor in meters
   */
  public double getRightDistance(){

    return rightEncoder.getDistance();

  }

  public boolean atLeftLimit(){
    //return !leftLimitSensor.get();
    return false;
  }

  public boolean atRightLimit(){
    //return !rightLimitSensor.get();
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
