// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  
  public static enum STATES {FUNNEL, POST_FUNNEL, EXPEL, STOP};
  
  public static STATES State = STATES.STOP;

  WPI_TalonFX leftMotor;
  WPI_TalonFX rightMotor;

  public double funnelSpeed = 0.6;
  public double expelSpeed = -0.6;


  /** Creates a new Hopper. */
  public Hopper() {

    this.leftMotor = new WPI_TalonFX(Constants.Hopper.leftMotor);
    this.rightMotor = new WPI_TalonFX(Constants.Hopper.rightMotor);

  }


  public void set(double speed) {

    this.leftMotor.set(-speed);
    this.rightMotor.set(speed / 2.0);

  }


  public void stop() {

    this.set(0.0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
