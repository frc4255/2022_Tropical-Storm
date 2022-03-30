// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public static enum STATES {ARMUP, ARMDOWN, STOP, RELEASE};
  
  public static STATES State = STATES.STOP;

  public WPI_TalonFX motor;
  public double armUpSpeed = -0.8;
  public double armDownSpeed = 0.8;

  public double max = 142502.0;
  public double min = 0.0;

  public Solenoid stopper;
  boolean holding = false;
  
  public Climber() {/*
    motor = new WPI_TalonFX(Constants.Climber.motor);

    motor.setNeutralMode(NeutralMode.Brake);

    stopper = new Solenoid(PneumaticsModuleType.REVPH, Constants.Climber.stopper);*/
  }
  /*

  public void set(double power) {
    motor.set(power);
  }

  public void stop() {
    set(0.0);
  }

  public void hold(){
    stopper.set(holding);
  }

  public void release(){
    stopper.set(!holding);
  }

  public double readEncoder(){
    return motor.getSelectedSensorPosition();
  }

  public void resetEncoder(){
    motor.setSelectedSensorPosition(0.0);
  }*/

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
