// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public static enum STATES {ARMUP, ARMDOWN, STOP};
  
  public static STATES State = STATES.STOP;

  public WPI_TalonFX motor;
  public double armUpSpeed = 0.4;
  public double armDownSpeed = -0.4;
  
  public Climber() {
    this.motor = new WPI_TalonFX(Constants.Climber.motor);

    this.motor.setNeutralMode(NeutralMode.Brake);
  }

  public void set(double power) {
    this.motor.set(power);
  }

  public void stop() {
    this.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
