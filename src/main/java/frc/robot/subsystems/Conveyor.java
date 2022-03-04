// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {

    
  public static enum STATES {LIFT, LOWER, STOP};
  
  public static STATES State = STATES.STOP;

  public WPI_TalonFX motor;
  public double liftSpeed = 0.2;
  public double lowerSpeed = -0.2;


  /** Creates a new Conveyor. */
  public Conveyor() {

    motor = new WPI_TalonFX(Constants.Conveyor.motor);

  }


  public void set(double speed) {

   this.motor.set(speed); 
  
  }


  public void stop() {

    this.set(0.0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
