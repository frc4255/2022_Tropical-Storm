// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  public static enum STATES {INTAKE, EXPEL, STOP};

  public static STATES State = STATES.STOP;

  public double intakeSpeed = 0.5;
  public double expelSpeed = -0.5;

  WPI_TalonFX intakeMotor;

  Solenoid intakeArm;

  public boolean out = true;
  public boolean in = false;


  public Intake() {

    this.intakeMotor = new WPI_TalonFX(Constants.Intake.motor);

    this.intakeArm = new Solenoid(PneumaticsModuleType.REVPH, Constants.Intake.arm);

    this.intakeMotor.setInverted(TalonFXInvertType.Clockwise);

  }


  public void set(double speed){

    this.intakeMotor.set(speed);

  }

  public void setArm(boolean mode){

    intakeArm.set(mode);

  }

  
  public void stop(){

    this.set(0.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
