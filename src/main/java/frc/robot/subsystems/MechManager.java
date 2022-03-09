// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MechManager extends SubsystemBase {
  /** Creates a new MechManager. */

  public static enum AUTO_STATES {NONE, SHOOT, INTAKE}

  public static AUTO_STATES State = AUTO_STATES.NONE;

  public MechManager() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
