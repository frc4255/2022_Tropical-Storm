// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.LEDs;

public class ShineBrightLikeADiamond extends CommandBase {
  /** Creates a new ShineBrightLikeADiamond. */

  LEDs leds;

  public ShineBrightLikeADiamond(LEDs m_leds) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_leds);

    leds = m_leds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    leds.set(LEDs.STATES[Conveyor.ballsInConveyor()]);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
