// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class Funnel extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Hopper hopper;

  /** Creates a new Funnel. */
  public Funnel(Hopper m_hopper) {
    addRequirements(m_hopper);

    this.hopper = m_hopper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Hopper.State == Hopper.STATES.STOP) {

      this.hopper.stop();

    } else if (Hopper.State == Hopper.STATES.FUNNEL) {

      this.hopper.set(this.hopper.funnelSpeed);

    } else if (Hopper.State == Hopper.STATES.EXPEL) {

      this.hopper.set(this.hopper.expelSpeed);

    }
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
