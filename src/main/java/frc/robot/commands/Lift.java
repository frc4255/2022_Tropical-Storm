// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class Lift extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Conveyor conveyor;

  /** Creates a new Lift. */
  public Lift(Conveyor m_conveyor) {
    addRequirements(m_conveyor);

    this.conveyor = m_conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Conveyor.State == Conveyor.STATES.STOP) {

      this.conveyor.stop();

    } else if (Conveyor.State == Conveyor.STATES.LIFT) {

      this.conveyor.set(this.conveyor.liftSpeed);

    } else if (Conveyor.State == Conveyor.STATES.LOWER) {

      this.conveyor.set(this.conveyor.lowerSpeed);

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
