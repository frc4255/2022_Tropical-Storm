// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class Suck extends CommandBase {
  /** Creates a new Suck. */
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Intake intake;

  public Suck(Intake m_intake) {
    addRequirements(m_intake);

    this.intake = m_intake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Intake.State == Intake.STATES.STOP) {

      this.intake.stop();
      this.intake.setArm(this.intake.in);

    } else if (Intake.State == Intake.STATES.INTAKE) {

      this.intake.set(this.intake.intakeSpeed);
      this.intake.setArm(this.intake.out);

    } else if (Intake.State == Intake.STATES.EXPEL) {

      this.intake.set(this.intake.expelSpeed);
      this.intake.setArm(this.intake.out);

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
