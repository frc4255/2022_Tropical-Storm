// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MechManager;
import frc.robot.subsystems.MechManager.AUTO_STATES;

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

      intake.stop();
      intake.setArm(intake.in);

    } else if (Intake.State == Intake.STATES.INTAKE) {

      if(MechManager.State == AUTO_STATES.ENABLE_INTAKE || (Conveyor.ballsInConveyor != 2)){
        intake.set(intake.intakeSpeed);
        intake.setArm(intake.out);
        System.out.println("  Intake Open!  ");
      } else{
        intake.stop();
        intake.setArm(intake.in);
        System.out.println("  Intake Blocking!  ");
      }

    } else if (Intake.State == Intake.STATES.EXPEL) {

      intake.set(intake.expelSpeed);
      intake.setArm(intake.out);

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
