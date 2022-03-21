// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Climber climber;

  Timer stopperTimer = new Timer();
  double stopDelay = 0.25;

  /** Creates a new Climb. */
  public Climb(Climber m_climber) {
    addRequirements(m_climber);

    this.climber = m_climber;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Climber.State == Climber.STATES.STOP) {

      climber.stop();
      climber.hold();
      stopperTimer.reset();

    } else if (Climber.State == Climber.STATES.ARMUP) {

      if(stopperTimer.get() > stopDelay){

        climber.set(climber.armUpSpeed);

      } else{

        climber.release();
        stopperTimer.start();
        
      }

    } else if (Climber.State == Climber.STATES.ARMDOWN) {

      if(stopperTimer.get() > stopDelay){

        climber.set(climber.armDownSpeed);

      } else{

        climber.release();
        stopperTimer.start();
        
      }

    } else if(Climber.State == Climber.STATES.RELEASE){

      climber.release();

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
