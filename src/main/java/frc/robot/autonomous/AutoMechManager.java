// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MechManager;
import frc.robot.subsystems.Shooter;

public class AutoMechManager extends CommandBase {
  /** Creates a new AutoMechManager. */

  double autoShootLimit = 4.0;
  Timer shootTimer = new Timer();
  
  double autoIntakeLimit = 2.0;
  Timer intakeTimer = new Timer();

  public AutoMechManager(MechManager m_mechManager) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_mechManager);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(MechManager.State == MechManager.AUTO_STATES.SHOOT && shootTimer.get() >= autoShootLimit){

      MechManager.State = MechManager.AUTO_STATES.NONE;
      shootTimer.reset();
      Shooter.State = Shooter.STATES.STOP;
      Hopper.State = Hopper.STATES.STOP;

    } else if(MechManager.State == MechManager.AUTO_STATES.SHOOT){

      shootTimer.start();
      Shooter.State = Shooter.STATES.SHOOT;
      Hopper.State = Hopper.STATES.FUNNEL;

    }


    if(MechManager.State == MechManager.AUTO_STATES.INTAKE && intakeTimer.get() >= autoIntakeLimit){

      MechManager.State = MechManager.AUTO_STATES.NONE;
      intakeTimer.reset();
      Intake.State = Intake.STATES.STOP;
      Hopper.State = Hopper.STATES.STOP;

    } else if(MechManager.State == MechManager.AUTO_STATES.INTAKE){

      intakeTimer.start();
      Intake.State = Intake.STATES.INTAKE;
      Hopper.State = Hopper.STATES.FUNNEL;

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
