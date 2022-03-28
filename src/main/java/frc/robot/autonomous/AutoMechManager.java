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

  double autoShootLimit = 2.2;
  Timer shootTimer = new Timer();
  
  double autoIntakeLimit = 0.25;
  Timer intakeTimer = new Timer();

  boolean finished;

  MechManager.AUTO_STATES state;

  public AutoMechManager(MechManager.AUTO_STATES auto_state) {
    // Use addRequirements() here to declare subsystem dependencies.
    state = auto_state;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    MechManager.State = state;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    

    if(MechManager.State == MechManager.AUTO_STATES.FENDER_SHOOT && shootTimer.get() >= autoShootLimit){

      MechManager.State = MechManager.AUTO_STATES.NONE;
      shootTimer.reset();
      Shooter.State = Shooter.STATES.STOP;
      Hopper.State = Hopper.STATES.STOP;
      finished = true;

    } else if(MechManager.State == MechManager.AUTO_STATES.FENDER_SHOOT){

      shootTimer.start();
      Shooter.State = Shooter.STATES.FENDER_SHOOT;
      Hopper.State = Hopper.STATES.FUNNEL;
      finished = false;

    }


    if(MechManager.State == MechManager.AUTO_STATES.INTAKE && intakeTimer.get() >= autoIntakeLimit){

      MechManager.State = MechManager.AUTO_STATES.NONE;
      intakeTimer.reset();
      Intake.State = Intake.STATES.STOP;
      Hopper.State = Hopper.STATES.STOP;
      finished = true;

    } else if(MechManager.State == MechManager.AUTO_STATES.INTAKE){

      intakeTimer.start();
      Intake.State = Intake.STATES.INTAKE;
      Hopper.State = Hopper.STATES.FUNNEL;
      finished = false;

    }


    if(MechManager.State == MechManager.AUTO_STATES.ENABLE_INTAKE){

      Intake.State = Intake.STATES.INTAKE;
      Hopper.State = Hopper.STATES.FUNNEL;
      finished = true;
      
    }


    if(MechManager.State == MechManager.AUTO_STATES.DISABLE_INTAKE){

      Intake.State = Intake.STATES.STOP;
      Hopper.State = Hopper.STATES.STOP;
      finished = true;
      
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
