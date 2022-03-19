// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hopper;

public class Funnel extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Hopper hopper;

  Timer funnelTimer = new Timer();

  double funnelDelay = 2.0;

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

      hopper.stop();

    } else if(Hopper.State == Hopper.STATES.POST_FUNNEL){

      if(funnelTimer.get() > funnelDelay){

        hopper.stop();
        funnelTimer.reset();
        Hopper.State = Hopper.STATES.STOP;

      } else{

        hopper.set(hopper.funnelSpeed);
        funnelTimer.start();
        
      }
    
    } else if (Hopper.State == Hopper.STATES.FUNNEL) {

      hopper.set(hopper.funnelSpeed);

    } else if (Hopper.State == Hopper.STATES.EXPEL) {

      hopper.set(this.hopper.expelSpeed);

    }

    if(Hopper.State != Hopper.STATES.POST_FUNNEL){

      funnelTimer.reset();

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
