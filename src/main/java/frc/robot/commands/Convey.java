// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hopper;

public class Convey extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Conveyor conveyor;

  Timer expelTimer = new Timer();
  Timer shootTimer = new Timer();

  boolean expelling = false;
  boolean shooting = true;

  double expelTime = 3.0;
  double shootTime = 3.0;

  /** Creates a new Lift. */
  public Convey(Conveyor m_conveyor) {
    addRequirements(m_conveyor);

    this.conveyor = m_conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Conveyor.State == Conveyor.STATES.STOP){

      this.conveyor.stop();

    } else if(Conveyor.State == Conveyor.STATES.INDEX){


      if(expelling || shooting){

        if(expelTimer.get() > expelTime){

          expelling = false;
          expelTimer.reset();

        }

        if(shootTimer.get() > shootTime){

          shooting = false;
          shootTimer.reset();

        }

      //-----------------------------------------------------------------
      } else if(this.conveyor.hasCorrectBall() == 1){

          if(this.conveyor.hasSecondBall()){

            this.conveyor.set(this.conveyor.lowerSpeed);
            Intake.State = Intake.STATES.EXPEL;
            Hopper.State = Hopper.STATES.EXPEL;
            expelling = true;
            expelTimer.start();

          //-----------------------------------------------------------------
          } else {
  
            this.conveyor.set(this.conveyor.liftSpeed);
            Shooter.State = Shooter.STATES.EXPEL;
            shooting = true;

          }
        
      //-----------------------------------------------------------------
      } else {

        if(this.conveyor.hasSecondBall()){

          this.conveyor.stop();
        
        //-----------------------------------------------------------------
        } else{

          this.conveyor.set(this.conveyor.liftSpeed);

        }

      }

    } else if(Conveyor.State == Conveyor.STATES.FEED){

      this.conveyor.set(this.conveyor.liftSpeed);

    }

    this.conveyor.displayRawColors();
    this.conveyor.hasSecondBall();

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
