// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Conveyor.SUBSTATES;

public class Convey extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Conveyor conveyor;

  Timer LOAD_TIMER = new Timer();
  Timer VOMIT_TIMER = new Timer();
  Timer MISFIRE_TIMER = new Timer();

  double LOAD_TIME = 0.3;
  double VOMIT_TIME = 1.0;
  double MISFIRE_TIME = 0.5;

  /** Creates a new Lift. */
  public Convey(Conveyor m_conveyor) {
    addRequirements(m_conveyor);

    conveyor = m_conveyor;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean SB = Conveyor.hasSecondBall();
    boolean CB = Conveyor.hasCorrectBall() == 0;
    boolean WB = Conveyor.hasCorrectBall() == 1;
  
    if(Conveyor.State == Conveyor.STATES.STOP){
      conveyor.stop();

      Conveyor.Substate = SUBSTATES.IDLE;
      LOAD_TIMER.reset();
      VOMIT_TIMER.reset();
      MISFIRE_TIMER.reset();

    } else if(Conveyor.State == Conveyor.STATES.INDEX){

      if(Conveyor.Substate == SUBSTATES.IDLE){

        conveyor.stop();

        LOAD_TIMER.reset();
        VOMIT_TIMER.reset();
        MISFIRE_TIMER.reset();

        if(!SB && CB){
          Conveyor.Substate = SUBSTATES.STORE;
        } else if(SB && CB){
          Conveyor.Substate = SUBSTATES.LOAD;
        } else if(!SB && WB){
          Conveyor.Substate = SUBSTATES.MISFIRE;
        } else if(SB && WB){
          Conveyor.Substate = SUBSTATES.VOMIT;
        }

      } else if(Conveyor.Substate == SUBSTATES.STORE){

        if(SB && !CB){
          Conveyor.Substate = SUBSTATES.IDLE;
          Hopper.State = Hopper.STATES.STOP;
        } else{
          Hopper.State = Hopper.STATES.FUNNEL;
          conveyor.set(conveyor.liftSpeed);
          Conveyor.ballsInConveyor = 1;
        }

      } else if(Conveyor.Substate == SUBSTATES.LOAD){

        if(LOAD_TIMER.get() > LOAD_TIME){
          Conveyor.Substate = SUBSTATES.IDLE;
          Hopper.State = Hopper.STATES.STOP;
        } else{
          Hopper.State = Hopper.STATES.FUNNEL;
          conveyor.set(conveyor.liftSpeed);
          Conveyor.ballsInConveyor = 2;
          LOAD_TIMER.start();
        }

      } else if(Conveyor.Substate == SUBSTATES.MISFIRE){

        if(MISFIRE_TIMER.get() > MISFIRE_TIME){
          Conveyor.Substate = SUBSTATES.IDLE;
          Hopper.State = Hopper.STATES.STOP;
          Shooter.State = Shooter.STATES.STOP;
        } else{
          conveyor.set(conveyor.liftSpeed * 3.0);
          Shooter.State = Shooter.STATES.EXPEL;
          Hopper.State = Hopper.STATES.FUNNEL;
          Conveyor.ballsInConveyor = 0;
          MISFIRE_TIMER.start();
        }

      } else if(Conveyor.Substate == SUBSTATES.VOMIT){

        if(VOMIT_TIMER.get() > VOMIT_TIME){
          Conveyor.Substate = SUBSTATES.IDLE;
          Hopper.State = Hopper.STATES.STOP;
          Intake.State = Intake.STATES.STOP;
        } else if(CB){
          conveyor.set(0.0);
        } else{
          conveyor.set(conveyor.lowerSpeed);
          Hopper.State = Hopper.STATES.EXPEL;
          Intake.State = Intake.STATES.EXPEL;
          Conveyor.ballsInConveyor = 1;
          VOMIT_TIMER.start();
        }
      
      }
    } else if(Conveyor.State == Conveyor.STATES.FEED){
      conveyor.set(conveyor.liftSpeed);
      
      Conveyor.Substate = SUBSTATES.IDLE;
      Conveyor.ballsInConveyor = 0;
      LOAD_TIMER.reset();
      VOMIT_TIMER.reset();
      MISFIRE_TIMER.reset();
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    conveyor.displayConveyorValues();
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
