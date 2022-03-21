// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Conveyor.INDEXING_SUBSTATES;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Hopper;

public class Convey extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Conveyor conveyor;

  Timer expelTimer = new Timer();
  Timer shootTimer = new Timer();
  Timer shiftTimer = new Timer();
  Timer colorCheckTimer = new Timer();
  Timer colorCheckTimer2 = new Timer();
  Timer secPosTimer = new Timer();

  double expelLimit = 0.5;
  double shootLimit = 1.2;
  double shiftLimit = 0.55;
  double colorCheckLimit = 0.45;
  double secPosLimit = 0.3;

  boolean resetCCT2 = true;
  boolean last1stBall = false;

  INDEXING_SUBSTATES dummyLastSubstate = INDEXING_SUBSTATES.NONE;

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

    resetCCT2 = true;
  
    if(Conveyor.State == Conveyor.STATES.STOP){
      //System.out.println("Conveyor Stopped!");
      conveyor.stop();
      Conveyor.Substate = INDEXING_SUBSTATES.NONE;

    } else if(Conveyor.State == Conveyor.STATES.INDEX){

      if(Conveyor.Substate != INDEXING_SUBSTATES.NONE){
        //System.out.println("Shooting or Expelling!");

        if(expelTimer.get() > expelLimit || Conveyor.hasCorrectBall() == 0){
          Conveyor.Substate = INDEXING_SUBSTATES.NONE;
          expelTimer.reset();
          Intake.State = Intake.STATES.STOP;
          Hopper.State = Hopper.STATES.STOP;
        }

        if(shootTimer.get() > shootLimit){
          Conveyor.Substate = INDEXING_SUBSTATES.NONE;
          shootTimer.reset();
          Shooter.State = Shooter.STATES.STOP;
        }

        if(shiftTimer.get() > shiftLimit){
          Conveyor.Substate = INDEXING_SUBSTATES.NONE;
          Hopper.State = Hopper.STATES.STOP;
          shiftTimer.reset();
          conveyor.stop();
        }

      //-----------------------------------------------------------------
      } else if(Conveyor.hasCorrectBall() == 1){
        //System.out.println("Has Incorrect Ball");

        if(Conveyor.hasSecondBall()){
          // Don't mess with:
          resetCCT2 = false;

          if(colorCheckTimer2.get() > colorCheckLimit){
            //conveyor.set(conveyor.lowerSpeed);
            Intake.State = Intake.STATES.EXPEL;
            Hopper.State = Hopper.STATES.EXPEL;
            Conveyor.Substate = INDEXING_SUBSTATES.EXPELLING;
            expelTimer.start();

          } else{
            colorCheckTimer2.start();
          }

        //-----------------------------------------------------------------
        } else {
          resetCCT2 = false;

          if(colorCheckTimer2.get() > colorCheckLimit){
            conveyor.set(conveyor.liftSpeed * 2.0);
            Shooter.State = Shooter.STATES.EXPEL;
            Conveyor.Substate = INDEXING_SUBSTATES.SHOOTING;
            shootTimer.start();
          
          } else{
            colorCheckTimer2.start();
          }
        }
      
        colorCheckTimer.reset();
      //-----------------------------------------------------------------
      } else if(Conveyor.hasCorrectBall() == 0){
        //System.out.println("Conveyor has correct or no ball!");

        if(Conveyor.hasSecondBall()){
          System.out.println("Lifting!");

          if(colorCheckTimer.get() > colorCheckLimit){
            conveyor.set(conveyor.liftSpeed);

            if(secPosTimer.get() > secPosLimit){
              Conveyor.Substate = INDEXING_SUBSTATES.SHIFTING;
              Hopper.State = Hopper.STATES.FUNNEL;
              secPosTimer.reset();
              shiftTimer.start();
              colorCheckTimer.reset();

            } else{
              secPosTimer.start();
            }
          
          } else{
            colorCheckTimer.start();
          }

          last1stBall = false;
        //-----------------------------------------------------------------
        } else{
          conveyor.set(conveyor.liftSpeed);
          last1stBall = true;
        }
      
      //-----------------------------------------------------------------
      } else{
          conveyor.stop();
          colorCheckTimer.reset();
      }
    
    } else if(Conveyor.State == Conveyor.STATES.FEED){
      conveyor.set(conveyor.liftSpeed);
    }




    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    if(Conveyor.State != Conveyor.STATES.INDEX){

      Conveyor.lastSubstate = INDEXING_SUBSTATES.NONE;
      Conveyor.Substate = INDEXING_SUBSTATES.NONE;

    } else if(Conveyor.Substate != dummyLastSubstate){

      Conveyor.lastSubstate = dummyLastSubstate;

    }

    if(resetCCT2){

      colorCheckTimer2.reset();
      
    }

    dummyLastSubstate = Conveyor.Substate;

    conveyor.displayConveyorValues();

    System.out.println(", DUMMY: " + dummyLastSubstate.toString());

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
