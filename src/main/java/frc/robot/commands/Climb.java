// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Climber climber;

  TabData climberData = Shuphlebord.climberData;

  double threshold = 0.5;

  boolean leftOverextended = false;
  boolean rightOverextended = false;

  /** Creates a new Climb. */
  public Climb(Climber m_climber) {
    addRequirements(m_climber);

    climber = m_climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftY  = -RobotContainer.mechController.getRightY();
    double rightY = -RobotContainer.mechController.getLeftY();
    double leftP = -climber.getLeftPower();
    double rightP = -climber.getRightPower();
    boolean extend = RobotContainer.mechController.getRightBumper();
    boolean retract = RobotContainer.mechController.getLeftBumper();

    // CHECK FOR OVER-EXTENSION
    if(climber.atLeftLimit() && leftP > 0.0){
      leftOverextended = true;
      climber.stopLeft();
    } else if(climber.atLeftLimit() && leftP < 0.0){
      leftOverextended = false;
    }

    if(climber.atRightLimit() && rightP > 0.0){
      rightOverextended = true;
      climber.stopRight();
    } else if(climber.atRightLimit() && rightP < 0.0){
      rightOverextended = false;
    }


    // MANAGE CLIMBER SPEED
    if(leftOverextended) {

      if (leftY < -threshold) {

        climber.setLeft(climber.armDownSpeed);

      } else if (leftY > threshold) {

        climber.stopLeft();

      } else {

        climber.stopLeft();

      }

    } else{
      if(leftY < -threshold){
        climber.setLeft(climber.armDownSpeed);
      } else if(leftY > threshold){
        climber.setLeft(climber.armUpSpeed);
      } else{
        climber.stopLeft();
      }
    }

    if(rightOverextended){
      if(rightY < -threshold){
        climber.setRight(climber.armDownSpeed);
      } else if(rightY > threshold){
        climber.stopRight();
      } else{
        climber.stopRight();
      }
    } else{
      if(rightY < -threshold){
        climber.setRight(climber.armDownSpeed);
      } else if(rightY > threshold){
        climber.setRight(climber.armUpSpeed);
      } else{
        climber.stopRight();
      }
    }


    // PNEUMATIC ARM MOVEMENT
    if(extend){
      climber.extendArm();
    } else if(retract){
      climber.retractArm();
    }

    climberData.updateEntry("Left Y", leftY);
    climberData.updateEntry("Right Y", rightY);
    climberData.updateEntry("Left Power", leftP);
    climberData.updateEntry("Right Power", rightP);
    climberData.updateEntry("Left Overextended", leftOverextended);
    climberData.updateEntry("Right Overextended", rightOverextended);
    
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
