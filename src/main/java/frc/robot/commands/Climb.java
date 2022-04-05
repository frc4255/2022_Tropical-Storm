// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Climber climber;

  double scaler = 5.0 / 7.0;

  /** Creates a new Climb. */
  public Climb(Climber m_climber) {
    addRequirements(m_climber);

    climber = m_climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftY = RobotContainer.mechController.getLeftY();
    double rightY = -RobotContainer.mechController.getRightY();
  
    if(leftY > 0.5){

      climber.setLeft(climber.armUpSpeed);

    } else if(leftY < -0.5){

      climber.setLeft(climber.armDownSpeed);

    } else{

      climber.stopLeft();

    }

    if(rightY > 0.5){

      climber.setRight(climber.armUpSpeed);

    } else if(rightY < -0.5){

      climber.setRight(climber.armDownSpeed);

    } else{

      climber.stopRight();

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
