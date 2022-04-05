// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.subsystems.Climber;

public class Climb extends CommandBase {
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Climber climber;

  TabData climberData = Shuphlebord.climberData;

  double scaler = 5.0 / 7.0;

  double maxVelocity = 1.0;
  double maxAcceleration = 1.0;
  Constraints constraints = new Constraints(maxVelocity, maxAcceleration);

  double setpoint = 0.0;
  double step = 0.01;

  double leftKP = 0.0;
  double leftKI = 0.0;
  double leftKD = 0.0;
  ProfiledPIDController leftController = new ProfiledPIDController(leftKP, leftKI, leftKD, constraints);

  double rightKP = 0.0;
  double rightKI = 0.0;
  double rightKD = 0.0;
  ProfiledPIDController rightController = new ProfiledPIDController(rightKP, rightKI, rightKD, constraints);

  /** Creates a new Climb. */
  public Climb(Climber m_climber) {
    addRequirements(m_climber);

    climber = m_climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    leftController.setGoal(setpoint);
    rightController.setGoal(setpoint);

    climberData.updateEntry("Setpoint", setpoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double joystickValue = RobotContainer.mechController.getLeftY();
  
    if(joystickValue > 0.5){

      setpoint += step;
      

    } else if(joystickValue < -0.5){

      setpoint -= step;

    } 

    double leftPower = leftController.calculate(climber.getLeftDistance());
    double rightPower = rightController.calculate(climber.getRightDistance());

    climber.setLeft(leftPower);
    climber.setRight(rightPower);

    climberData.updateEntry("Setpoint", setpoint);
    climberData.updateEntry("Left Pos", climber.getLeftDistance());
    climberData.updateEntry("Right Pos", climber.getRightDistance());
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
