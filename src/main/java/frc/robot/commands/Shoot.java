// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.subsystems.Shooter;

public class Shoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /** Creates a new Shoot. */

  Shooter shooter;
  TabData shooterData = Shuphlebord.shooterData;

  double kp = 0.0;
  double ki = 0.00025;
  double kd = 0.0;
  double setpoint = 3560.0;
  double power = 0.0;
  double tolerance = 100.0;
  PIDController controller = new PIDController(kp, ki, kd);

  boolean lastPressed = false;

  public Shoot(Shooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
    shooter = m_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shooterData.updateEntry("kP", kp);
    shooterData.updateEntry("kI", ki);
    shooterData.updateEntry("kD", kd);
    shooterData.updateEntry("Setpoint", setpoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double adjustedkp = shooterData.getEntryData("kP").getDouble();
    double adjustedki = shooterData.getEntryData("kI").getDouble();
    double adjustedkd = shooterData.getEntryData("kD").getDouble();
    double adjustedSetpoint = shooterData.getEntryData("Setpoint").getDouble();

    if(kd != adjustedkp || ki != adjustedki || kd != adjustedkd || setpoint != adjustedSetpoint){
      kp = adjustedkp;
      ki = adjustedki;
      kd = adjustedkd;

      controller.setPID(kp, ki, kd);
      setpoint = adjustedSetpoint;
    }

    if(RobotContainer.controller.getLeftBumper()){

      if(!lastPressed){
        lastPressed = true;
      }

      controller.setSetpoint(setpoint);

      double rpm = shooter.getRPM();
      double power = controller.calculate(rpm);
      shooterData.updateEntry("RPM", rpm);
      shooterData.updateEntry("Setpoint", setpoint);
      shooterData.updateEntry("Power", power);

      shooter.set(power);

    } else{

      lastPressed = false;
      shooter.stop();
      System.out.println("Stopped!");

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
