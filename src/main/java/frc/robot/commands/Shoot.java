// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.MechManager;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.MechManager.AUTO_STATES;

public class Shoot extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  /** Creates a new Shoot. */

  Shooter shooter;
  TabData shooterData = Shuphlebord.shooterData;

  
  double kp = 0.4;
  double ki = 0.1;
  double kd = 0.014;
  double power = 0.0;
  double tolerance = 50.0;
  PIDController controller = new PIDController(kp, ki, kd);

  double ks = 0.89497;
  double kv = 0.12483;
  double ka = 0.0094741;
  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ks, kv, ka);

  Timer toleranceTimer = new Timer();

  boolean lastPressed = false;

  public Shoot(Shooter m_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shooter);
    shooter = m_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    controller.setIntegratorRange(-1000.0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(Shooter.State == Shooter.STATES.SHOOT) {

      double velocitySetpoint = Shooter.setpoint;
      velocitySetpoint /= 60.0; //In rotations per second

      controller.setSetpoint(velocitySetpoint);

      double rps = shooter.getRPM() / 60.0;
      double power = feedforward.calculate(velocitySetpoint) + controller.calculate(rps);
      shooterData.updateEntry("RPM", rps * 60.0);
      shooterData.updateEntry("Setpoint", Shooter.setpoint);
      shooterData.updateEntry("Power", power);
      shooterData.updateEntry("Error", Shooter.setpoint - rps * 60.0);

      shooter.setVoltage(power);

      if(Math.abs(shooter.getRPM() - Shooter.setpoint) <= tolerance && toleranceTimer.get() > 0.1){

        Conveyor.State = Conveyor.STATES.FEED;
        Hopper.State = Hopper.STATES.FUNNEL;

      } else if(Math.abs(shooter.getRPM() - Shooter.setpoint) <= tolerance){
        
        toleranceTimer.start();

      }else{

        if(MechManager.State != AUTO_STATES.SHOOT){

          Conveyor.State = Conveyor.STATES.STOP;

        }
        toleranceTimer.reset();
        
      }

      lastPressed = true;

    } else if(Shooter.State == Shooter.STATES.EXPEL){

      double velocitySetpoint = Shooter.expelSetpoint;
      velocitySetpoint /= 60.0; //In rotations per second

      controller.setSetpoint(velocitySetpoint);

      double rps = shooter.getRPM() / 60; 
      double power = feedforward.calculate(velocitySetpoint) + controller.calculate(rps);
      shooterData.updateEntry("RPM", rps * 60.0);
      shooterData.updateEntry("Setpoint", Shooter.expelSetpoint);
      shooterData.updateEntry("Power", power);

      shooter.setVoltage(power);
    
    } else if(Shooter.State == Shooter.STATES.STOP){

      if(lastPressed){

        Conveyor.State = Conveyor.STATES.INDEX;

      }

      shooter.setVoltage(0.0);

      lastPressed = false;
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
