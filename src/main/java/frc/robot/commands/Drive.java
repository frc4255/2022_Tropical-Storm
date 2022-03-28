// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.Constants.DTProperties;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Drivetrain drivetrain;

  SlewRateLimiter filter = new SlewRateLimiter(1.0);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DTProperties.ksVolts, DTProperties.kvVoltSecondsPerMeter,
                                                                  DTProperties.kaVoltSecondsSquaredPerMeter);

  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double tolerance = 0.5;
  PIDController controller = new PIDController(kp, ki, kd);

  TabData drivetrainData = Shuphlebord.drivetrainData;

  public Drive(Drivetrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    drivetrain = m_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    drivetrainData.updateEntry("kP", kp);
    drivetrainData.updateEntry("kI", ki);
    drivetrainData.updateEntry("kD", kd);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(drivetrain.firstRead){
      drivetrain.m_gyro.reset();
      drivetrain.firstRead = false;
    }

    drivetrain.m_odometry.update(drivetrain.m_gyro.getRotation2d(), -drivetrain.m_leftEncoder.getDistance(), drivetrain.m_rightEncoder.getDistance());
    drivetrainData.updateEntry("Pose X", drivetrain.getPose().getX());
    drivetrainData.updateEntry("Pose Y", drivetrain.getPose().getY());
    drivetrainData.updateEntry("Pose A", drivetrain.getPose().getRotation().getDegrees());


    double adjustedkp = drivetrainData.getEntryData("kP").getDouble();
    double adjustedki = drivetrainData.getEntryData("kI").getDouble();
    double adjustedkd = drivetrainData.getEntryData("kD").getDouble();

    if(kd != adjustedkp || ki != adjustedki || kd != adjustedkd){
      kp = adjustedkp;
      ki = adjustedki;
      kd = adjustedkd;

      controller.setPID(kp, ki, kd);
    }

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

    if(Drivetrain.State == Drivetrain.STATES.MANUAL){

      controller.reset();

      double controllerY = -RobotContainer.driveController.getLeftY();
      double controllerX = RobotContainer.driveController.getRightX();

      double filteredY = filter.calculate(controllerY);

      WheelSpeeds speeds = drivetrain.curvatureDriveIK(filteredY, controllerX);

      double left = speeds.left;
      double right = speeds.right;

      drivetrainData.updateEntry("Filtered Left", left);
      drivetrainData.updateEntry("Filtered Right", right);

      drivetrain.tankDrive(left, right);
    
    // ------------------------------------------------------------------------------------------------------
    } else if(Drivetrain.State == Drivetrain.STATES.AUTO){

      double pos = Limelight.getTx();
      controller.setSetpoint(0.0);
      
      double power = controller.calculate(pos);

      if(Math.abs(Math.abs(pos)) > tolerance){
        drivetrain.arcadeDrive(0.0, power);
        Limelight.ALIGNED = false;
      } else{
        Limelight.ALIGNED = true;
        drivetrain.arcadeDrive(0.0, 0.0);
      }
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
