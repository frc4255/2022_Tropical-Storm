// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.Constants.DTProperties;
import frc.robot.subsystems.Drivetrain;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })

  Drivetrain drivetrain;

  SlewRateLimiter filter = new SlewRateLimiter(1.0);

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DTProperties.ksVolts, DTProperties.kvVoltSecondsPerMeter,
                                                                  DTProperties.kaVoltSecondsSquaredPerMeter);

  TabData drivetrainData = Shuphlebord.drivetrainData;

  public Drive(Drivetrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drivetrain);
    drivetrain = m_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
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

    double controllerY = -RobotContainer.driveController.getLeftY();
    double controllerX = RobotContainer.driveController.getRightX();

    double filteredY = filter.calculate(controllerY);

    WheelSpeeds speeds = drivetrain.curvatureDriveIK(filteredY, controllerX);

    double left = speeds.left;
    double right = speeds.right;

    drivetrainData.updateEntry("Filtered Left", left);
    drivetrainData.updateEntry("Filtered Right", right);

    drivetrain.tankDrive(left, right);

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
