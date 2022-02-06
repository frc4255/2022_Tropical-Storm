// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DTProperties;
import frc.robot.commands.Drive;
import frc.robot.commands.Shoot;
import frc.robot.commands.Suck;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Shooter m_shooter = new Shooter();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Intake m_intake = new Intake();

  // Controllers
  public static final XboxController controller = new XboxController(0);

  // Buttons
  public final JoystickButton shootButton = new JoystickButton(controller, Button.kRightBumper.value);
  public final JoystickButton intakeButton = new JoystickButton(controller, Button.kLeftBumper.value);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_shooter.setDefaultCommand(new Shoot(m_shooter));
    m_drivetrain.setDefaultCommand(new Drive(m_drivetrain));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    intakeButton.whileHeld(new Suck(m_intake));

  }

  private RamseteCommand getRamseteCommand(Trajectory trajectory){

    RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_drivetrain::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DTProperties.ksVolts, DTProperties.kvVoltSecondsPerMeter, DTProperties.kaVoltSecondsSquaredPerMeter),
        DTProperties.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DTProperties.kPDriveVel, 0, 0),
        new PIDController(DTProperties.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_drivetrain::tankDriveVolts,
        m_drivetrain
    );

    return ramseteCommand;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // CREATE PATHS
    RamseteCommand shorty = getRamseteCommand(Robot.shortyTrajectory);
    // RamseteCommand sPathToBalls = getRamseteCommand(Robot.sPTBTrajectory);
    // RamseteCommand intakeBalls = getRamseteCommand(Robot.iBTrajectory);
    // RamseteCommand backToLine = getRamseteCommand(Robot.bTLTrajectory);
    // RamseteCommand driveAndIntake = getRamseteCommand(Robot.dAITrajectory);

    // Reset odometry to the starting pose of the trajectory.
    m_drivetrain.resetOdometry(Robot.shortyTrajectory.getInitialPose());

    
    return shorty;
  }
}
