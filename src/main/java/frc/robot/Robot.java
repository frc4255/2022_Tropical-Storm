// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.autonomous.AutoMechManager;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.MechManager.AUTO_STATES;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // PATHS FOR AUTO
  // 4 BALL PATHS

  public static class FourBallAuto{

    // PATH CHARACTERISTICS
    static double maxAccel = 3.0;
    static double maxVel = 5.0;

    // 4 BALL DIRECTORIES
    static String ball1Dir = "output/Ball1_FOUR_BALL.wpilib.json";
    static String shoot1Dir = "output/Shoot1_FOUR_BALL.wpilib.json";
    static String ball2and3Dir = "output/Ball2and3_FOUR_BALL.wpilib.json";
    static String shoot2Dir = "output/Shoot2_FOUR_BALL.wpilib.json";


    // 4 BALL TRAJECTORIES
    public static Trajectory ball1Trajectory = new Trajectory();
    public static Trajectory shoot1Trajectory = new Trajectory();
    public static Trajectory ball2and3Trajectory = new Trajectory();
    public static Trajectory shoot2Trajectory = new Trajectory();

    /*
    public static Trajectory ball1and2Trajectory = new Trajectory();
    public static Trajectory ball3and4Trajectory = new Trajectory();
    */
    
  }


  public static class TwoBallAuto{

    static double maxAccel = 3.0;
    static double maxVel = 5.0;

    // 2 BALL PATHS
    static String ball1Dir = "output/Ball1_TWO_BALL.wpilib.json";
    static String shoot1Dir = "output/Shoot1_TWO_BALL.wpilib.json";

    // 2 BALL TRAJECTORIES
    public static Trajectory ball1Trajectory = new Trajectory();
    public static Trajectory shoot1Trajectory = new Trajectory();

    public static Trajectory ball1and2Trajectory = new Trajectory();
    public static Trajectory trollKTurnTrajectory = new Trajectory();
    public static Trajectory trollSTurnTrajectory = new Trajectory();

  }


  public static class FiveBallAuto{

    // PATH CHARACTERISTICS
    static double maxAccel = 4.0;
    static double maxVel = 5.25;

    // 5 BALL TRAJECTORIES

    public static Trajectory ball1and2Trajectory = new Trajectory();
    public static Trajectory ball3Trajectory = new Trajectory();
    public static Trajectory ball4and5Trajectory = new Trajectory();

    public static Trajectory altBall2and3Trajectory = new Trajectory();
    public static Trajectory altBall4and5Trajectory = new Trajectory();

  }


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.

    // LIMELIGHT OVER USB
    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5802, "limelight.local", 5802);
    PortForwarder.add(5803, "limelight.local", 5803);
    PortForwarder.add(5804, "limelight.local", 5804);
    PortForwarder.add(5805, "limelight.local", 5805);


    // 4 BALL TRAJECTORIES
    FourBallAuto.ball1Trajectory = loadTrajectories(FourBallAuto.ball1Dir);
    FourBallAuto.shoot1Trajectory = loadTrajectories(FourBallAuto.shoot1Dir);
    FourBallAuto.ball2and3Trajectory = loadTrajectories(FourBallAuto.ball2and3Dir);
    FourBallAuto.shoot2Trajectory = loadTrajectories(FourBallAuto.shoot2Dir);

    /*
    FourBallAuto.ball1and2Trajectory = PathPlanner.loadPath("4_Ball_2-2_Ball_1-2", FourBallAuto.maxVel, FourBallAuto.maxAccel);
    FourBallAuto.ball3and4Trajectory = PathPlanner.loadPath("4_Ball_2-2_Ball_3-4", FourBallAuto.maxVel, FourBallAuto.maxAccel);
    */


    // 2 BALL TRAJECTORIES

    TwoBallAuto.ball1Trajectory = loadTrajectories(TwoBallAuto.ball1Dir);
    TwoBallAuto.shoot1Trajectory = loadTrajectories(TwoBallAuto.shoot1Dir);

    TwoBallAuto.ball1and2Trajectory = PathPlanner.loadPath("2_Ball_Ball_1-2", TwoBallAuto.maxVel, TwoBallAuto.maxAccel);
    TwoBallAuto.trollKTurnTrajectory = PathPlanner.loadPath("2_Ball_Hanger_Expell", TwoBallAuto.maxVel, TwoBallAuto.maxAccel);
    TwoBallAuto.trollSTurnTrajectory = PathPlanner.loadPath("2_Ball_Hanger_Expell_Alternate", TwoBallAuto.maxVel, TwoBallAuto.maxAccel);

    // 5 BALL TRAJECTORIES

    FiveBallAuto.ball1and2Trajectory = PathPlanner.loadPath("5_Ball_2-1-2_Ball_1-2", FiveBallAuto.maxVel, FiveBallAuto.maxAccel);
    FiveBallAuto.ball3Trajectory = PathPlanner.loadPath("5_Ball_2-1-2_Ball_3", FiveBallAuto.maxVel, FiveBallAuto.maxAccel);
    FiveBallAuto.ball4and5Trajectory = PathPlanner.loadPath("5_Ball_2-1-2_Ball_4-5", FiveBallAuto.maxVel, FiveBallAuto.maxAccel);
    FiveBallAuto.altBall2and3Trajectory = PathPlanner.loadPath("5_Ball_1-2-2_Ball_2-3", FiveBallAuto.maxVel, FiveBallAuto.maxAccel);
    FiveBallAuto.altBall4and5Trajectory = PathPlanner.loadPath("5_Ball_1-2-2_Ball_4-5", FiveBallAuto.maxVel, FiveBallAuto.maxAccel);

    m_robotContainer = new RobotContainer();
    LiveWindow.disableAllTelemetry();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    if(DriverStation.getAlliance() == DriverStation.Alliance.Red){
      Constants.allianceColor = 0;
    } else if(DriverStation.getAlliance() == DriverStation.Alliance.Blue){
      Constants.allianceColor = 1;
    }
    

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    Conveyor.ballsInConveyor = 0;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      new AutoMechManager(AUTO_STATES.DISABLE_INTAKE).schedule();;
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}


  /**
   * Method attempts to load a Trajectory from a path, will print the stack trace if failed.
   * @param path A string containing the path to the trajectory
   * @return A Trajectory
   */
  private Trajectory loadTrajectories(String path){
    Trajectory trajectory = new Trajectory();

    try {
      Path pathObj = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(pathObj);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
    }

    return trajectory;
  }
}
