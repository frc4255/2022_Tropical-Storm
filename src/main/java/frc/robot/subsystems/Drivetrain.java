package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shuphlebord;
import frc.robot.TabData;
import frc.robot.Constants.DTProperties;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.wrappers.TalonFXEncoder;
import frc.robot.Goals;

public class Drivetrain extends SubsystemBase {
  // The motors on the left side of the drive.

  WPI_TalonFX topLeft = new WPI_TalonFX(DrivetrainConstants.topLeftMotor);
  WPI_TalonFX bottomLeft = new WPI_TalonFX(DrivetrainConstants.bottomLeftMotor);

  private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(topLeft, bottomLeft);

  // The motors on the right side of the drive.

  WPI_TalonFX topRight = new WPI_TalonFX(DrivetrainConstants.topRightMotor);
  WPI_TalonFX bottomRight = new WPI_TalonFX(DrivetrainConstants.bottomRightMotor);

  private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(topRight, bottomRight);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The encoders
  public TalonFXEncoder m_leftEncoder = new TalonFXEncoder(topLeft, bottomLeft);
  public TalonFXEncoder m_rightEncoder = new TalonFXEncoder(topRight, bottomRight);

  // The gyro sensor
  public final AHRS m_gyro = new AHRS();

  // Odometry class for tracking robot pose
  public final DifferentialDriveOdometry m_odometry;

  // Boolean used to set initial error offset.
  public boolean firstRead = true;


  /**
   * Creates a new DriveSubsystem.
   */
  public Drivetrain() {
    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DTProperties.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DTProperties.kEncoderDistancePerPulse);

    resetEncoders();
    
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    if(firstRead){
      m_gyro.reset();
      firstRead = false;
    }

    m_odometry.update(m_gyro.getRotation2d(), -m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
    Shuphlebord.drivetrainData.updateEntry("Pose X", getPose().getX());
    Shuphlebord.drivetrainData.updateEntry("Pose Y", getPose().getY());
    Shuphlebord.drivetrainData.updateEntry("Pose A", getPose().getRotation().getDegrees());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * 
   * @return The distance between the goal and the robot.
   */
  public double getDistance(){
    Translation2d goalPos = Goals.getGoalPosition();

    Pose2d pose = getPose();

    double deltaX = Math.abs(pose.getX() - goalPos.getX());
    double deltaY = Math.abs(pose.getY() - goalPos.getY());

    double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);

    return distance;
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(-m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, m_gyro.getRotation2d());
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Drives the robot using curvature drive
   * 
   * @param throttle
   * @param turn
   */
  public void curvatureDrive(double throttle, double turn){

    if(Math.abs(throttle) <= 0.1){
      m_drive.curvatureDrive(throttle, turn , true);
    } else{
      m_drive.curvatureDrive(throttle, turn, false);
    }
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(-leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (-m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public TalonFXEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public TalonFXEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Zeroes the heading of the robot.
   */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate();
  }

  /**
   * Returns the current draws of each drivetrain motor
   * 
   * @return An double array containing the supply amperage for each motor
   */
  public double[] getCurrentDraw(){

    TabData data = Shuphlebord.powerData;

    double tl = topLeft.getSupplyCurrent();
    double tr = topRight.getSupplyCurrent();
    double bl = bottomLeft.getSupplyCurrent();
    double br = bottomRight.getSupplyCurrent();

    data.updateEntry("Drivetrain TL", tl);
    data.updateEntry("Drivetrian TR", tr);
    data.updateEntry("Drivetrain BL", bl);
    data.updateEntry("Drivetrian BR", br);

    double[] powers = {tl, tr, bl, br};

    return powers;

  }
}