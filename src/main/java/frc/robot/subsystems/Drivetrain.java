package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Shuphlebord;
import frc.robot.Constants.DTProperties;
import frc.robot.Constants;
import frc.robot.wrappers.TalonFXEncoder;

public class Drivetrain extends SubsystemBase {
  // The motors on the left side of the drive.

  WPI_TalonFX left0 = new WPI_TalonFX(Constants.Drivetrain.leftMotor0);
  WPI_TalonFX left1 = new WPI_TalonFX(Constants.Drivetrain.leftMotor1);
  WPI_TalonFX left2 = new WPI_TalonFX(Constants.Drivetrain.leftMotor2);

  private final MotorControllerGroup m_leftMotors = new MotorControllerGroup(left0, left1, left2);

  // The motors on the right side of the drive.

  WPI_TalonFX right0 = new WPI_TalonFX(Constants.Drivetrain.rightMotor0);
  WPI_TalonFX right1 = new WPI_TalonFX(Constants.Drivetrain.rightMotor1);
  WPI_TalonFX right2 = new WPI_TalonFX(Constants.Drivetrain.rightMotor2);

  private final MotorControllerGroup m_rightMotors = new MotorControllerGroup(right0, right1, right2);

  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

  // The encoders
  public TalonFXEncoder m_leftEncoder = new TalonFXEncoder(left0, left1, left2);
  public TalonFXEncoder m_rightEncoder = new TalonFXEncoder(right0, right1, right2);

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

    // Set break modes
    left0.setNeutralMode(NeutralMode.Coast);
    left1.setNeutralMode(NeutralMode.Coast);
    left2.setNeutralMode(NeutralMode.Coast);

    right0.setNeutralMode(NeutralMode.Coast);
    right1.setNeutralMode(NeutralMode.Coast);
    right2.setNeutralMode(NeutralMode.Coast);

    // Sets the distance per pulse for the encoders
    m_leftEncoder.setDistancePerPulse(DTProperties.kEncoderDistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DTProperties.kEncoderDistancePerPulse);

    m_leftMotors.setInverted(true);

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

    Rotation2d rot = new Rotation2d(-m_gyro.getRotation2d().getRadians());

    m_odometry.update(rot, -m_leftEncoder.getDistance(), m_rightEncoder.getDistance());
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
      m_drive.curvatureDrive(throttle, 0.5 * turn , true);
    } else{
      m_drive.curvatureDrive(throttle, turn, false);
    }

  }

  /**
   * Returns Wheel Speed Setpoints for curvature drive
   * 
   * @param throttle
   * @param turn
   */
  public WheelSpeeds curvatureDriveIK(double throttle, double turn){

    WheelSpeeds speeds;

    if(Math.abs(throttle) <= 0.1){
      speeds = DifferentialDrive.curvatureDriveIK(throttle, 0.5 * turn , true);
    } else{
      speeds = DifferentialDrive.curvatureDriveIK(throttle, turn, false);
    }

    return speeds;

  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /**
   * 
   * 
   */
  public void tankDrive(double leftSpeed, double rightSpeed){

    m_leftMotors.set(leftSpeed);
    m_rightMotors.set(rightSpeed);

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

}