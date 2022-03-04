// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Drivetrain{

        public static final int leftMotor0 = 0;
        public static final int leftMotor1 = 1;
        public static final int leftMotor2 = 2;
        public static final int rightMotor0 = 3;
        public static final int rightMotor1 = 4;
        public static final int rightMotor2 = 5;

        public static final double gearRatio = 9.0 / 64.0;
        public static final double wheelCircumference = Math.PI * 4;

    }

    public static final class Shooter{

        public static final int leftMotor = 6;
        public static final int rightMotor = 7;

    }

    public static final class Hopper{

        public static final int leftMotor = 8;
        public static final int rightMotor = 9;

    }

    public static final class Conveyor{

        public static final int motor = 10;

    }

    public static final class Intake{

        public static final int motor = 11;

        public static final int arm = 0;

    }

    public static final class DTProperties{

        // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.63879;
        public static final double kvVoltSecondsPerMeter = 2.3536;
        public static final double kaVoltSecondsSquaredPerMeter = 0.15842;

        // Example value only - as above, this must be tuned for your drive!
        public static final double kPDriveVel = 2.5803;

        public static final double kTrackwidthMeters = 0.64135;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

        private static final double inchesToMeters = 0.0254;
        public static final double kEncoderDistancePerPulse = inchesToMeters * Drivetrain.wheelCircumference * THROUGH_BORE_ENCODER_REVS_PER_TICK * Drivetrain.gearRatio;

    }

    public final class AutoConstants{

        public static final double kMaxSpeedMetersPerSecond = 0.01;//3.048;
        public static final double kMaxAccelerationMetersPerSecondSquared = 0.005;//3.0;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2.0;
        public static final double kRamseteZeta = 0.7;

    }

    public static final double THROUGH_BORE_ENCODER_REVS_PER_TICK = 1.0 / 2048.0;

}
