// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double driveMotorGearRatio = 6.67;
    public static final double turningMotorGearRatio = 1.2;
    public static final double wheelDiameterInMeters = Units.inchesToMeters(4);

    // distance between right and left wheels
    public static final double trackWidth = Units.inchesToMeters(22);
    // distance between front and back wheels
    public static final double wheelBase = Units.inchesToMeters(21);

    // this creates the geometry of the swerveDrive for proper calculations. Also it has to be in meters.
    public static final SwerveDriveKinematics driveKinematis = new SwerveDriveKinematics(
    new Translation2d(wheelBase / 2, -trackWidth / 2),
    new Translation2d(wheelBase / 2, trackWidth / 2),
    new Translation2d(-wheelBase / 2, -trackWidth / 2),
    new Translation2d(-wheelBase, trackWidth / 2));

    public static final double driveEncoderRotationsToMeters = driveMotorGearRatio * Math.PI * wheelDiameterInMeters;
    public static final double turningEncoderRotationsToRadians = turningMotorGearRatio * 2 * Math.PI;
    public static final double driveEncoderRPMPerSecond = driveEncoderRotationsToMeters / 60;
    public static final double turnEncoderRPMPerSecond = turningEncoderRotationsToRadians / 60;
    public static final double physicalMaxSpeedMPerSecond = Units.feetToMeters(30);

    // the PID values for the swerve drive.
    public static final double pTurning = 0.8;
    public static final double iTurning = 0;
    public static final double dTurning = 0; 

    // these numbers are arbitrary, and the only way to find a good number is testing. YAY!
    public static final double maxAllowedInputtedAcceleration = 30;
    public static final double maxAllowedInputtedAngularAcceleration = 20;
    
    public static final class MotorID {
        public static final int FRONT_LEFT_MOTOR_ID = 6;
        public static final int FRONT_RIGHT_MOTOR_ID = 4;
        public static final int BACK_LEFT_MOTOR_ID = 8;
        public static final int BACK_RIGHT_MOTOR_ID = 2;
    
        public static final int FRONT_LEFT_TURN_ID = 5;
        public static final int FRONT_RIGHT_TURN_ID = 3;
        public static final int BACK_LEFT_TURN_ID = 7;
        public static final int BACK_RIGHT_TURN_ID = 1;
    
        public static final int TURRET_TURN_ID = 9;
        }
    
        public static final class EncoderPort {
        public static final int FRONT_LEFT_ID_A = 2;
        public static final int FRONT_RIGHT_ID_A = 0;
        public static final int BACK_LEFT_ID_A = 3;
        public static final int BACK_RIGHT_ID_A = 1;
        }
    
        public static final class EncoderData {
        // These reverse values are only used for the optimization of the wheel turning.
        public static final boolean isFrontLeftDEncReversed = true;
        public static final boolean isFrontRightDEncreversed = false;
        public static final boolean isBackLeftDEncReversed = true;
        public static final boolean isBackRightDEncReversed = false;
    
        public static final boolean isFrontLeftAbsEncReversed = false;
        public static final boolean isFrontRightAbsEncReversed = false;
        public static final boolean isBackLeftAbsEncReversed = false;
        public static final boolean isBackRightAbsEncReversed = false;
    
        // These offsets need to be adjusted constantly.
        public static final double frontLeftAbsEncoderOffset = 29;
        public static final double frontRightAbsEncoderOffset = 8;
        public static final double backLeftAbsEncoderOffset = 155;
        public static final double backRightAbsEncoderOffset = 356;
        }
}
