package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.EncoderData;
import frc.robot.Constants.EncoderPort;
import frc.robot.Constants.MotorID;

public class SwerveSubsystem extends SubsystemBase {
    private double frontLeftVelocity;
    private double frontRightVelocity;
    private double backLeftVelocity;
    private double backRightVelocity;
    ADIS16448_IMU gyro = new ADIS16448_IMU();
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
        Constants.driveKinematis, 
        new Rotation2d(0)
    );

    public final SwerveModule frontLeft = new SwerveModule(
        MotorID.FRONT_LEFT_MOTOR_ID,
        MotorID.FRONT_LEFT_TURN_ID,
        EncoderData.isFrontLeftDEncReversed,
        EncoderPort.FRONT_LEFT_ID_A,
        EncoderData.frontLeftAbsEncoderOffset,
        EncoderData.isFrontLeftAbsEncReversed
    );

    public final SwerveModule frontRight = new SwerveModule(
        MotorID.FRONT_RIGHT_MOTOR_ID, 
        MotorID.FRONT_RIGHT_TURN_ID, 
        EncoderData.isFrontRightDEncreversed, 
        EncoderPort.FRONT_RIGHT_ID_A, 
        EncoderData.frontRightAbsEncoderOffset, 
        EncoderData.isFrontRightAbsEncReversed
    );

    public final SwerveModule backLeft = new SwerveModule(
        MotorID.BACK_LEFT_MOTOR_ID, 
        MotorID.BACK_LEFT_TURN_ID, 
        EncoderData.isBackLeftDEncReversed, 
        EncoderPort.BACK_LEFT_ID_A, 
        EncoderData.backLeftAbsEncoderOffset, 
        EncoderData.isFrontLeftAbsEncReversed
    );

    public final SwerveModule backRight = new SwerveModule(
        MotorID.BACK_RIGHT_MOTOR_ID, 
        MotorID.BACK_RIGHT_TURN_ID, 
        EncoderData.isBackRightDEncReversed,
        EncoderPort.BACK_RIGHT_ID_A, 
        EncoderData.backRightAbsEncoderOffset, 
        EncoderData.isBackRightAbsEncReversed
    );

    public void resetGyro() {
        gyro.reset();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getGyroRotation());
    }

    public Pose2d getPose() {
        SmartDashboard.putString("Robot Location", odometry.getPoseMeters().getTranslation().toString());
        return odometry.getPoseMeters();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public double getAngle() {
        return gyro.getAngle();
    }

    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(getHeading() * -1);
    }

    public Rotation2d getRotation() {
        return Rotation2d.fromDegrees(getHeading() * -1);
    }

    public void updateOdometry() {
        odometry.update(getGyroRotation(), frontLeft.getState(),
        frontRight.getState(), backLeft.getState(), backRight.getState());
    }

    public void rawModuleInput(SwerveModuleState[] balls) {
        setModuleStates(balls);
    }

    public double getRobotVelocity() {
        // this probably isn't the best way or even a correct way of finding the velocity, oh well.
        frontLeftVelocity = frontLeft.getDriveVelocity();
        frontRightVelocity = frontRight.getDriveVelocity();
        backLeftVelocity = backLeft.getDriveVelocity();
        backRightVelocity = backRight.getDriveVelocity();
        return (frontLeftVelocity + frontRightVelocity + backLeftVelocity + backRightVelocity) / 4;
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backRight.stop();
        backLeft.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.physicalMaxSpeedMPerSecond );
        // frontLeft.setDesiredState(desiredStates[0]);
        // frontRight.setDesiredState(desiredStates[1]);
        // backLeft.setDesiredState(desiredStates[2]);
        // backRight.setDesiredState(desiredStates[3]);

        frontRight.setDesiredState(desiredStates[0]);
        frontLeft.setDesiredState(desiredStates[1]);
        backRight.setDesiredState(desiredStates[2]);
        backLeft.setDesiredState(desiredStates[3]);
    }

    @Override 
    public void periodic() {
        updateOdometry();
        SmartDashboard.putNumber("Fancy Heading", getHeading());
        SmartDashboard.putString("Roboto Locato", getPose().getTranslation().toString());
    }
}
