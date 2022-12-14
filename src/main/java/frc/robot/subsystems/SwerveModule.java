package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;

public class SwerveModule {
    Constants constants;

    private final CANSparkMax driveMotor;
    private final TalonSRX turnMotor;
    private SwerveModuleState optimizedState;
    private final RelativeEncoder driveEncoder;
    private final AnalogEncoder turnEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetToDegrees;
    private double turnMotorPower;
    private double driveMotorPower;
    private double balance;
    private final PIDController turningPID;

    public SwerveModule( int driveMotorId, int turningMotorId, boolean driveMotorReversed,
     int absEncoderChannel, double absEncoderOffset, boolean absEncoderReversed ) {

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turnMotor = new TalonSRX(turningMotorId);

        driveMotor.setInverted(driveMotorReversed);
        // turnMotor.setInverted(turningMotorReversed); 

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = new AnalogEncoder(absEncoderChannel);

        turnMotor.setInverted(absEncoderReversed);
         
        this.absoluteEncoderOffsetToDegrees = absEncoderOffset;
        this.absoluteEncoderReversed = absEncoderReversed;

        driveEncoder.setPositionConversionFactor(constants.driveEncoderRotationsToMeters);
        driveEncoder.setVelocityConversionFactor(constants.driveEncoderRPMPerSecond);

        turningPID = new PIDController(constants.pTurning, constants.iTurning, constants.dTurning);
        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }


    public double getTurnRad() {
        return turnEncoder.getAbsolutePosition() * Math.PI * 2;
    }

    public double getTurnPos() {
        // return ((turnEncoder.getAbsolutePosition() * Math.PI * 2 ) - Math.toRadians(absoluteEncoderOffsetToDegrees));
        return (getTurnRad() - Math.toRadians(absoluteEncoderOffsetToDegrees) /* - Math.PI */);
        // return ((turnEncoder.getAbsolutePosition() * Math.PI * 2 ) - Math.PI); 
    }
    
    public double getDegreeTurnPos() {
        return Math.round(turnEncoder.getAbsolutePosition() * 360) - absoluteEncoderOffsetToDegrees;
    }

    public double getRawTurnPos() {
        // this does not account for motor offsets.
        return turnEncoder.getAbsolutePosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurnVelocity() {
        return turnEncoder.getDistancePerRotation();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurnPos()));
    }
    
    public void setBrakeMode() {
        driveMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoastMode() {
        driveMotor.setIdleMode(IdleMode.kCoast);
    }

    public void resetDriveEncoder() {
        driveEncoder.setPosition(0);
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(ControlMode.PercentOutput, 0);
    }

    public void goToSetPoint(double setPoint) {
        turnMotor.set(ControlMode.PercentOutput, turningPID.calculate(getRawTurnPos(), setPoint));
    }

    public double inputtedPercentOutput(double setPoint) {
        return turningPID.calculate(getRawTurnPos(), setPoint);
    }

    public void runAllMotors() {
        turnMotor.set(ControlMode.PercentOutput, 1);
        driveMotor.set(1);
    }

    public void runDriveMotors() {
        driveMotor.set(1);
    }

    public void runTurnMotors() {
        turnMotor.set(ControlMode.PercentOutput, 1);
    }
    
    public double getDriveSpeed() {
        return driveMotorPower;
    }

    public double getTurnSpeed() {
        return turnMotorPower;
    }

    public void setDesiredState(SwerveModuleState state) {
        optimizedState = SwerveModuleState.optimize(state, getState().angle);
        turnMotorPower = turningPID.calculate(getTurnPos(), optimizedState.angle.getRadians());
        driveMotorPower = optimizedState.speedMetersPerSecond /*/ 2*/;
        if (Math.abs(optimizedState.speedMetersPerSecond) < 0.01 ) {
            stop();
        }

        driveMotor.set(driveMotorPower);
        turnMotor.set(ControlMode.PercentOutput, turnMotorPower);
    } 
}