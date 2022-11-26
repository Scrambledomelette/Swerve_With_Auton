package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

public class JoystickCommand extends CommandBase {
    Constants constants;
    SwerveSubsystem swerveSubsystem;
    SwerveModuleState swerveModuleState;
    private ChassisSpeeds chassisSpeeds;
    private Supplier<Double> xAxis;
    private Supplier<Double> yAxis;
    private Supplier<Double> zAxis;
    private double xSpeed;
    private double ySpeed;
    private double zSpeed;

    public JoystickCommand( SwerveSubsystem swerveSubsystem, Supplier<Double> xSpeed,
    Supplier<Double> ySpeed, Supplier<Double> zSpeed) {
        this.swerveSubsystem = swerveSubsystem;
        this.xAxis = xSpeed;
        this.yAxis = ySpeed;
        this.zAxis = zSpeed;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        xSpeed = -xAxis.get();
        ySpeed = yAxis.get();
        zSpeed = zAxis.get();
        if (xSpeed < 0.1 && xSpeed > -.1) {
            xSpeed = 0;
        } 
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed,xSpeed, zSpeed, swerveSubsystem.getGyroRotation());
        SwerveModuleState[] swerveModuleStates = constants.driveKinematis.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleStates);
    }
    
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            swerveSubsystem.stopModules();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}