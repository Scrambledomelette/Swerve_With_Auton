package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

public class Spin extends CommandBase{
    
    SwerveSubsystem swerveSubsystem;

    SwerveModuleState swerveModuleState = new SwerveModuleState(0.1, new Rotation2d());
    SwerveModuleState[] moduleStates = {swerveModuleState, swerveModuleState, swerveModuleState, swerveModuleState};

    public Spin(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }


    @Override
    public void initialize() {
        // swerveSubsystem.setModuleStates(moduleStates);
        swerveSubsystem.frontLeft.runDriveMotors();
        System.out.println("spin");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
