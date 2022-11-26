package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutonSubsystem extends SubsystemBase {
    SwerveSubsystem swerveSubsystem;

    public AutonSubsystem(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
    }

    public void rawModuleInput(SwerveModuleState[] rawModuleStates) {
        swerveSubsystem.setModuleStates(rawModuleStates);
    }

    public void resetOdometry(Pose2d inputPose) {
        swerveSubsystem.resetOdometry(inputPose);
    }

    public void stopModules() {
        swerveSubsystem.stopModules();
    }

    public void print() {
        System.out.println("i'm not the null one!!");
    }
}
