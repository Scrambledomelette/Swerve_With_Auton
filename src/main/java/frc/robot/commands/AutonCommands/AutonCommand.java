package frc.robot.commands.AutonCommands;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PerpetualCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AutonSubsystem;

public class AutonCommand extends CommandBase {
    AutonSubsystem autonSubsystem;
    Constants constants;
    Constraints constraints;
    Boolean isFinished = false;
    PIDController xController;
    PIDController yController;
    ProfiledPIDController profiledController;
    TrajectoryConfig trajectoryConfig;
    Trajectory trajectory;
    SequentialCommandGroup commandGroup;
    SwerveControllerCommand controllerCommand;
    Supplier<Pose2d> robotPose;
    Pose2d printingPose;
    Consumer<SwerveModuleState[]> consumerOfModules;

    
    public AutonCommand(AutonSubsystem autonSubsystem, Supplier<Pose2d> poseSupplier) {
        this.autonSubsystem = autonSubsystem;
        this.robotPose = poseSupplier;
        addRequirements(autonSubsystem);
    }

    public void finish() {
        isFinished = true;
    }
 
    @Override
    public void initialize() {
        printingPose = robotPose.get();
        SmartDashboard.putString("Robot Location", printingPose.getTranslation().toString());
        consumerOfModules = autonSubsystem::rawModuleInput;
        constraints = new Constraints(100, 1);      
        xController = new PIDController(1, 0, 0);
        yController = new PIDController(1, 0, 0);
        profiledController = new ProfiledPIDController(.8, 0, 0, constraints);
        profiledController.enableContinuousInput(-Math.PI, Math.PI);

        trajectoryConfig = new TrajectoryConfig(
            Constants.physicalMaxSpeedMPerSecond,
            Constants.maxAllowedInputtedAcceleration
        );

        trajectory = TrajectoryGenerator.generateTrajectory(
            // the starting location of the robot.
            // since the odom is only used for this can just set location as origin
            new Pose2d(0,0, new Rotation2d(0)), 
            List.of(
                new Translation2d(50 , 0),
                new Translation2d(50, 50)
            ),
            new Pose2d(60, 60, new Rotation2d(0)),
            trajectoryConfig
        );

        controllerCommand = new SwerveControllerCommand(
            trajectory, 
            robotPose, 
            Constants.driveKinematis, 
            xController, 
            yController, 
            profiledController, 
            consumerOfModules, 
            autonSubsystem
        );

        // ok so this stuff is wierd but basically its like functions but 
        // not functions cuz they're declarative and named commands
        SequentialCommandGroup commandGroup = new SequentialCommandGroup(
            new PrintCommand("sugoma"),
            new InstantCommand(() -> autonSubsystem.resetOdometry(trajectory.getInitialPose())),
            new PrintCommand("odometry set"),
            controllerCommand,
            new PrintCommand("auton has been run, stopping modules."),
            new InstantCommand(() -> autonSubsystem.stopModules()),
            new PrintCommand("auton has stopped and finished, NOTHING SHOULD HAPPEN NOW")
            // new InstantCommand(() -> finish())
        );

        commandGroup.schedule();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

}
