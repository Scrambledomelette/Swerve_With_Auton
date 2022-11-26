// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.JoystickCommand;
import frc.robot.commands.AutonCommands.AutonCommand;
import frc.robot.subsystems.AutonSubsystem;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  Supplier<Pose2d> pose2d;
  Consumer<SwerveModuleState[]> consumerOfModules;
  Constraints constraints = new TrapezoidProfile.Constraints(1, 1);      
  PIDController xController = new PIDController(0, 0, 0);
  PIDController yController = new PIDController(0, 0, 0);
  ProfiledPIDController profiledController = new ProfiledPIDController(0, 0, 0, constraints);


  // The robot's subsystems and commands are defined here
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final AutonSubsystem autonSubsystem = new AutonSubsystem(swerveSubsystem);

  private Joystick driverJoystick = new Joystick(0);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new JoystickCommand(
      swerveSubsystem,
      () -> driverJoystick.getRawAxis(1),
      () -> driverJoystick.getRawAxis(0),
      () -> driverJoystick.getRawAxis(2)
    ));
    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    new JoystickButton(driverJoystick, 5).whenPressed(() -> swerveSubsystem.resetGyro());
  }

  public Command getAutonomousCommand() {
    // Initial trajectory setup
    profiledController.enableContinuousInput(-Math.PI, Math.PI);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
        Constants.physicalMaxSpeedMPerSecond,
        Constants.maxAllowedInputtedAcceleration).setKinematics(Constants.driveKinematis);

    // create and declare trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, new Rotation2d(0)), 
        List.of(
            new Translation2d(0, -5),
            new Translation2d(0, -5)
        ),
        new Pose2d(0, -10, new Rotation2d(0)),
        trajectoryConfig
    );

    // create swerve command
    SwerveControllerCommand controllerCommand = new SwerveControllerCommand(
        trajectory, 
        swerveSubsystem::getPose,
        Constants.driveKinematis, 
        xController, 
        yController, 
        profiledController, 
        swerveSubsystem::setModuleStates, 
        autonSubsystem
    );

    // sequential command group
    SequentialCommandGroup commandGroup = new SequentialCommandGroup(
        new PrintCommand("sugoma"),
        new InstantCommand(() -> autonSubsystem.resetOdometry(trajectory.getInitialPose())),
        new PrintCommand("odometry set"),
        controllerCommand,
        new PrintCommand("auton has been run, stopping modules."),
        new InstantCommand(() -> autonSubsystem.stopModules()),
        new PrintCommand("auton has stopped and finished, NOTHING SHOULD HAPPEN NOW")
    );

    return commandGroup;

    // autonSubsystem.setDefaultCommand(new AutonCommand(
    //   autonSubsystem,
    //   () -> swerveSubsystem.getPose()
    // ));
    // return autonSubsystem.getDefaultCommand();
  }
}