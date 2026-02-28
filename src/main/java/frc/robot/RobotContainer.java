// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;

import java.io.File;
import java.util.Arrays;

// PathPlanner (make sure vendor lib is installed)
import com.pathplanner.lib.auto.AutoBuilder;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // The operator's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // Set up driver controller
    new JoystickButton(m_driverController, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    // Button 2 (B) -> zero the gyro and reset odometry so current heading becomes the new zero
    new JoystickButton(m_driverController, 2)
        .onTrue(new InstantCommand(
            () -> {
              // Zero the physical gyro/Yaw on the Pigeon
              m_robotDrive.zeroHeading();
              // Reset odometry to keep current translation but set rotation to 0
              m_robotDrive.resetOdometry(new Pose2d(m_robotDrive.getPose().getTranslation(), new Rotation2d(0)));
            },
            m_robotDrive));

    // TODO: Set up operator controller
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandOld() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // // An example trajectory to follow. All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(3, 0, new Rotation2d(0)),
    //     config);

    final double L = 1.0;

    Trajectory seg1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
        List.of(),
        new Pose2d(0.0, -L, Rotation2d.fromDegrees(0.0)),
        config
    );

    // Trajectory seg2 = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(L, 0.0, Rotation2d.fromDegrees(90.0)),
    //     List.of(),
    //     new Pose2d(L, L, Rotation2d.fromDegrees(90.0)),
    //     config
    // );

    // Trajectory seg3 = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(L, L, Rotation2d.fromDegrees(180.0)),
    //     List.of(),
    //     new Pose2d(0.0, L, Rotation2d.fromDegrees(180.0)),
    //     config
    // );

    // Trajectory seg4 = TrajectoryGenerator.generateTrajectory(
    //     new Pose2d(0.0, L, Rotation2d.fromDegrees(-90.0)),
    //     List.of(),
    //     new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(-90.0)),
    //     config
    // );

    Trajectory square = seg1;
    // .concatenate(seg2).concatenate(seg3).concatenate(seg4);


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        // exampleTrajectory,
        square,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    // m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());
    m_robotDrive.resetOdometry(square.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  } // getAutonomousCommandOld

  /**
   * Autonomous selection:
   * - Try PathPlanner Auto "Example Path"
   * - If it fails for ANY reason, do a 360 spin in place (no lateral movement), then stop.
   */
  public Command getAutonomousCommand() {
    final String autoName = "New Auto"; // MUST match the Auto name in PathPlanner GUI

    // Helpful diagnostics: list what is actually deployed on the roboRIO
    logDeployedPathPlannerFiles();

    // Try PathPlanner
    try {
      // If AutoBuilder wasn't configured (in DriveSubsystem), buildAuto can throw.
      Command ppAuto = AutoBuilder.buildAuto(autoName);
      if (ppAuto == null) {
        DriverStation.reportError("PathPlanner AutoBuilder.buildAuto(\"" + autoName + "\") returned null.", false);
        return spinInPlace360Fallback();
      }
      System.out.println("Running PathPlanner auto: " + autoName);
      return ppAuto;
    } catch (Throwable t) {
      DriverStation.reportError("PathPlanner auto failed: " + t.getClass().getSimpleName() + ": " + t.getMessage(), t.getStackTrace());
      return spinInPlace360Fallback();
    }
  } // getAutonomousCommand

  /**
   * Fallback: rotate in place one full turn then stop.
   * - No lateral translation (x=y=0)
   * - Uses robot-relative rotation (fieldRelative=false)
   */
  private Command spinInPlace360Fallback() {
    // Choose a safe angular speed (rad/s). Keep it modest so it doesn't tip/oscillate.
    // If your DriveConstants.kMaxAngularSpeed is realistic, 25% of it is a good start.
    final double omegaRadPerSec = DriveConstants.kMaxAngularSpeed * 0.25;

    // Time to rotate 2*pi radians at omega = 2*pi/omega seconds
    final double seconds = (2.0 * Math.PI) / Math.max(0.01, omegaRadPerSec);

    System.out.println("Fallback auto: spin 360 in place for " + seconds + " seconds @ omega=" + omegaRadPerSec);

    return new RunCommand(
            () -> m_robotDrive.drive(0.0, 0.0, omegaRadPerSec / DriveConstants.kMaxAngularSpeed, false),
            m_robotDrive)
        .withTimeout(seconds)
        .andThen(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));
  } // spinInPlace360Fallback

  private void logDeployedPathPlannerFiles() {
    try {
      File deployDir = Filesystem.getDeployDirectory();
      File ppDir = new File(deployDir, "pathplanner");
      File autosDir = new File(ppDir, "autos");
      File pathsDir = new File(ppDir, "paths");

      System.out.println("WPILib deploy directory: " + deployDir.getAbsolutePath());
      System.out.println("PathPlanner dir: " + ppDir.getAbsolutePath());

      if (autosDir.exists() && autosDir.isDirectory()) {
        System.out.println("Deployed PathPlanner autos (" + autosDir.getAbsolutePath() + "):");
        File[] files = autosDir.listFiles();
        if (files != null) {
          Arrays.sort(files);
          for (File f : files) {
            System.out.println(" - " + f.getName());
          }
        }
      } else {
        System.out.println("No PathPlanner autos directory found at: " + autosDir.getAbsolutePath());
      }

      if (pathsDir.exists() && pathsDir.isDirectory()) {
        System.out.println("Deployed PathPlanner paths (" + pathsDir.getAbsolutePath() + "):");
        File[] files = pathsDir.listFiles();
        if (files != null) {
          Arrays.sort(files);
          for (File f : files) {
            System.out.println(" - " + f.getName());
          }
        }
      } else {
        System.out.println("No PathPlanner paths directory found at: " + pathsDir.getAbsolutePath());
      }
    } catch (Throwable t) {
      System.out.println("Failed to list deployed PathPlanner files: " + t.getMessage());
    }
  } // logDeployedPathPlannerFiles
}
