package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.List;
import java.io.File;
import java.util.Arrays;

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
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // The operator's controller
  XboxController m_operatorController = new XboxController(OIConstants.kOperatorControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // =========================
    // Driver controller bindings
    // =========================

    new JoystickButton(m_driverController, XboxController.Button.kBack.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kB.value)
        .onTrue(new InstantCommand(
            () -> {
              m_robotDrive.zeroHeading();
              m_robotDrive.resetOdometry(
                  new Pose2d(
                      m_robotDrive.getPose().getTranslation(),
                      new Rotation2d(0)
                  )
              );
            },
            m_robotDrive));

    // Shoot / feed bindings
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(m_feeder.shootAndFeed0Command(m_shooter));

    new JoystickButton(m_driverController, XboxController.Button.kY.value)
        .whileTrue(m_feeder.shootAndFeed1Command(m_shooter));

    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(m_feeder.shootAndFeed2Command(m_shooter));

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
        .whileTrue(m_feeder.shootAndFeedCommand(m_shooter));

    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
        .whileTrue(m_feeder.reverseAllCommand(m_shooter));

    // Proper trigger binding for analog trigger axis
    new Trigger(() -> m_driverController.getRightTriggerAxis() > 0.10)
        .whileTrue(
            m_feeder.shootAndFeedVariableRPMCommand(
                m_shooter,
                () -> m_driverController.getRightTriggerAxis()
            )
        );

    new JoystickButton(m_driverController, XboxController.Button.kRightStick.value)
        .whileTrue(m_feeder.feedCommand());

    new JoystickButton(m_driverController, XboxController.Button.kLeftStick.value)
        .whileTrue(m_feeder.reverseCommand());

    // =========================
    // Operator controller bindings
    // =========================

    // Manual pivot jog
    new JoystickButton(m_operatorController, XboxController.Button.kLeftBumper.value)
        .whileTrue(m_intake.movePivot(-0.85));

    new JoystickButton(m_operatorController, XboxController.Button.kRightBumper.value)
        .whileTrue(m_intake.movePivot(1.00));

    // Intake roller
    new JoystickButton(m_operatorController, XboxController.Button.kX.value)
        .whileTrue(m_intake.moveIntake(-0.5));

    new JoystickButton(m_operatorController, XboxController.Button.kY.value)
        .whileTrue(m_intake.moveIntake(0.5));

    // Pivot presets
    new JoystickButton(m_operatorController, XboxController.Button.kA.value)
        .onTrue(m_intake.moveToIntakePosition());

    new JoystickButton(m_operatorController, XboxController.Button.kB.value)
        .onTrue(m_intake.moveToRampPosition());

    new JoystickButton(m_operatorController, XboxController.Button.kStart.value)
        .onTrue(m_intake.moveToStowPosition());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommandOld() {
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics);

    final double L = 1.0;

    Trajectory seg1 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
        List.of(),
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0)),
        config
    );

    Trajectory square = seg1;

    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController,
        0,
        0,
        AutoConstants.kThetaControllerConstraints
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        square,
        m_robotDrive::getPose,
        DriveConstants.kDriveKinematics,
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive
    );

    m_robotDrive.resetOdometry(square.getInitialPose());

    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  /**
   * Autonomous selection:
   * - Try PathPlanner Auto "New Auto"
   * - If it fails for ANY reason, do a 360 spin in place, then stop.
   */
  public Command getAutonomousCommand() {
    final String autoName = "New Auto";

    logDeployedPathPlannerFiles();

    try {
      Command ppAuto = AutoBuilder.buildAuto(autoName);
      if (ppAuto == null) {
        DriverStation.reportError(
            "PathPlanner AutoBuilder.buildAuto(\"" + autoName + "\") returned null.",
            false
        );
        return spinInPlace360Fallback();
      }
      System.out.println("Running PathPlanner auto: " + autoName);
      return ppAuto;
    } catch (Throwable t) {
      DriverStation.reportError(
          "PathPlanner auto failed: " + t.getClass().getSimpleName() + ": " + t.getMessage(),
          t.getStackTrace()
      );
      return spinInPlace360Fallback();
    }
  }

  /**
   * Fallback: rotate in place one full turn then stop.
   * - No lateral translation
   * - Uses robot-relative rotation
   */
  private Command spinInPlace360Fallback() {
    final double omegaRadPerSec = DriveConstants.kMaxAngularSpeed * 0.25;
    final double seconds = (2.0 * Math.PI) / Math.max(0.01, omegaRadPerSec);

    System.out.println(
        "Fallback auto: spin 360 in place for " + seconds + " seconds @ omega=" + omegaRadPerSec
    );

    return new RunCommand(
            () -> m_robotDrive.drive(
                0.0,
                0.0,
                omegaRadPerSec / DriveConstants.kMaxAngularSpeed,
                false
            ),
            m_robotDrive)
        .withTimeout(seconds)
        .andThen(new InstantCommand(() -> m_robotDrive.drive(0, 0, 0, false), m_robotDrive));
  }

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
  }
}