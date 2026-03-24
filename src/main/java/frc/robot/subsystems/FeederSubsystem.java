package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CanIds.FeederCanIds;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;

import java.util.function.DoubleSupplier;

/**
 * FeederSubsystem controls a single Vortex motor via a SPARK MAX + SOLO adapter.
 * <p>
 * Direct-drive hex axle with flapper wheels that hold fuel away from the
 * shooter wheels until they reach their target RPM, then feeds it in.
 * <p>
 * The feeder runs simple duty-cycle control — no PID needed for flappers.
 */
public class FeederSubsystem extends SubsystemBase {

  private final SparkMax m_feederMotor;

  public FeederSubsystem() {
    // SOLO adapter presents a Vortex as brushless to the SPARK MAX
    m_feederMotor = new SparkMax(FeederCanIds.kFeederMotorCanId, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config
        .idleMode(IdleMode.kBrake)          // Hold fuel in place when not feeding
        .smartCurrentLimit(FeederConstants.kCurrentLimit);

    m_feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ---------------------------------------------------------------------------
  // Control methods
  // ---------------------------------------------------------------------------

  /** Run the feeder flappers forward at the configured feed speed. */
  public void feed() {
    m_feederMotor.set(FeederConstants.kFeedSpeed);
    SmartDashboard.putBoolean("Feeder/Running", true);
    System.out.println("FeederSubsystem: feed() -> output=" + FeederConstants.kFeedSpeed);
  }

  /** Run the feeder in reverse (eject / unjam). */
  public void reverse() {
    m_feederMotor.set(-FeederConstants.kFeedSpeed);
    SmartDashboard.putBoolean("Feeder/Running", true);
    System.out.println("FeederSubsystem: reverse() -> output=" + -FeederConstants.kFeedSpeed);
  }

  /** Stop the feeder motor. */
  public void stop() {
    m_feederMotor.set(0.0);
    SmartDashboard.putBoolean("Feeder/Running", false);
    System.out.println("FeederSubsystem: stop() -> output=0.0");
  }

  // ---------------------------------------------------------------------------
  // Command factories
  // ---------------------------------------------------------------------------

  /** Feed while held, stop on release. */
  public Command feedCommand() {
    return startEnd(this::feed, this::stop);
  }

  /** Reverse while held, stop on release. */
  public Command reverseCommand() {
    return startEnd(this::reverse, this::stop);
  }

  /**
   * FULL SHOOT SEQUENCE — spins up ALL shooter motors, then feeds when at RPM.
   * <p>
   * This command <b>requires both the FeederSubsystem AND ShooterSubsystem</b>,
   * so it will pre-empt (cancel) any individual shooter commands that are running.
   * On release it stops both the feeder and all shooters.
   *
   * @param shooter the ShooterSubsystem to spin up and query for readiness
   */
  public Command shootAndFeedCommand(ShooterSubsystem shooter) {
    // FunctionalCommand lets us explicitly list required subsystems
    return new FunctionalCommand(
        // init — nothing extra needed
        () -> {},
        // execute — spin shooters, feed when ready
        () -> {
          shooter.spinAll();
          if (shooter.isAtTargetRPM(FeederConstants.kShooterRPMTolerance)) {
            feed();
          } else {
            stop();
          }
        },
        // end — stop everything
        interrupted -> {
          stop();
          shooter.stopAll();
        },
        // isFinished — never (run while held)
        () -> false,
        // require BOTH subsystems so this pre-empts individual commands
        this, shooter
    ).withName("ShootAndFeed");
  }

  public Command shootAndFeedVariableRPMCommand(ShooterSubsystem shooter, DoubleSupplier targetRPMSupplier) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          // double targetRPM = targetRPMSupplier.getAsDouble();
          double rawTriggerValue = targetRPMSupplier.getAsDouble();
          // Map trigger value (0.0–1.0) to a reasonable RPM range
          double targetRPM = ShooterConstants.kShooterMinRPM + rawTriggerValue * (ShooterConstants.kShooterMaxRPM - ShooterConstants.kShooterMinRPM);
          shooter.spinAllVariableRPM(targetRPM);
          if (shooter.isAtTargetVariableRPM(targetRPM, FeederConstants.kShooterRPMTolerance)) {
            feed();
          } else {
            stop();
          }
        },
        interrupted -> {
          stop();
          shooter.stopAll();
        },
        () -> false,
        this, shooter
    ).withName("ShootAndFeedVariableRPM");
  }

  /**
   * Spin up shooter 0 only, feed when it reaches target RPM.
   * Requires both subsystems — pre-empts other shooter/feeder commands.
   */
  public Command shootAndFeed0Command(ShooterSubsystem shooter) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          shooter.spinShooter0();
          if (shooter.isShooter0AtTargetRPM(FeederConstants.kShooterRPMTolerance)) {
            feed();
          } else {
            stop();
          }
        },
        interrupted -> {
          stop();
          shooter.stopShooter0();
        },
        () -> false,
        this, shooter
    ).withName("ShootAndFeed0");
  }

  /**
   * Spin up shooter 1 only, feed when it reaches target RPM.
   * Requires both subsystems — pre-empts other shooter/feeder commands.
   */
  public Command shootAndFeed1Command(ShooterSubsystem shooter) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          shooter.spinShooter1();
          if (shooter.isShooter1AtTargetRPM(FeederConstants.kShooterRPMTolerance)) {
            feed();
          } else {
            stop();
          }
        },
        interrupted -> {
          stop();
          shooter.stopShooter1();
        },
        () -> false,
        this, shooter
    ).withName("ShootAndFeed1");
  }

  /**
   * Spin up shooter 2 only, feed when it reaches target RPM.
   * Requires both subsystems — pre-empts other shooter/feeder commands.
   */
  public Command shootAndFeed2Command(ShooterSubsystem shooter) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          shooter.spinShooter2();
          if (shooter.isShooter2AtTargetRPM(FeederConstants.kShooterRPMTolerance)) {
            feed();
          } else {
            stop();
          }
        },
        interrupted -> {
          stop();
          shooter.stopShooter2();
        },
        () -> false,
        this, shooter
    ).withName("ShootAndFeed2");
  }

  /**
   * FULL REVERSE — reverses ALL shooter motors AND the feeder (unjam everything).
   * Requires both subsystems, pre-empts any running shooter/feeder commands.
   *
   * @param shooter the ShooterSubsystem to reverse
   */
  public Command reverseAllCommand(ShooterSubsystem shooter) {
    return new FunctionalCommand(
        () -> {},
        () -> {
          shooter.reverseAll();
          reverse();
        },
        interrupted -> {
          stop();
          shooter.stopAll();
        },
        () -> false,
        this, shooter
    ).withName("ReverseAll");
  }

  // ---------------------------------------------------------------------------
  // Telemetry
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Feeder/Output", m_feederMotor.getAppliedOutput());
    SmartDashboard.putNumber("Feeder/Current", m_feederMotor.getOutputCurrent());
    // Also expose a simple boolean for whether the feeder is actively running
    // (set by feed/reverse/stop). This helps verify button bindings are firing.
    if (!SmartDashboard.containsKey("Feeder/Running")) {
      SmartDashboard.putBoolean("Feeder/Running", false);
    }
  }
}
