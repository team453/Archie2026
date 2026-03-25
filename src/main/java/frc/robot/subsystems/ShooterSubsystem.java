package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.units.measure.AngularVelocity;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.CanIds.ShooterCanIds;
import frc.robot.Constants.ShooterConstants;

/**
 * ShooterSubsystem controls three Falcon 500 motors (TalonFX) via Phoenix 6.
 * Motors run in REVERSE (inverted) using closed-loop velocity (RPM) control.
 */
public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX m_shooter0;
  private final TalonFX m_shooter1;
  private final TalonFX m_shooter2;

  // Cached velocity signals — avoids blocking CAN reads every periodic loop
  private final StatusSignal<AngularVelocity> m_vel0;
  private final StatusSignal<AngularVelocity> m_vel1;
  private final StatusSignal<AngularVelocity> m_vel2;

  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_neutralRequest = new NeutralOut();

  public ShooterSubsystem() {
    m_shooter0 = new TalonFX(ShooterCanIds.kShooterMotor0CanId, "rio");
    m_shooter1 = new TalonFX(ShooterCanIds.kShooterMotor1CanId, "rio");
    m_shooter2 = new TalonFX(ShooterCanIds.kShooterMotor2CanId, "rio");

    configureMotor(m_shooter0);
    configureMotor(m_shooter1);
    configureMotor(m_shooter2);

    // Cache the velocity StatusSignals once (they auto-update via CAN bus)
    m_vel0 = m_shooter0.getVelocity();
    m_vel1 = m_shooter1.getVelocity();
    m_vel2 = m_shooter2.getVelocity();
  }

  /** Apply a common configuration to each Falcon 500 motor. */
  private void configureMotor(TalonFX motor) {
    TalonFXConfiguration config = new TalonFXConfiguration();

    // Invert the motor output (shooters run in reverse)
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // Current limits to protect the Falcons
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;

    // Velocity PID in Slot 0
    Slot0Configs slot0 = config.Slot0;
    slot0.kP = ShooterConstants.kP;
    slot0.kI = ShooterConstants.kI;
    slot0.kD = ShooterConstants.kD;
    slot0.kV = ShooterConstants.kV;
    slot0.kS = ShooterConstants.kS;

    motor.getConfigurator().apply(config);
  }

  // ---------------------------------------------------------------------------
  // Public control methods
  // ---------------------------------------------------------------------------

  /** Convert RPM to rotations per second (Phoenix 6 native velocity unit). */
  private static double rpmToRps(double rpm) {
    return rpm / 60.0;
  }

  /** Spin a single motor at the given RPM. */
  private void setMotorRPM(TalonFX motor, double rpm) {
    motor.setControl(m_velocityRequest.withVelocity(rpmToRps(rpm)));
  }

  /** Stop a single motor. */
  private void stopMotor(TalonFX motor) {
    motor.setControl(m_neutralRequest);
  }

  // --- Individual motor helpers ---

  public void spinShooter0() {
    setMotorRPM(m_shooter0, ShooterConstants.kShooterTargetRPM);
  }

  public void spinShooter1() {
    setMotorRPM(m_shooter1, ShooterConstants.kShooterTargetRPM);
  }

  public void spinShooter2() {
    setMotorRPM(m_shooter2, ShooterConstants.kShooterTargetRPM);
  }

  public void stopShooter0() {
    stopMotor(m_shooter0);
  }

  public void stopShooter1() {
    stopMotor(m_shooter1);
  }

  public void stopShooter2() {
    stopMotor(m_shooter2);
  }

  /** Spin ALL three shooter motors to the target RPM. */
  public void spinAll() {
    spinShooter0();
    spinShooter1();
    spinShooter2();
  }

  public void spinAllVariableRPM(double rpm) {
    setMotorRPM(m_shooter0, rpm);
    setMotorRPM(m_shooter1, rpm);
    setMotorRPM(m_shooter2, rpm);
  }

  /** Stop ALL three shooter motors. */
  public void stopAll() {
    stopShooter0();
    stopShooter1();
    stopShooter2();
  }

  /** Reverse ALL three shooter motors (for unjamming). */
  public void reverseAll() {
    setMotorRPM(m_shooter0, -ShooterConstants.kShooterTargetRPM);
    setMotorRPM(m_shooter1, -ShooterConstants.kShooterTargetRPM);
    setMotorRPM(m_shooter2, -ShooterConstants.kShooterTargetRPM);
  }

  // ---------------------------------------------------------------------------
  // Command factories (for easy button binding)
  // ---------------------------------------------------------------------------

  /** Command that spins shooter 0 while held, stops on release. */
  public Command spinShooter0Command() {
    return startEnd(this::spinShooter0, this::stopShooter0);
  }

  /** Command that spins shooter 1 while held, stops on release. */
  public Command spinShooter1Command() {
    return startEnd(this::spinShooter1, this::stopShooter1);
  }

  /** Command that spins shooter 2 while held, stops on release. */
  public Command spinShooter2Command() {
    return startEnd(this::spinShooter2, this::stopShooter2);
  }

  /** Command that spins ALL shooters while held, stops on release. */
  public Command spinAllCommand() {
    return startEnd(this::spinAll, this::stopAll);
  }

  /** Command that reverses ALL shooters while held, stops on release. */
  public Command reverseAllCommand() {
    return startEnd(this::reverseAll, this::stopAll);
  }

  // ---------------------------------------------------------------------------
  // Readiness check (used by FeederSubsystem)
  // ---------------------------------------------------------------------------

  /** Returns true if ALL three shooter motors are within tolerance of the target RPM. */
  public boolean isAtTargetRPM(double toleranceRPM) {
    // double target = ShooterConstants.kShooterTargetRPM;
    // double rpm0 = m_vel0.getValueAsDouble() * 60.0;
    // double rpm1 = m_vel1.getValueAsDouble() * 60.0;
    // double rpm2 = m_vel2.getValueAsDouble() * 60.0;
    // return Math.abs(rpm0 - target) < toleranceRPM
    //     && Math.abs(rpm1 - target) < toleranceRPM
    //     && Math.abs(rpm2 - target) < toleranceRPM;
    return isAtTargetVariableRPM(ShooterConstants.kShooterTargetRPM, toleranceRPM);
  }

  public boolean isAtTargetVariableRPM(double targetRPM, double toleranceRPM) {
    double rpm0 = m_vel0.getValueAsDouble() * 60.0;
    double rpm1 = m_vel1.getValueAsDouble() * 60.0;
    double rpm2 = m_vel2.getValueAsDouble() * 60.0;
    return Math.abs(rpm0 - targetRPM) < toleranceRPM
        && Math.abs(rpm1 - targetRPM) < toleranceRPM
        && Math.abs(rpm2 - targetRPM) < toleranceRPM;
  }

  /** Returns true if shooter motor 0 is within tolerance of the target RPM. */
  public boolean isShooter0AtTargetRPM(double toleranceRPM) {
    return Math.abs(m_vel0.getValueAsDouble() * 60.0 - ShooterConstants.kShooterTargetRPM) < toleranceRPM;
  }

  /** Returns true if shooter motor 1 is within tolerance of the target RPM. */
  public boolean isShooter1AtTargetRPM(double toleranceRPM) {
    return Math.abs(m_vel1.getValueAsDouble() * 60.0 - ShooterConstants.kShooterTargetRPM) < toleranceRPM;
  }

  /** Returns true if shooter motor 2 is within tolerance of the target RPM. */
  public boolean isShooter2AtTargetRPM(double toleranceRPM) {
    return Math.abs(m_vel2.getValueAsDouble() * 60.0 - ShooterConstants.kShooterTargetRPM) < toleranceRPM;
  }

  // ---------------------------------------------------------------------------
  // Telemetry
  // ---------------------------------------------------------------------------

  @Override
  public void periodic() {
    // Batch-refresh all cached velocity signals in one CAN round-trip
    BaseStatusSignal.refreshAll(m_vel0, m_vel1, m_vel2);

    // Publish actual RPM to SmartDashboard for tuning
    SmartDashboard.putNumber("Shooter/Motor0 RPM", m_vel0.getValueAsDouble() * 60.0);
    SmartDashboard.putNumber("Shooter/Motor1 RPM", m_vel1.getValueAsDouble() * 60.0);
    SmartDashboard.putNumber("Shooter/Motor2 RPM", m_vel2.getValueAsDouble() * 60.0);
  }
}
