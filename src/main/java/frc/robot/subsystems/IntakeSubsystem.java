package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.IntakeSubsystemConstants;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class IntakeSubsystem extends SubsystemBase {
    public enum PivotPreset {
        STOW,
        RAMP,
        INTAKE
    }

    private static final String SD_PREFIX = "IntakePivot/";
    private static final String SD_STOW_POSITION = SD_PREFIX + "StowPosition";
    private static final String SD_RAMP_POSITION = SD_PREFIX + "RampPosition";
    private static final String SD_INTAKE_POSITION = SD_PREFIX + "IntakePosition";
    private static final String SD_PIVOT_SETPOINT = SD_PREFIX + "PivotSetpoint";
    private static final String SD_PIVOT_PID_OUTPUT = SD_PREFIX + "PivotPIDOutput";
    private static final String SD_PIVOT_ENCODER = SD_PREFIX + "PivotEncoderPosition";
    private static final String SD_PIVOT_CONNECTED = SD_PREFIX + "PivotEncoderConnected";
    private static final String SD_PIVOT_MOTOR_OUTPUT = SD_PREFIX + "PivotMotorOutput";
    private static final String SD_PIVOT_AT_SETPOINT = SD_PREFIX + "PivotAtSetpoint";
    private static final String SD_PIVOT_TARGET_NAME = SD_PREFIX + "PivotTargetName";
    private static final String SD_PIVOT_MANUAL_MODE = SD_PREFIX + "PivotManualMode";
    private static final String SD_PIVOT_TOLERANCE = SD_PREFIX + "PivotTolerance";
    private static final String SD_PIVOT_UP_MAX = SD_PREFIX + "PivotUpMaxOutput";
    private static final String SD_PIVOT_DOWN_MAX = SD_PREFIX + "PivotDownMaxOutput";

    // These are ONLY starting placeholders for this year's robot.
    // Tune them on the real robot and update them on SmartDashboard / Shuffleboard.
    // Because the encoder mounting/index is different this year, do NOT trust last year's values.
    // The DutyCycleEncoder returns a raw duty value in [0,1). Previously we stored
    // raw values here; convert them to degrees for safer angle math (0..360).
    // We initialize defaults by converting the old raw defaults -> degrees.
    private static final double DEFAULT_STOW_POSITION = 0.18 * 360.0; // ~64.8 deg
    private static final double DEFAULT_RAMP_POSITION = 0.0029 * 360.0; // ~1.044 deg
    private static final double DEFAULT_INTAKE_POSITION = 0.88 * 360.0; // ~316.8 deg

    private static final double DEFAULT_PIVOT_TOLERANCE = 0.015;
    private static final double DEFAULT_PIVOT_UP_MAX_OUTPUT = 0.65;
    private static final double DEFAULT_PIVOT_DOWN_MAX_OUTPUT = -0.25;

    private final TalonFX m_intakeMotorFx;
    private final SparkMax m_pivotMotor;
    private final DutyCycleEncoder m_pivotEncoder;
    private final PIDController m_pivotPIDController;
    // For encoder unwrap/continuous angle tracking
    private double m_lastRawEncoder = 0.0;
    private double m_unwrappedRevolutions = 0.0;
    // Optional zero/reference offset in revolutions
    private double m_encoderZeroOffset = 0.0;

    public IntakeSubsystem() {
        m_intakeMotorFx = new TalonFX(Constants.CanIds.IntakeCanIds.kIntakeMotorCanId);
        m_pivotMotor = new SparkMax(Constants.CanIds.IntakeCanIds.kPivotMotorCanId, MotorType.kBrushed);
        m_pivotEncoder = new DutyCycleEncoder(IntakeSubsystemConstants.kPivotEncoderPort);
        m_pivotPIDController = new PIDController(
            IntakeSubsystemConstants.kP,
            IntakeSubsystemConstants.kI,
            IntakeSubsystemConstants.kD
        );

        m_intakeMotorFx.setNeutralMode(NeutralModeValue.Brake);
        m_intakeMotorFx.setSafetyEnabled(true);
        m_intakeMotorFx.setExpiration(0.1);

        m_pivotMotor.configure(
            Configs.IntakeSubsystem.pivotConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        m_pivotPIDController.setTolerance(DEFAULT_PIVOT_TOLERANCE);

    // Dashboard defaults are in degrees now (0..360). Team can edit these live.
    SmartDashboard.putNumber(SD_STOW_POSITION, DEFAULT_STOW_POSITION);
    SmartDashboard.putNumber(SD_RAMP_POSITION, DEFAULT_RAMP_POSITION);
    SmartDashboard.putNumber(SD_INTAKE_POSITION, DEFAULT_INTAKE_POSITION);
        SmartDashboard.putNumber(SD_PIVOT_TOLERANCE, DEFAULT_PIVOT_TOLERANCE);
        SmartDashboard.putNumber(SD_PIVOT_UP_MAX, DEFAULT_PIVOT_UP_MAX_OUTPUT);
        SmartDashboard.putNumber(SD_PIVOT_DOWN_MAX, DEFAULT_PIVOT_DOWN_MAX_OUTPUT);
        SmartDashboard.putString(SD_PIVOT_TARGET_NAME, "NONE");
        SmartDashboard.putBoolean(SD_PIVOT_MANUAL_MODE, false);

        setDefaultCommand(
            runOnce(() -> {
                stopIntakeMotor();
                stopPivotMotor();
            }).andThen(run(() -> {
                m_intakeMotorFx.feed();
            })).withName("Idle")
        );
    }

    @Override
    public void periodic() {
        m_intakeMotorFx.feed();

        m_pivotPIDController.setTolerance(getPivotTolerance());

        // Update unwrapped encoder tracking before publishing values
        updateEncoderUnwrap();

        // Publish angle in degrees for easier tuning
        SmartDashboard.putNumber(SD_PIVOT_ENCODER, getPivotAngleDegrees());
        SmartDashboard.putBoolean(SD_PIVOT_CONNECTED, isPivotEncoderConnected());
        SmartDashboard.putNumber(SD_PIVOT_MOTOR_OUTPUT, m_pivotMotor.get());
        SmartDashboard.putBoolean(SD_PIVOT_AT_SETPOINT, m_pivotPIDController.atSetpoint());
    }

    public double getPivotEncoderPosition() {
        // For backwards compatibility return the raw duty cycle value
        return m_pivotEncoder.get();
    }

    public boolean isPivotEncoderConnected() {
        return m_pivotEncoder.isConnected();
    }

    public double getStowPosition() {
        // Dashboard stores degrees now
        return SmartDashboard.getNumber(SD_STOW_POSITION, DEFAULT_STOW_POSITION);
    }

    public double getRampPosition() {
        return SmartDashboard.getNumber(SD_RAMP_POSITION, DEFAULT_RAMP_POSITION);
    }

    public double getIntakePosition() {
        return SmartDashboard.getNumber(SD_INTAKE_POSITION, DEFAULT_INTAKE_POSITION);
    }

    public double getPivotTolerance() {
        return SmartDashboard.getNumber(SD_PIVOT_TOLERANCE, DEFAULT_PIVOT_TOLERANCE);
    }

    public double getPivotUpMaxOutput() {
        return SmartDashboard.getNumber(SD_PIVOT_UP_MAX, DEFAULT_PIVOT_UP_MAX_OUTPUT);
    }

    public double getPivotDownMaxOutput() {
        return SmartDashboard.getNumber(SD_PIVOT_DOWN_MAX, DEFAULT_PIVOT_DOWN_MAX_OUTPUT);
    }

    public double getPresetPosition(PivotPreset preset) {
        switch (preset) {
            case STOW:
                return getStowPosition();
            case RAMP:
                return getRampPosition();
            case INTAKE:
                return getIntakePosition();
            default:
                return getStowPosition();
        }
    }

    public String getPresetName(PivotPreset preset) {
        switch (preset) {
            case STOW:
                return "STOW";
            case RAMP:
                return "RAMP";
            case INTAKE:
                return "INTAKE";
            default:
                return "UNKNOWN";
        }
    }

    public void stopPivotMotor() {
        m_pivotMotor.set(0.0);
    }

    public void stopIntakeMotor() {
        m_intakeMotorFx.set(0.0);
    }

    public void resetPivotPID() {
        m_pivotPIDController.reset();
    }

    private double clampPivotOutput(double output) {
        return MathUtil.clamp(output, getPivotDownMaxOutput(), getPivotUpMaxOutput());
    }

    public double calculatePivotOutput(double setpoint) {
        // PID operates in degrees. Ensure we compare angles properly by using
        // the continuous angle from the encoder and wrapping the error to [-180,180].
        double currentDeg = getPivotAngleDegrees();

        // Compute minimal angle difference (setpoint and current are in degrees)
        double error = ((setpoint - currentDeg + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;

        double pidOutput = m_pivotPIDController.calculate(0.0, error);
        pidOutput = clampPivotOutput(pidOutput);

        SmartDashboard.putNumber(SD_PIVOT_SETPOINT, setpoint);
        SmartDashboard.putNumber(SD_PIVOT_PID_OUTPUT, pidOutput);

        return pidOutput;
    }

    // ------------------ Encoder unwrap / helpers ------------------
    /**
     * Call periodically to update internal unwrap state. Handles crossing the
     * 0/1 duty-cycle wrap and maintains a continuous revolution count.
     */
    private void updateEncoderUnwrap() {
        double raw = m_pivotEncoder.get();
        // If this is the first time, initialize
        if (m_lastRawEncoder == 0.0 && m_unwrappedRevolutions == 0.0) {
            m_lastRawEncoder = raw;
            return;
        }

        double delta = raw - m_lastRawEncoder;
        // Detect large jumps across the 0/1 boundary
        if (delta > 0.5) {
            // wrapped negative (e.g., 0.99 -> 0.01 gives delta ~ -0.98)
            m_unwrappedRevolutions -= 1.0;
        } else if (delta < -0.5) {
            // wrapped positive
            m_unwrappedRevolutions += 1.0;
        }

        m_lastRawEncoder = raw;
    }

    /**
     * Returns the continuous pivot position in revolutions (can be negative/large).
     */
    public double getPivotRevolutions() {
        // raw in [0,1) plus unwrapped revolutions, minus any zero offset
        return (m_lastRawEncoder + m_unwrappedRevolutions) - m_encoderZeroOffset;
    }

    /**
     * Returns the pivot angle in degrees in a continuous domain (can be outside 0..360).
     */
    public double getPivotAngleDegrees() {
        return getPivotRevolutions() * 360.0;
    }

    /**
     * Reset the current position to be treated as zero reference.
     */
    public void zeroPivotReference() {
        // Ensure unwrap is up-to-date
        updateEncoderUnwrap();
        m_encoderZeroOffset = m_lastRawEncoder + m_unwrappedRevolutions;
    }

    public Command movePivotToPosition(double setpoint) {
        return runOnce(() -> {
                SmartDashboard.putBoolean(SD_PIVOT_MANUAL_MODE, false);
                SmartDashboard.putString(SD_PIVOT_TARGET_NAME, "CUSTOM");
                resetPivotPID();
            })
            .andThen(
                run(() -> {
                    if (!isPivotEncoderConnected()) {
                        stopPivotMotor();
                        return;
                    }

                    double output = calculatePivotOutput(setpoint);
                    m_pivotMotor.set(output);
                })
                .until(m_pivotPIDController::atSetpoint)
            )
            .finallyDo(interrupted -> stopPivotMotor())
            .withName("MovePivotToPosition");
    }

    public Command moveToPreset(PivotPreset preset) {
        return runOnce(() -> {
                SmartDashboard.putBoolean(SD_PIVOT_MANUAL_MODE, false);
                SmartDashboard.putString(SD_PIVOT_TARGET_NAME, getPresetName(preset));
                resetPivotPID();
            })
            .andThen(movePivotToPosition(getPresetPosition(preset)))
            .withName("MoveTo" + getPresetName(preset));
    }

    public Command moveToStowPosition() {
        return moveToPreset(PivotPreset.STOW);
    }

    public Command moveToRampPosition() {
        return moveToPreset(PivotPreset.RAMP);
    }

    public Command moveToIntakePosition() {
        return moveToPreset(PivotPreset.INTAKE);
    }

    public void autonMovePivotToPositionStep(double setpoint) {
        if (!isPivotEncoderConnected()) {
            stopPivotMotor();
            return;
        }

        double output = calculatePivotOutput(setpoint);
        m_pivotMotor.set(output);
    }

    public boolean isPivotAtPosition(double setpoint) {
        return Math.abs(getPivotEncoderPosition() - setpoint) <= getPivotTolerance();
    }

    public boolean isPivotAtPreset(PivotPreset preset) {
        return isPivotAtPosition(getPresetPosition(preset));
    }

    public void autonStopPivotMotor() {
        stopPivotMotor();
    }

    public void autonMoveToStowPositionStep() {
        autonMovePivotToPositionStep(getStowPosition());
    }

    public void autonMoveToRampPositionStep() {
        autonMovePivotToPositionStep(getRampPosition());
    }

    public void autonMoveToIntakePositionStep() {
        autonMovePivotToPositionStep(getIntakePosition());
    }

    public Command movePivot(double speed) {
        return startEnd(
            () -> {
                SmartDashboard.putBoolean(SD_PIVOT_MANUAL_MODE, true);
                SmartDashboard.putString(SD_PIVOT_TARGET_NAME, "MANUAL");
                resetPivotPID();
                m_pivotMotor.set(speed);
            },
            () -> stopPivotMotor()
        ).withName("MovePivot");
    }

    public Command moveIntake(double speed) {
        return startEnd(
            () -> m_intakeMotorFx.set(speed),
            () -> stopIntakeMotor()
        ).withName("MoveIntake");
    }

    public void autonMoveIntake(double speed) {
        m_intakeMotorFx.set(speed);
    }
}