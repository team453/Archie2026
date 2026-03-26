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
    private static final double DEFAULT_STOW_POSITION = 0.60;
    private static final double DEFAULT_RAMP_POSITION = 0.42;
    private static final double DEFAULT_INTAKE_POSITION = 0.24;

    private static final double DEFAULT_PIVOT_TOLERANCE = 0.015;
    private static final double DEFAULT_PIVOT_UP_MAX_OUTPUT = 0.65;
    private static final double DEFAULT_PIVOT_DOWN_MAX_OUTPUT = -0.25;

    private final TalonFX m_intakeMotorFx;
    private final SparkMax m_pivotMotor;
    private final DutyCycleEncoder m_pivotEncoder;
    private final PIDController m_pivotPIDController;

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

        SmartDashboard.putNumber(SD_PIVOT_ENCODER, getPivotEncoderPosition());
        SmartDashboard.putBoolean(SD_PIVOT_CONNECTED, isPivotEncoderConnected());
        SmartDashboard.putNumber(SD_PIVOT_MOTOR_OUTPUT, m_pivotMotor.get());
        SmartDashboard.putBoolean(SD_PIVOT_AT_SETPOINT, m_pivotPIDController.atSetpoint());
    }

    public double getPivotEncoderPosition() {
        return m_pivotEncoder.get();
    }

    public boolean isPivotEncoderConnected() {
        return m_pivotEncoder.isConnected();
    }

    public double getStowPosition() {
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
        double pidOutput = m_pivotPIDController.calculate(getPivotEncoderPosition(), setpoint);
        pidOutput = clampPivotOutput(pidOutput);

        SmartDashboard.putNumber(SD_PIVOT_SETPOINT, setpoint);
        SmartDashboard.putNumber(SD_PIVOT_PID_OUTPUT, pidOutput);

        return pidOutput;
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