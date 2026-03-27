package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {

    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);

            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor)
                    .velocityConversionFactor(drivingFactor / 60.0);

            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);

            turningConfig.absoluteEncoder
                    .inverted(true)
                    .positionConversionFactor(turningFactor)
                    .velocityConversionFactor(turningFactor / 60.0)
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class IntakeSubsystem {
        // ✅ NEW: intake motor config (Vortex)
        public static final SparkMaxConfig intakeConfig = new SparkMaxConfig();

        // Existing pivot config
        public static final SparkMaxConfig pivotConfig = new SparkMaxConfig();

        static {
            // ✅ Intake motor (roller)
            intakeConfig
                .inverted(false)              // flip if spinning wrong way
                .idleMode(IdleMode.kCoast)    // intake should usually coast
                .smartCurrentLimit(40);       // Vortex can handle more current

            // Optional: smoother startup (prevents brownouts)
            intakeConfig.openLoopRampRate(0.25);

            // ✅ Pivot motor
            pivotConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(30);
        }
    }
}