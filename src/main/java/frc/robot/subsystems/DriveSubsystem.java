// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.ctre.phoenix.sensors.PigeonIMU;

import frc.robot.Constants.CanIds;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      CanIds.DriveCanIds.kFrontLeftDrivingCanId,
      CanIds.DriveCanIds.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      CanIds.DriveCanIds.kFrontRightDrivingCanId,
      CanIds.DriveCanIds.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      CanIds.DriveCanIds.kRearLeftDrivingCanId,
      CanIds.DriveCanIds.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      CanIds.DriveCanIds.kRearRightDrivingCanId,
      CanIds.DriveCanIds.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor (CTRE Pigeon on CAN bus)
  private final PigeonIMU m_gyro = new PigeonIMU(frc.robot.Constants.DriveConstants.kPigeonCanId);
  // Fallback state for detecting stale/no CAN frames from the Pigeon
  private double m_lastHeading = 0.0; // degrees
  private int m_consecutiveStaleGyro = 0;
  private static final int kStaleThreshold = 3; // number of consecutive stale reads before falling back

  // NetworkTables entries for Elastic dashboard
  private final NetworkTable m_driveTable = NetworkTableInstance.getDefault().getTable("Drive");
  private final NetworkTableEntry m_headingEntry = m_driveTable.getEntry("Heading");
  private final NetworkTableEntry m_frontLeftAngleEntry = m_driveTable.getEntry("FrontLeftAngle");
  private final NetworkTableEntry m_frontRightAngleEntry = m_driveTable.getEntry("FrontRightAngle");
  private final NetworkTableEntry m_rearLeftAngleEntry = m_driveTable.getEntry("RearLeftAngle");
  private final NetworkTableEntry m_rearRightAngleEntry = m_driveTable.getEntry("RearRightAngle");

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
  Rotation2d.fromDegrees(getHeading()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }
    );
    // Publish heading and module absolute angles to NetworkTables (Elastic uses these keys)
    m_headingEntry.setDouble(getHeading());
    m_frontLeftAngleEntry.setDouble(m_frontLeft.getPosition().angle.getDegrees());
    m_frontRightAngleEntry.setDouble(m_frontRight.getPosition().angle.getDegrees());
    m_rearLeftAngleEntry.setDouble(m_rearLeft.getPosition().angle.getDegrees());
    m_rearRightAngleEntry.setDouble(m_rearRight.getPosition().angle.getDegrees());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
  Rotation2d.fromDegrees(getHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

  var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
    fieldRelative
      ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
        Rotation2d.fromDegrees(getHeading()))
      : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // PigeonIMU provides setYaw to reset the yaw to a specific value.
    // Ignore the returned ErrorCode for simplicity.
    m_gyro.setYaw(0.0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
  // Some gyros report the sign opposite of WPILib expectations depending on
  // mounting orientation. Use a fixed negation and then apply kGyroReversed
  // to allow flipping without changing code elsewhere.
  // Use the PigeonIMU yaw value. getYawPitchRoll fills an array with
  // [yaw, pitch, roll] (degrees). If the call fails the array will still be
  // used but may contain zeros.
  double[] ypr = new double[3];
  m_gyro.getYawPitchRoll(ypr);
  double raw = ypr[0];

  // Detect all-zero (likely stale) readings. It's possible for a real
  // heading to be very near zero, but repeated consecutive exact-zero
  // readings are a good heuristic for a failed JNI/CAN read (logs show
  // frequent 'CAN frame not received/too-stale' messages).
  boolean allZero = Math.abs(ypr[0]) < 1e-6 && Math.abs(ypr[1]) < 1e-6 && Math.abs(ypr[2]) < 1e-6;
  if (allZero) {
    m_consecutiveStaleGyro++;
    if (m_consecutiveStaleGyro > kStaleThreshold) {
      // Return last-known heading (normalized) while hardware is flaky
      double normalized = ((m_lastHeading + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
      return normalized;
    }
  } else {
    // Good reading: reset stale counter and update last-known heading
    m_consecutiveStaleGyro = 0;
    m_lastHeading = -raw * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  double angle = -raw; // preserve previous negation behavior
  angle = angle * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  // Normalize to [-180, 180)
  double normalized = ((angle + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
  return normalized;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
  // Match sign convention used by getHeading() (negated above)
  // PigeonIMU.getRawGyro fills array with angular rates [x, y, z] in
  // degrees/sec. The z element is the yaw rate.
  double[] rawRates = new double[3];
  m_gyro.getRawGyro(rawRates);
  boolean allZeroRates = Math.abs(rawRates[0]) < 1e-6 && Math.abs(rawRates[1]) < 1e-6 && Math.abs(rawRates[2]) < 1e-6;
  if (allZeroRates) {
    // Stale/noisy gyro read — return zero rate until readings recover
    return 0.0;
  }
  double yawRate = rawRates[2];
  // Reset stale counter if we got a valid rate
  m_consecutiveStaleGyro = 0;
  return -yawRate * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
}
