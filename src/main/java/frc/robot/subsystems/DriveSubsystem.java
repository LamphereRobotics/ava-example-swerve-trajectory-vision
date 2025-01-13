// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class DriveSubsystem extends SubsystemBase {
  // Robot swerve modules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveEncoderPorts,
      DriveConstants.kFrontLeftTurningEncoderPorts,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed);

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDriveMotorPort,
      DriveConstants.kRearLeftTurningMotorPort,
      DriveConstants.kRearLeftDriveEncoderPorts,
      DriveConstants.kRearLeftTurningEncoderPorts,
      DriveConstants.kRearLeftDriveEncoderReversed,
      DriveConstants.kRearLeftTurningEncoderReversed);

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveEncoderPorts,
      DriveConstants.kFrontRightTurningEncoderPorts,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed);

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDriveMotorPort,
      DriveConstants.kRearRightTurningMotorPort,
      DriveConstants.kRearRightDriveEncoderPorts,
      DriveConstants.kRearRightTurningEncoderPorts,
      DriveConstants.kRearRightDriveEncoderReversed,
      DriveConstants.kRearRightTurningEncoderReversed);

  // The gyro sensor
  private final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  // Pose estimator class for estimating robot pose with swerve module encoders
  // and vision
  private final SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      m_gyro.getRotation2d(),
      getModulePositions(),
      Pose2d.kZero,
      DriveConstants.kStateStdDevs,
      LimelightConstants.kMegaTag2VisionMeasurementStdDevs);

  // Create config for trajectory
  private final TrajectoryConfig m_trajectoryConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics);

  private final ProfiledPIDController m_thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void periodic() {
    // Update the pose estimator in the periodic block
    m_poseEstimator.update(
        m_gyro.getRotation2d(),
        getModulePositions());

    // Add vision measurement to pose estimate using limelight
    if (LimelightConstants.kUseMegaTag2) {
      useMegaTag2VisionEstimate();
    } else {
      useMegaTag1VisionEstimate();
    }
  }

  private void useMegaTag1VisionEstimate() {
    boolean doRejectUpdate = false;

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
        .getBotPoseEstimate_wpiBlue(LimelightConstants.kLimelightName);

    if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
      if (mt1.rawFiducials[0].ambiguity > .7) {
        doRejectUpdate = true;
      }
      if (mt1.rawFiducials[0].distToCamera > 3) {
        doRejectUpdate = true;
      }
    }
    if (mt1.tagCount == 0) {
      doRejectUpdate = true;
    }

    if (!doRejectUpdate) {
      m_poseEstimator.setVisionMeasurementStdDevs(LimelightConstants.kMegaTag1VisionMeasurementStdDevs);
      m_poseEstimator.addVisionMeasurement(
          mt1.pose,
          mt1.timestampSeconds);
    }
  }

  private void useMegaTag2VisionEstimate() {
    boolean doRejectUpdate = false;

    LimelightHelpers.SetRobotOrientation(LimelightConstants.kLimelightName,
        m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
        .getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.kLimelightName);
    if (Math.abs(m_gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                          // vision updates
    {
      doRejectUpdate = true;
    }
    if (mt2.tagCount == 0) {
      doRejectUpdate = true;
    }
    if (!doRejectUpdate) {
      m_poseEstimator.setVisionMeasurementStdDevs(LimelightConstants.kMegaTag2VisionMeasurementStdDevs);
      m_poseEstimator.addVisionMeasurement(
          mt2.pose,
          mt2.timestampSeconds);
    }
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the pose estimate to the specified pose.
   *
   * @param pose The pose to which to set the pose estimate.
   */
  public void resetPoseEstimate(Pose2d pose) {
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(),
        getModulePositions(),
        pose);
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
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
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            DriveConstants.kDrivePeriod));
    setModuleStates(swerveModuleStates);
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
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public Trajectory buildTrajectory(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end) {
    return TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, m_trajectoryConfig);
  }

  public SwerveControllerCommand followTrajectoryCommand(Trajectory trajectory) {
    return new SwerveControllerCommand(
        trajectory,
        this::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        m_thetaController,
        this::setModuleStates,
        this);
  }
}
