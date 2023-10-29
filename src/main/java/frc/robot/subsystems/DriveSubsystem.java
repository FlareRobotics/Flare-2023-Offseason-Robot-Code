// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
//CTRE Imports
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SwerveConstants;
import frc.robot.SwerveConstants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  public static final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      false, true);

      public static final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      false, true);

      public static final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      false, true);

      public static final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      false, true);

  // The gyro sensor
  public static final Pigeon2 m_gyro = new Pigeon2(DriveConstants.gyroCanId);

  // Slew rate filter variables for controlling lateral acceleration


  private SlewRateLimiter m_magXLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);
  private SlewRateLimiter m_magYLimiter = new SlewRateLimiter(DriveConstants.kMaxAcceleration);

  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kMaxAngularAcceleration);

  public PIDController xController = new PIDController(SwerveConstants.AutoConstants.kPXController, 0, 0);
  public PIDController yController = new PIDController(SwerveConstants.AutoConstants.kPYController, 0, 0);
  public PIDController thetaController = new PIDController(SwerveConstants.AutoConstants.kPThetaController, 0, 0);


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getYaw()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      }, new Pose2d(1.95d, 3.3d, Rotation2d.fromDegrees(m_gyro.getYaw())));

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    resetEncoders();
    zeroHeading();
  }

  private final Field2d mField2d = new Field2d();

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(m_gyro.getYaw()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });



    mField2d.setRobotPose(m_odometry.getPoseMeters());

    SmartDashboard.putData(mField2d);
    SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
    SmartDashboard.putNumber("Gyro angle", m_gyro.getYaw());
    

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
        Rotation2d.fromDegrees(m_gyro.getYaw()),
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
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit,
      boolean isOpenLoop) {

    double xSpeedDelivered, ySpeedDelivered, rotDelivered;
    if(rateLimit)
    {
    // Convert the commanded speeds into the correct units for the drivetrain
     xSpeedDelivered = m_magXLimiter.calculate(xSpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
     ySpeedDelivered = m_magYLimiter.calculate(ySpeed) * DriveConstants.kMaxSpeedMetersPerSecond;
     rotDelivered = m_rotLimiter.calculate(rot) * DriveConstants.kMaxAngularSpeed;
    }
    else
    {
      xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
      rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    }
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getYaw()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0], isOpenLoop);
    m_frontRight.setDesiredState(swerveModuleStates[1], isOpenLoop);
    m_rearLeft.setDesiredState(swerveModuleStates[2], isOpenLoop);
    m_rearRight.setDesiredState(swerveModuleStates[3], isOpenLoop);
  }

  public void setSpeeds(ChassisSpeeds speeds)
  {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0], true);
    m_frontRight.setDesiredState(swerveModuleStates[1], true);
    m_rearLeft.setDesiredState(swerveModuleStates[2], true);
    m_rearRight.setDesiredState(swerveModuleStates[3], true);
  }

  public static void setBrake(boolean enabled)
  {
    m_frontLeft.m_drivingTalon.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    m_frontRight.m_drivingTalon.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    m_rearLeft.m_drivingTalon.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    m_rearRight.m_drivingTalon.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
  }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates, boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0], isOpenLoop);
    m_frontRight.setDesiredState(desiredStates[1], isOpenLoop);
    m_rearLeft.setDesiredState(desiredStates[2], isOpenLoop);
    m_rearRight.setDesiredState(desiredStates[3], isOpenLoop);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public static void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public static void zeroHeading() {
    m_gyro.setYaw(0);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getYaw()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   * 
   *         public double getTurnRate() {
   *         return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 :
   *         1.0);
   *         }
   */
}
