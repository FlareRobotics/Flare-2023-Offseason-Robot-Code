package frc.robot.commands.Auto;

import frc.robot.RobotContainer;
import frc.robot.Custom.RobotState;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbPigeon extends CommandBase {

  double mps;
  DriveSubsystem subsystem;

  public ClimbPigeon(DriveSubsystem driveSubsystem, double MPS) {
    subsystem = driveSubsystem;
    mps = MPS;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    System.out.println("goonmbbjjhbjhbhvjjvhvvhjjvhhvjhvjvhjjvhvhjvhj");
  }

  @Override
  public void execute() {
    if (Math.abs(DriveSubsystem.m_gyro.getPitch()) > 3d) {
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(mps * (DriveSubsystem.m_gyro.getPitch() > 0 ? -1 : 1), 0, 0,
              Rotation2d.fromDegrees(DriveSubsystem.m_gyro.getYaw())));

      SwerveDriveKinematics.desaturateWheelSpeeds(
          swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
      DriveSubsystem.m_frontLeft.setDesiredState(swerveModuleStates[0], true);
      DriveSubsystem.m_frontRight.setDesiredState(swerveModuleStates[1], true);
      DriveSubsystem.m_rearLeft.setDesiredState(swerveModuleStates[2], true);
      DriveSubsystem.m_rearRight.setDesiredState(swerveModuleStates[3], true);
    }
    else
    {
      var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(0, 0, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    DriveSubsystem.m_frontLeft.setDesiredState(swerveModuleStates[0], true);
    DriveSubsystem.m_frontRight.setDesiredState(swerveModuleStates[1], true);
    DriveSubsystem.m_rearLeft.setDesiredState(swerveModuleStates[2], true);
    DriveSubsystem.m_rearRight.setDesiredState(swerveModuleStates[3], true);

    DriveSubsystem.setBrake(true);

    }
  }

  @Override
  public void end(boolean interrupted) {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(0, 0, 0));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    DriveSubsystem.m_frontLeft.setDesiredState(swerveModuleStates[0], true);
    DriveSubsystem.m_frontRight.setDesiredState(swerveModuleStates[1], true);
    DriveSubsystem.m_rearLeft.setDesiredState(swerveModuleStates[2], true);
    DriveSubsystem.m_rearRight.setDesiredState(swerveModuleStates[3], true);

    DriveSubsystem.setBrake(true);
    System.out.println(interrupted);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}