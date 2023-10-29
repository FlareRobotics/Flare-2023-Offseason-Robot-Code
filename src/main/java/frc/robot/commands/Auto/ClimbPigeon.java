package frc.robot.commands.Auto;


import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ClimbPigeon extends CommandBase {

  int mps;
  DriveSubsystem subsystem;
  public ClimbPigeon(DriveSubsystem driveSubsystem, int MPS) {
    subsystem = driveSubsystem;
    mps = MPS;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
   
  }

  @Override
  public void execute() {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
      new ChassisSpeeds(0, mps, 0));
  SwerveDriveKinematics.desaturateWheelSpeeds(
      swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
  DriveSubsystem.m_frontLeft.setDesiredState(swerveModuleStates[0], true);
  DriveSubsystem.m_frontRight.setDesiredState(swerveModuleStates[1], true);
  DriveSubsystem.m_rearLeft.setDesiredState(swerveModuleStates[2], true);
  DriveSubsystem.m_rearRight.setDesiredState(swerveModuleStates[3], true);
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

  subsystem.setX();
  }

  @Override
  public boolean isFinished() {
    return DriveSubsystem.m_gyro.getPitch() < 4.5d;
  }
}