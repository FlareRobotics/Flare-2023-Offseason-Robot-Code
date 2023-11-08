package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.lib.VisionTarget;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlareVisionSubsystem;

public class AutoAlign extends CommandBase {
  /** Creates a new AutoAlign. */
  DriveSubsystem swerve;

  PIDController sidewaysController = new PIDController(0.03, 0, 0);
  PIDController rotationController = new PIDController(0.035, 0, 0);
  PIDController distanceController = new PIDController(0.016, 0, 0);

  VisionTarget lowerCone = new VisionTarget(61, 12, 20);

  public AutoAlign(DriveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    distanceController.setTolerance(1);
    sidewaysController.setTolerance(1);
    rotationController.setTolerance(1.5d);
  }

  @Override
  public void initialize() {
    FlareVisionSubsystem.ledSpx.set(1);
  }

  @Override
  public void execute() {
    if(FlareVisionSubsystem.getBestTarget() == null)
    {
      return;
    }
    
    if(sidewaysController.atSetpoint() && distanceController.atSetpoint())
    {
      RobotContainer.driver_main.setRumble(RumbleType.kBothRumble, 1);
      stopMotors();
      return;
    }

     double rotation = SwerveConstants.DriveConstants.kMaxAngularSpeed / 10
         * rotationController.calculate(DriveSubsystem.m_gyro.getYaw(), 0);
     lowerCone.updatePitch(FlareVisionSubsystem.getYdistance(FlareVisionSubsystem.getBestTarget()));
     double output = SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond / 5 * 1.
         * sidewaysController.calculate(FlareVisionSubsystem.getDistanceToGoal(FlareVisionSubsystem.getBestTarget()));
     double distance = lowerCone.getDistance();

     if (Math.abs(DriveSubsystem.m_gyro.getYaw()) >= 4d) {
       swerve.drive(0, 0, rotation, false, true, true);
     } else if (Math.abs(FlareVisionSubsystem.getDistanceToGoal(FlareVisionSubsystem.getBestTarget())) >= 2.1d) {
       swerve.drive(0, output, rotation, false, true, true);
     } else {  
       double distanceOutput = -distanceController.calculate(distance, 130)
           * SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond / 15;
       swerve.drive(distanceOutput, output, rotation, false, true, true);

     }
  }

  @Override
  public void end(boolean interrupted) {
     stopMotors();
     FlareVisionSubsystem.ledSpx.set(0);
     RobotContainer.driver_main.setRumble(RumbleType.kBothRumble, 0);
  }

  private void stopMotors()
  {
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
         new ChassisSpeeds(0, 0, 0));
     SwerveDriveKinematics.desaturateWheelSpeeds(
         swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
     DriveSubsystem.m_frontLeft.setDesiredState(swerveModuleStates[0], true);
     DriveSubsystem.m_frontRight.setDesiredState(swerveModuleStates[1], true);
     DriveSubsystem.m_rearLeft.setDesiredState(swerveModuleStates[2], true);
     DriveSubsystem.m_rearRight.setDesiredState(swerveModuleStates[3], true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
