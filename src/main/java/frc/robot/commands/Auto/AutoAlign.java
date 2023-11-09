package frc.robot.commands.Auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.SwerveConstants;
import frc.robot.SwerveConstants.DriveConstants;
import frc.robot.lib.VisionTarget;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FlareVisionSubsystem;

public class AutoAlign extends CommandBase {
  DriveSubsystem swerve;

  PIDController sidewaysController = new PIDController(0.03, 0, 0);

  VisionTarget lowerCone = new VisionTarget(61, 12, 20);

  public AutoAlign(DriveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    sidewaysController.setTolerance(1);
  }

  @Override
  public void initialize() {
    FlareVisionSubsystem.ledSpx.set(1);
  }

  @Override
  public void execute() {

    if(Constants.enableSmartDashboard)
    {
      SmartDashboard.putBoolean("Sideways At Setpoint", sidewaysController.atSetpoint());
    }
    
  

    if(FlareVisionSubsystem.getBestTarget() == null)
    {
      
      return;
    }

     double output = SwerveConstants.DriveConstants.kMaxSpeedMetersPerSecond / 10 * 1.
         * sidewaysController.calculate(FlareVisionSubsystem.getDistanceToGoal(FlareVisionSubsystem.getBestTarget()));

    if (Math.abs(FlareVisionSubsystem.getDistanceToGoal(FlareVisionSubsystem.getBestTarget())) >= 1.5d) {
       swerve.drive(0, output, 0, false, true, true);
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
