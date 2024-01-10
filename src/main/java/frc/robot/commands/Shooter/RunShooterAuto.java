package frc.robot.commands.Shooter;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.LimelightHelpers;
import frc.robot.Constants.IntakeFeederConstants;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterAuto extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private IntakeFeederSubsystem intakeFeederSubsystem;
    private long goalRPM;

    public RunShooterAuto(ShooterSubsystem subsystem, IntakeFeederSubsystem intakeFeederSubsystem)
    {
        this.shooterSubsystem = subsystem;
        this.intakeFeederSubsystem = intakeFeederSubsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Run Shooter Auto Start");
    }

    @Override
    public void execute() {
        if(LimelightHelpers.getFiducialID("Limelight") < 0)
        {
            System.out.println("No Target Found!");
            return;
        }
        
        Pose3d aprilTagPose3D = LimelightHelpers.getTargetPose3d_RobotSpace("Limelight");
        double targetDistance = aprilTagPose3D.getZ();
        goalRPM = shooterSubsystem.calculateRPM(targetDistance);

        if(shooterSubsystem.getRPMReached(goalRPM))
        {
            intakeFeederSubsystem.setIntakeFeederSpeed(IntakeFeederConstants.intakeFeederFeedSpeedPercentage);
        }

        shooterSubsystem.setShooterRPM(goalRPM);
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterRPM(0);
        intakeFeederSubsystem.setIntakeFeederSpeed(0);
        System.out.println("Run Shooter Auto End");
    }

    @Override
    public boolean isFinished() {
        return !intakeFeederSubsystem.getIntakeFeedeEndSensor() && !intakeFeederSubsystem.getIntakeFeederSensor();
    }
}
