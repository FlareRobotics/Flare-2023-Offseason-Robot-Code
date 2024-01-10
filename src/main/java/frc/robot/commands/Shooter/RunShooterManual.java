package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeFeederConstants;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RunShooterManual extends CommandBase{
    private ShooterSubsystem shooterSubsystem;
    private IntakeFeederSubsystem intakeFeederSubsystem;
    private int goalRPM;

    public RunShooterManual(ShooterSubsystem subsystem, IntakeFeederSubsystem intakeFeederSubsystem, int RPMGoal)
    {
        this.shooterSubsystem = subsystem;
        this.intakeFeederSubsystem = intakeFeederSubsystem;
        this.goalRPM = RPMGoal;
        addRequirements(subsystem, intakeFeederSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Run Shooter Manual Start");
    }

    @Override
    public void execute() {
        shooterSubsystem.setShooterRPM(goalRPM);

        if(shooterSubsystem.getRPMReached(goalRPM))
        {
            intakeFeederSubsystem.setIntakeFeederSpeed(IntakeFeederConstants.intakeFeederFeedSpeedPercentage);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setShooterRPM(0);
        intakeFeederSubsystem.setIntakeFeederSpeed(0);
        System.out.println("Run Shooter Manual End");
    }

    @Override
    public boolean isFinished() {
        return !intakeFeederSubsystem.getIntakeFeedeEndSensor() && !intakeFeederSubsystem.getIntakeFeederSensor();
    }
}
