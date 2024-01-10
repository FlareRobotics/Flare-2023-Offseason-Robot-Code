package frc.robot.commands.IntakeFeeder;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeFeederConstants;
import frc.robot.subsystems.IntakeFeederSubsystem;

public class RunIntakeFeeder extends CommandBase{
    private IntakeFeederSubsystem intakeFeederSubsystem;
    private boolean inside;

    public RunIntakeFeeder(IntakeFeederSubsystem subsystem, boolean inside)
    {
        this.intakeFeederSubsystem = subsystem;
        this.inside = inside;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Intake Feeder Run Start: " + inside);
    }

    @Override
    public void execute() {
        intakeFeederSubsystem.setIntakeFeederSpeed(IntakeFeederConstants.intakeFeederGroundSpeedPercentage * (inside ? 1 : -1));
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Intake Feeder Run End: " + interrupted);
        intakeFeederSubsystem.setIntakeFeederSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return intakeFeederSubsystem.getIntakeFeederSensor();
    }
}
