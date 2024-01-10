package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class RunClimb extends CommandBase{
    private ClimbSubsystem climbSubsystem;
    private boolean up;
    private boolean isFinised = false;

    public RunClimb(ClimbSubsystem subsystem, boolean up)
    {
        this.climbSubsystem = subsystem;
        this.up = up;
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        System.out.println("Run Climb Start");
    }

    @Override
    public void execute() {
        climbSubsystem.setClimbSpeed(up);
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.stopClimb();
        System.out.println("Run Climb Stop");
    }

    @Override
    public boolean isFinished() {
        return isFinised;
    }
}
