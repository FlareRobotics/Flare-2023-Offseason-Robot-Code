package frc.robot.commands.PathOnFly;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PathPosition;
import frc.robot.subsystems.DriveSubsystem;

public class PathOnFlyCommand extends CommandBase {
    private DriveSubsystem sDriveSubsystem;
    private PathPosition path;
    private PathConstraints constraints;
    private boolean finished = false;

    public PathOnFlyCommand(DriveSubsystem driveSubsystem, PathPosition path, PathConstraints pathConstraints)
    {
        this.sDriveSubsystem = driveSubsystem;
        this.path = path;
        this.constraints = pathConstraints;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        boolean blueAlliance = DriverStation.getAlliance() == Alliance.Blue;

        Command autoCommand = sDriveSubsystem.returnPathCommand(blueAlliance ? path.bluePose : path.redPose, constraints);

        autoCommand.schedule();
    }

    @Override
    public void execute() {
        finished = true;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
