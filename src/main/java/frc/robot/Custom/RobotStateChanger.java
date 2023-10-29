package frc.robot.Custom;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotStateChanger extends CommandBase {
    public int istenen_state;

    public RobotStateChanger(int Robotstate) {
        istenen_state = Robotstate;
    }

    @Override
    public void initialize() {
        System.out.println("Robot State Changer START");

        switch (istenen_state) {
            case 0:
                RobotContainer.currentState = RobotState.LedCLose;
                break;
            case 1:
                RobotContainer.currentState = RobotState.Balanced;
                DriveSubsystem.setX();
                break;
            case 2:
                RobotContainer.currentState = RobotState.NotBalanced;
                break;
        }
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Robot State changer End!");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
