package frc.robot;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeFeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

//8054 <3
public class RobotContainer {
        private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
        private static final ShooterSubsystem SHOOTER_SUBSYSTEM = new ShooterSubsystem();
        private static final IntakeFeederSubsystem INTAKE_FEEDER_SUBSYSTEM = new IntakeFeederSubsystem();
        private static final ClimbSubsystem CLIMB_SUBSYSTEM = new ClimbSubsystem();

        CommandXboxController driver_main = new CommandXboxController(0);

        public RobotContainer() {
                configureButtonBindings();

                m_robotDrive.setDefaultCommand(
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(driver_main.getLeftY(),
                                                                                OIConstants.kDriveDeadband) / 1.00d,
                                                                -MathUtil.applyDeadband(driver_main.getLeftX(),
                                                                                OIConstants.kDriveDeadband) / 1.00d,
                                                                -MathUtil.applyDeadband(driver_main.getRightX(),
                                                                                OIConstants.kDriveDeadband) / 2,
                                                                true, true, true),
                                                m_robotDrive));
        }

        private void configureButtonBindings() {

        }

        public Command getAutonomousCommand() {
                return null;
        }

        public static PPSwerveControllerCommand getTraj(PathPlannerTrajectory trajectory) {
                PPSwerveControllerCommand command = new PPSwerveControllerCommand(trajectory,
                                m_robotDrive::getPose,
                                m_robotDrive.xController, m_robotDrive.yController, m_robotDrive.thetaController,
                                m_robotDrive::setSpeeds, m_robotDrive);

                return command;
        }
}
