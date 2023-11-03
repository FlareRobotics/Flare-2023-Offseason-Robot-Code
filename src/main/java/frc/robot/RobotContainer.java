package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Custom.Distance_State;
import frc.robot.Custom.ResetRobot;
import frc.robot.Custom.RobotState;
import frc.robot.Custom.RobotStateChanger;
import frc.robot.Custom.SupplyGather;
import frc.robot.SwerveConstants.OIConstants;
import frc.robot.commands.Arm.AutoArm;
import frc.robot.commands.Arm.ManuelArm;
import frc.robot.commands.Auto.ClimbPigeon;
import frc.robot.commands.Claw.ClawSet;
import frc.robot.commands.Claw.ToggleCompressor;
import frc.robot.commands.Elevator.AutoElevator;
import frc.robot.commands.Elevator.ManuelElevator;
import frc.robot.commands.Led.LedController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.CompressorSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LedSubsystem;

//8054 <3
public class RobotContainer {

        public static XboxController driver_main = new XboxController(1);
        public static XboxController driver_2 = new XboxController(2);

        private static final ElevatorSubsystem elevatorsubsystem = new ElevatorSubsystem();
        private static final ClawSubsystem clawSubsystem = new ClawSubsystem();
        private static final CompressorSubsystem compressorSubsystem = new CompressorSubsystem();
        private static final ArmSubsystem armSubsystem = new ArmSubsystem();
        private static final DriveSubsystem m_robotDrive = new DriveSubsystem();
        public final static LedSubsystem ledSubsystem = new LedSubsystem();

        public static SendableChooser<Integer> autoChooser = new SendableChooser<>();

        public static boolean clawOpen = true;
        public static RobotState currentState = RobotState.None;
        public static boolean wantedCone = false;


        public RobotContainer() {
                configureButtonBindings();

                // BN = Blue Normal, BB = Blue bump
                autoChooser.addOption("BN Cube Mobility", 0);
                autoChooser.addOption("BN Cube Mobility Balance ", 1);
                autoChooser.addOption("BB Cube Mobility", 2);
                autoChooser.addOption("BB Cube Mobility Balance", 8);

                autoChooser.addOption("RN Cube Mobility", 3);
                autoChooser.addOption("RN Cube Mobility Balance", 4);
                autoChooser.addOption("RB Cube Mobility", 5);
                autoChooser.addOption("RB Cube Mobility Balance", 9);

                autoChooser.addOption("RM Cube Balance", 10);
                autoChooser.addOption("BM Cube Balance", 11);

                autoChooser.setDefaultOption("No Auto", 6);

                autoChooser.addOption("Only Cube", 7);

                SmartDashboard.putData(autoChooser);

                ledSubsystem.setDefaultCommand(new LedController(ledSubsystem));
                // Configure the button bindings

                // Configure default commands
                m_robotDrive.setDefaultCommand(
                                // The left stick controls translation of the robot.
                                // Turning is controlled by the X axis of the right stick.
                                new RunCommand(
                                                () -> m_robotDrive.drive(
                                                                -MathUtil.applyDeadband(driver_main.getLeftY(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driver_main.getLeftX(),
                                                                                OIConstants.kDriveDeadband),
                                                                -MathUtil.applyDeadband(driver_main.getRightX(),
                                                                                OIConstants.kDriveDeadband),
                                                                true, true, true),
                                                m_robotDrive));
        }

        private void configureButtonBindings() {
                // Reset Heading for Field Relative
                new JoystickButton(driver_main, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new RunCommand(() -> DriveSubsystem.m_gyro.setYaw(0)));
                                
                //Shifter
                new JoystickButton(driver_main, XboxController.Axis.kLeftTrigger.value)
                .whileTrue(new RunCommand(() -> DriveSubsystem.shifterReduction = 2));

                new JoystickButton(driver_main, XboxController.Axis.kLeftTrigger.value)
                .onFalse(new RunCommand(() -> DriveSubsystem.shifterReduction = 1));

                // Manuel Elevator
                new JoystickButton(driver_2, 8)
                                .whileTrue(new ManuelElevator(elevatorsubsystem, true));
                new JoystickButton(driver_2, 7)
                                .whileTrue(new ManuelElevator(elevatorsubsystem, false));

                // Manuel Arm
                new JoystickButton(driver_main, XboxController.Button.kB.value)
                                .whileTrue(new ManuelArm(armSubsystem, true));
                new JoystickButton(driver_main, XboxController.Button.kX.value)
                                .whileTrue(new ManuelArm(armSubsystem, false));

                // Compressor Toggle
                new JoystickButton(driver_main, XboxController.Button.kStart.value)
                                .toggleOnTrue(new ToggleCompressor(compressorSubsystem));

                // Claw For Cone
                new JoystickButton(driver_main, XboxController.Button.kRightBumper.value)
                                .toggleOnTrue(new ClawSet(clawSubsystem));

                // Wanted Status
                new JoystickButton(driver_2, 5).toggleOnTrue(new SupplyGather(ledSubsystem));

                // Reset Encoders
                new JoystickButton(driver_2, 9)
                                .whileTrue(new ResetRobot(armSubsystem, elevatorsubsystem, clawSubsystem));

                // Led Close
                new JoystickButton(driver_2, 11).whileTrue(new RobotStateChanger(0));

                // Substation 
                new JoystickButton(driver_2, 6).toggleOnTrue(
                                new ParallelCommandGroup(
                                                new AutoElevator(elevatorsubsystem, Distance_State.Substation_Elevator),
                                                new SequentialCommandGroup(new WaitCommand(1d),
                                                                new AutoArm(armSubsystem,
                                                                                Distance_State.Substation_Arm))));

                // Cube Teleop?
                new JoystickButton(driver_2, 1).toggleOnTrue(new ParallelCommandGroup(
                                new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cube_Elevator),
                                new SequentialCommandGroup(
                                                new WaitCommand(.5),
                                                new AutoArm(armSubsystem, Distance_State.Middle_Cone_Arm))));

                // Koni teleop?
                new JoystickButton(driver_2, 4).toggleOnTrue(new ParallelCommandGroup(
                                new AutoElevator(elevatorsubsystem, Distance_State.Middle_Cone_Elevator_Auto),
                                new SequentialCommandGroup(
                                                new WaitCommand(.5),
                                                new AutoArm(armSubsystem, Distance_State.Middle_Cone_Arm))));

                // Reset Arm And Elevator
                new JoystickButton(driver_2, 2).toggleOnTrue(new ParallelCommandGroup(
                                new AutoArm(armSubsystem, Distance_State.Zero_All),
                                new SequentialCommandGroup(
                                                new WaitCommand(.8),
                                                new AutoElevator(elevatorsubsystem, Distance_State.Zero_All))));

                // Reset arm only
                new JoystickButton(driver_2, 3).toggleOnTrue(new AutoArm(armSubsystem, Distance_State.Zero_All));


        }

        public Command getAutonomousCommand() {

                if (autoChooser.getSelected().equals(7)) // Cube only
                {
                        return placeCubeCommand()
                                        .andThen(new RobotStateChanger(1));
                } 
                else if (autoChooser.getSelected().equals(0)) // Blue Normal Mobility
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Blue_Normal_Mobility",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj)
                                                        .andThen(new RobotStateChanger(1)));
                } 
                else if (autoChooser.getSelected().equals(2)) // Blue Bump Mobility
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Blue_Bump_Mobility",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj)
                                                        .andThen(new RobotStateChanger(1)));
                } 
                else if (autoChooser.getSelected().equals(8)) // Blue Bump Mobility Balance
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Blue_Bump_MobilityBalance",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj)
                                                        .andThen(new RobotStateChanger(1)));
                } 
                else if (autoChooser.getSelected().equals(10)) // Red Middle Balance
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Red_Middle_Balance",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj)
                                                        .andThen(new RobotStateChanger(1)));
                } 
                else if (autoChooser.getSelected().equals(11)) // Blue Middle Balance
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Blue_Middle_Balance",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj)
                                                        .andThen(new RobotStateChanger(1)));
                } 
                else if (autoChooser.getSelected().equals(9)) // Red Bump Mobility Balance
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Red_Bump_MobilityBalance",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj)
                                                        .andThen(new RobotStateChanger(1)));
                } 
                else if (autoChooser.getSelected().equals(1)) // Cube + Mobility + Balance
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Blue_Normal_MobilityBalance",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj))
                                        .andThen(new ClimbPigeon(m_robotDrive, -0.5))
                                        .andThen(new RobotStateChanger(1));
                } 
                else if (autoChooser.getSelected().equals(3)) // Red Normal Mobility
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Red_Normal_Mobility",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj)
                                                        .andThen(new RobotStateChanger(1)));
                } 
                else if (autoChooser.getSelected().equals(5)) // Red Bump Mobility
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Red_Bump_Mobility",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj)
                                                        .andThen(new RobotStateChanger(1)));
                } 
                else if (autoChooser.getSelected().equals(4)) // Red + Mobility + Balance
                {
                        PathPlannerTrajectory traj = PathPlanner.loadPath("Red_Normal_MobilityBalance",
                                        new PathConstraints(4, 3));
                        m_robotDrive.resetOdometry(traj.getInitialHolonomicPose());
                        return placeCubeCommand()
                                        .andThen(getTraj(traj))
                                        .andThen(new ClimbPigeon(m_robotDrive, -0.5))
                                        .andThen(new RobotStateChanger(1));
                }

                return new RobotStateChanger(2); // No Auto
        }

        private PPSwerveControllerCommand getTraj(PathPlannerTrajectory trajectory) {
                PPSwerveControllerCommand command = new PPSwerveControllerCommand(trajectory,
                                m_robotDrive::getPose,
                                m_robotDrive.xController, m_robotDrive.yController, m_robotDrive.thetaController,
                                m_robotDrive::setSpeeds, m_robotDrive);

                return command;
        }

        private Command placeCubeCommand() {
                return new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                                new AutoElevator(elevatorsubsystem,
                                                                Distance_State.Middle_Cube_Elevator),
                                                new SequentialCommandGroup(
                                                                new WaitCommand(.5),
                                                                new AutoArm(armSubsystem,
                                                                                Distance_State.Middle_Cube_Arm))),
                                new ClawSet(clawSubsystem).withTimeout(0.5d),
                                new ParallelCommandGroup(new AutoArm(armSubsystem, Distance_State.Zero_All),
                                                new AutoElevator(elevatorsubsystem, Distance_State.Zero_All)
                                                                .beforeStarting(new WaitCommand(0.5d))));
        }
}
