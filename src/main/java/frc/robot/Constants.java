package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public final class Constants 
{
    public static boolean enableSmartDashboard = false;

    public static final class IntakeFeederConstants
    {
        public static int intakeFeederCanID = 0;
        public static int intakeFeederSensorID = 0;
        public static int intakeFeederSensorEndID = 0;
        public static double intakeFeederGroundSpeedPercentage = 1;
        public static double intakeFeederFeedSpeedPercentage = 1;
        public static boolean intakeFeederReversed = false;
        public static NeutralMode intakeFeederNeutralMode = NeutralMode.Brake;
    }

    public static final class ShooterConstants
    {
        public static int shooterCanID = 0;
        public static int shooterBaseRPM = 6000;
        public static double shooterBaseDistanceMM = 500;
        public static boolean shooterMotorReversed = false;
        public static NeutralMode shooterMotorNeutralMode = NeutralMode.Brake;
        public static int shooterMotorTimeoutMS = 30;
        public static double shooterMotorKp = 0;
        public static double shooterMotorKi = 0;
        public static double shooterMotorKd = 0;
        public static double RPMTolerance = 100;
    }

    public static final class ClimbConstants
    {
        public static int climbCanID = 0;
        public static boolean climbMotorReversed = false;
        public static NeutralMode climbMotorNeutralMode = NeutralMode.Brake;
        public static double climbSpeedPercentage = 1;
    }

    public static final class PathPosition
    {
        public Pose2d redPose;
        public Pose2d bluePose;

        public PathPosition(Pose2d redPose, Pose2d bluePose)
        {
            this.redPose = redPose;
            this.bluePose = bluePose;
        }
    }

    public static final class PathOnFlyConstants
    {
        PathPosition speakerPosition = new PathPosition(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)));

    }
}