package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase{
    private WPI_VictorSPX climbMotor = new WPI_VictorSPX(ClimbConstants.climbCanID);

    public ClimbSubsystem()
    {
        climbMotor.setInverted(ClimbConstants.climbMotorReversed);
        climbMotor.setNeutralMode(ClimbConstants.climbMotorNeutralMode);
    }

    public void setClimbSpeed(boolean up)
    {
        climbMotor.set(VictorSPXControlMode.PercentOutput, ClimbConstants.climbSpeedPercentage * (up ? 1 : -1));
    }

    public void stopClimb()
    {
        climbMotor.set(0);
    }
}
