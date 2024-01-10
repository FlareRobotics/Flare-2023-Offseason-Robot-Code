package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase{
    private TalonFX shooterMotor = new TalonFX(ShooterConstants.shooterCanID);

    public ShooterSubsystem()
    {
        shooterMotor.configFactoryDefault();

        shooterMotor.selectProfileSlot(0, 0);
        shooterMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, ShooterConstants.shooterMotorTimeoutMS);
        shooterMotor.setInverted(ShooterConstants.shooterMotorReversed);
        shooterMotor.setNeutralMode(ShooterConstants.shooterMotorNeutralMode);
        shooterMotor.config_kP(0, ShooterConstants.shooterMotorKp);
        shooterMotor.config_kI(0, ShooterConstants.shooterMotorKi);
        shooterMotor.config_kD(0, ShooterConstants.shooterMotorKd);
    }

    @Override
    public void periodic() {
        if(Constants.enableSmartDashboard)
        {
            SmartDashboard.putNumber("Shooter RPM", Conversions.falconToRPM(shooterMotor.getSelectedSensorVelocity(), 1));
        }
    }

    public void setShooterRPM(long RPM)
    {
        shooterMotor.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(RPM, 1));
    }

    public boolean getRPMReached(long goalRPM)
    {
        return Math.abs(Math.abs(goalRPM) - Math.abs(Conversions.falconToRPM(shooterMotor.getSelectedSensorVelocity(), 1))) <= ShooterConstants.RPMTolerance; 
    }

    public long calculateRPM(double distanceMM)
    {
        int constant = ShooterConstants.shooterBaseRPM / ShooterConstants.shooterBaseRPM;

        return Math.round(distanceMM * constant);
    }
}
