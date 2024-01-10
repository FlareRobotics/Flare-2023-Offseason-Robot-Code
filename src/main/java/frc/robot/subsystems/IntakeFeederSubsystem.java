package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeFeederConstants;

public class IntakeFeederSubsystem extends SubsystemBase {
    private final WPI_VictorSPX intakeFeederMotor = new WPI_VictorSPX(IntakeFeederConstants.intakeFeederCanID);
    private final DigitalInput intakeFeederSensor = new DigitalInput(IntakeFeederConstants.intakeFeederSensorID);
    private final DigitalInput intakeFeederEndSensor = new DigitalInput(IntakeFeederConstants.intakeFeederSensorEndID);

    public IntakeFeederSubsystem()
    {
        intakeFeederMotor.setInverted(IntakeFeederConstants.intakeFeederReversed);
        intakeFeederMotor.setNeutralMode(IntakeFeederConstants.intakeFeederNeutralMode);
    }

    @Override
    public void periodic() {
        if(Constants.enableSmartDashboard)
        {
            SmartDashboard.putBoolean("Feeder Sensor", intakeFeederSensor.get());
        }
    }

    public void setIntakeFeederSpeed(double speed)
    {
        intakeFeederMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public boolean getIntakeFeederSensor()
    {
        return intakeFeederSensor.get();
    }

    public boolean getIntakeFeedeEndSensor()
    {
        return intakeFeederEndSensor.get();
    }
}
