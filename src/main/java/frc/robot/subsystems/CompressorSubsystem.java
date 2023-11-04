package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;

public class CompressorSubsystem extends SubsystemBase {

  public static Compressor Compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  public CompressorSubsystem() {

  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Compresor On", Compressor.isEnabled());
    SmartDashboard.putBoolean("Compressor Switch", Compressor.getPressureSwitchValue());
  }

  public void setDash() {
  }

}
