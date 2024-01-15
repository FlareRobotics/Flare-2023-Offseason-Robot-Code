package frc.robot.commands.Claw;

import frc.robot.subsystems.CompressorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ToggleCompressor extends CommandBase {

  public ToggleCompressor(CompressorSubsystem subsystem) {
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    // System.out.println("Toggle Compressor Start!");
  }

  @Override
  public void execute() {
    CompressorSubsystem.Compressor.enableDigital();
  }

  @Override
  public void end(boolean interrupted) {
    CompressorSubsystem.Compressor.disable();
    // System.out.println("Toggle Compressor End!");
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
