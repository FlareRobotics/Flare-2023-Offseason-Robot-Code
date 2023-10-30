package frc.robot.commands.Auto;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Shifter extends CommandBase {

  int reduction;
  DriveSubsystem subsystem;
  public Shifter(DriveSubsystem driveSubsystem, int reduction) {
    subsystem = driveSubsystem;
    this.reduction = reduction;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    DriveSubsystem.shifterReduction = reduction;
  }

  @Override
  public void execute() {
   
  }

  @Override
  public void end(boolean interrupted) {
    DriveSubsystem.shifterReduction = 1;
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}