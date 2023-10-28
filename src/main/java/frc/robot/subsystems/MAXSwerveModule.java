// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// WPILIB Imports
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

// Rev Imports
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;

// CTRE Imports
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

//File Imports
import frc.lib.math.Conversions;
import frc.robot.SwerveConstants.ModuleConstants;

public class MAXSwerveModule {

  private final TalonFX m_drivingTalon;
  private final CANSparkMax m_turningSparkMax;

  private final AbsoluteEncoder m_turningEncoder;

  private final SparkMaxPIDController m_turningPIDController;

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ModuleConstants.kS, ModuleConstants.kV,
      ModuleConstants.kA);

  private double m_chassisAngularOffset = 0;

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, boolean drivingReversed,
      boolean turningReversed) {
    m_drivingTalon = new TalonFX(drivingCANId);
    m_turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);

    // Factory reset, so we get the SPARKS MAX to a known state before configuring
    // them. This is useful in case a SPARK MAX is swapped out.
    // Same for TalonFX
    m_drivingTalon.configFactoryDefault();
    // m_turningSparkMax.restoreFactoryDefaults();

    // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
    m_turningEncoder = m_turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    m_turningPIDController = m_turningSparkMax.getPIDController();
    m_turningPIDController.setFeedbackDevice(m_turningEncoder);

    /* Configure TalonFX's Sensor Source for Pirmary PID */
    m_drivingTalon.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 30);
    m_drivingTalon.setInverted(drivingReversed);

    // Apply position and velocity conversion factors for the driving encoder. The
    // native units for position and velocity are rotations and RPM, respectively,
    // but we want meters and meters per second to use with WPILib's swerve APIs.

    // Apply position and velocity conversion factors for the turning encoder. We
    // want these in radians and radians per second to use with WPILib's swerve
    // APIs.
    m_turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor);
    m_turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    m_turningEncoder.setInverted(turningReversed);

    // Enable PID wrap around for the turning motor. This will allow the PID
    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
    // to 10 degrees will go through 0 rather than the other direction which is a
    // longer route.
    m_turningPIDController.setPositionPIDWrappingEnabled(true);
    m_turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput);
    m_turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);

    // Set the PID gains for the driving motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!(P,I,D,FF)

    // Set the PID gains for the turning motor. Note these are example gains, and
    // you
    // may need to tune them for your own robot!
    m_turningPIDController.setP(ModuleConstants.kTurningP);
    m_turningPIDController.setI(ModuleConstants.kTurningI);
    m_turningPIDController.setD(ModuleConstants.kTurningD);
    m_turningPIDController.setFF(ModuleConstants.kTurningFF);
    m_turningPIDController.setOutputRange(ModuleConstants.kTurningMinOutput,
        ModuleConstants.kTurningMaxOutput);

    m_drivingTalon.setNeutralMode(ModuleConstants.kDrivingMotorNeutralMode);
    m_turningSparkMax.setIdleMode(ModuleConstants.kTurningMotorIdleMode);
    // No current limit is set for driving motor
    m_turningSparkMax.setSmartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_turningSparkMax.burnFlash();

    m_chassisAngularOffset = chassisAngularOffset;
    m_drivingTalon.setSelectedSensorPosition(0);

    m_turningPIDController.setReference( m_chassisAngularOffset,
        CANSparkMax.ControlType.kPosition);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        Conversions.falconToMPS(m_drivingTalon.getSelectedSensorVelocity(), ModuleConstants.kWheelCircumferenceMeters,
            ModuleConstants.kDrivingMotorReduction),
        new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        getDriveMotorPosition(),
        new Rotation2d(m_turningEncoder.getPosition() + m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

    if (Math.abs(desiredState.speedMetersPerSecond) < 0.01) {
      stop();
      return;
    }

    SwerveModuleState optiSwerveModuleState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRadians(m_turningEncoder.getPosition() > 6.23 ? 0 :  m_turningEncoder.getPosition()));
    // Command driving and turning SPARKS MAX towards their respective setpoints.

    // Apply chassis angular offset to the desired state.
    // Set speed of Falcon
    setSpeed(optiSwerveModuleState, isOpenLoop);

    m_turningPIDController.setReference(optiSwerveModuleState.angle.getRadians() + m_chassisAngularOffset,
        CANSparkMax.ControlType.kPosition);
  }

  public void stop() {
    m_drivingTalon.set(ControlMode.PercentOutput, 0);
    m_turningSparkMax.set(0);
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingTalon.setSelectedSensorPosition(0);
  }

  /**
   * Calculates distance by drive motors encoder.
   *
   * @return Motor position in meters.
   */
  public double getDriveMotorPosition() {
    return Conversions.falconToMeters(m_drivingTalon.getSelectedSensorPosition(),
        ModuleConstants.kWheelCircumferenceMeters, ModuleConstants.kDrivingMotorReduction);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.kmaxSpeed;
      m_drivingTalon.set(ControlMode.PercentOutput, percentOutput);
    } else {
      double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond,
          ModuleConstants.kWheelCircumferenceMeters, ModuleConstants.kDrivingMotorReduction);
      m_drivingTalon.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }
}
