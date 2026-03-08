// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.HardwareMap;

public class ShooterSubsystem extends SubsystemBase {
  // motor
  private final SparkMax leftShooterMotor = new SparkMax(HardwareMap.SHOOTER_LEFT, MotorType.kBrushless);
  private final SparkMax middleShooterMotor = new SparkMax(HardwareMap.SHOOTER_MIDDLE, MotorType.kBrushless);
  private final SparkMax rightShooterMotor = new SparkMax(HardwareMap.SHOOTER_RIGHT, MotorType.kBrushless);

  // encoder
  private final RelativeEncoder leftEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder middleEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder rightEncoder = leftShooterMotor.getEncoder();

  // pidf
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0, 0); // to tune
  private final PIDController motorController = new PIDController(0, 0, 0); // to tune

  private static final double kVelocityTolerance = 0.1; // if current RPM is within desired RPM +- velocity tolerance,
                                                       // then its within tolerance
  private double targetRPM = 0; // desired RPM we want the wheels to turn at, to tune (?)

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    SparkBaseConfig config = new SparkMaxConfig();
    leftShooterMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    middleShooterMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightShooterMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setVoltage(Voltage voltage) {
    leftShooterMotor.setVoltage(voltage);
    middleShooterMotor.setVoltage(voltage);
    rightShooterMotor.setVoltage(voltage);
  }

  public void updateCurrentSpeed(double rpm, RelativeEncoder encoder, SparkMax motor) {
    double spins = motorController.calculate(encoder.getVelocity(), rpm);
    motor.setVoltage(spins * motor.getBusVoltage());

  }

  public boolean isVelocityWithinTolerance(RelativeEncoder encoder) {
    double currentRPM = encoder.getVelocity();
    return MathUtil.isNear(targetRPM, currentRPM, velocityTolerance); // checks if the current RPM is within desired RPM
  }

  public Command runCommand() {
    return startEnd(() -> setVoltage(Speed.RUN), () -> setVoltage(Speed.STOP));
  }

  @Override
  public void periodic() {

  }

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addDoubleProperty("RPM (Rotations per minute)", () -> leftEncoder.getVelocity(), null);
  }
}