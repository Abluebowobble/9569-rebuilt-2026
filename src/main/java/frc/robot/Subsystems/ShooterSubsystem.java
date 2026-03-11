// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HardwareMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
  // motor
  private final SparkMax leftShooterMotor = new SparkMax(HardwareMap.SHOOTER_LEFT, MotorType.kBrushless);
  private final SparkMax middleShooterMotor = new SparkMax(HardwareMap.SHOOTER_MIDDLE, MotorType.kBrushless);
  private final SparkMax rightShooterMotor = new SparkMax(HardwareMap.SHOOTER_RIGHT, MotorType.kBrushless);
  private final SparkMax[] motors = { leftShooterMotor, middleShooterMotor, rightShooterMotor };

  // encoder
  private final RelativeEncoder leftEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder middleEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder rightEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder[] encoders = { leftEncoder, middleEncoder, rightEncoder };

  // pidf
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0.19, 0.58); // to tune
  private final PIDController controller = new PIDController(0, 0, 0); // to tune

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

    SmartDashboard.putData(this);
  }

  public void setVoltage(Voltage voltage) {
    for (SparkMax m : motors) {
      m.setVoltage(voltage);
    }
  }

  public void updateCurrentSpeedOfMotor(RelativeEncoder encoder, SparkMax motor) {
    double pidVoltage = controller.calculate(encoder.getVelocity(), targetRPM) * motor.getBusVoltage();
    motor.setVoltage(feedForward.calculate(targetRPM) + pidVoltage);

  }

  public void updateCurrentSpeed() {
    for (int i = 0; i < 3; i++) {
      updateCurrentSpeedOfMotor(encoders[i], motors[i]);
    }
  }

  public void set(double rpm) {
    targetRPM = rpm;
  }

  public boolean isVelocityWithinTolerance() {
    for (RelativeEncoder e : encoders) {
      double currentRPM = e.getVelocity();
      if (!MathUtil.isNear(targetRPM, currentRPM, kVelocityTolerance)) {
        return false;
      }
    }

    return true;
  }

  public Command runCommand(double rpm) {
    return runOnce(() -> set(rpm))
        .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
  }

  @Override
  public void periodic() {
    updateCurrentSpeed();
    SmartDashboard.putData(this);
  }

  @Override
  public void initSendable(SendableBuilder sendableBuilder) {
    sendableBuilder.addDoubleProperty("left RPM", () -> leftEncoder.getVelocity(), null);
    sendableBuilder.addDoubleProperty("middle RPM", () -> middleEncoder.getVelocity(), null);
    sendableBuilder.addDoubleProperty("right RPM", () -> rightEncoder.getVelocity(), null);
    sendableBuilder.addDoubleProperty("target RPM", () -> targetRPM,
        value -> targetRPM = value);

  }
}