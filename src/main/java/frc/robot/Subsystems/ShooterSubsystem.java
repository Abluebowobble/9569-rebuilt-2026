// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.HardwareMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
  // motors
  private final SparkMax leftShooterMotor = new SparkMax(HardwareMap.SHOOTER_LEFT, MotorType.kBrushless);
  private final SparkMax middleShooterMotor = new SparkMax(HardwareMap.SHOOTER_MIDDLE, MotorType.kBrushless);
  private final SparkMax rightShooterMotor = new SparkMax(HardwareMap.SHOOTER_RIGHT, MotorType.kBrushless);

  // encoders
  private final RelativeEncoder leftEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder middleEncoder = middleShooterMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightShooterMotor.getEncoder();

  private final SparkClosedLoopController controller = leftShooterMotor.getClosedLoopController();

  // pidf
  private AngularVelocity targetRPM = RPM.of(0); // desired RPM we want the wheels to turn at

  private static final AngularVelocity kVelocityTolerance = RPM.of(50); // if current RPM is within desired RPM +-
                                                                        // velocity tolerance,
  // then its within tolerance
  private Voltage voltage = Volts.of(0);

  private static final AngularVelocity kStartingVelocity = RPM.of(0);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // initialize motors
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .voltageCompensation(12.0)
        .openLoopRampRate(1)
        .closedLoopRampRate(1)
        .inverted(false);
    leaderConfig.closedLoop
        .pid(0.0001, 0, 0.001, ClosedLoopSlot.kSlot0).feedForward // test
        .sv(0.115, 0.00203, ClosedLoopSlot.kSlot0); // might wanna increase kV
    leaderConfig.smartCurrentLimit(110, 80);

    leftShooterMotor.configure(
        leaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig middleFollower = new SparkMaxConfig();
    middleFollower.follow(leftShooterMotor, true); // invert follow if needed
    middleFollower.smartCurrentLimit(110, 80);
    middleShooterMotor.configure(
        middleFollower,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig rightFollower = new SparkMaxConfig();
    rightFollower.follow(leftShooterMotor, true); // invert follow if needed
    rightFollower.smartCurrentLimit(110, 80);
    rightShooterMotor.configure(
        rightFollower,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    setDefaultCommand(idle());
  }

  /** sets voltage of all motors given Voltage enum */
  public void setVoltage(Voltage voltage) {
    leftShooterMotor.setVoltage(voltage.magnitude());
    middleShooterMotor.setVoltage(voltage.magnitude());
    rightShooterMotor.setVoltage(voltage.magnitude());
  }

  /**
   * this function updates the rpm based on the value in smart dashboard, updates
   * each motor separately
   */
  public void updateSpeedWithSmartDashboard() {
    double targetL = SmartDashboard.getNumber("Left Pivot RPM", 0);
    double targetM = SmartDashboard.getNumber("Middle Pivot RPM", 0);
    double targetR = SmartDashboard.getNumber("Right Pivot RPM", 0);
    double[] targets = { targetL, targetM, targetR };

    for (int i = 0; i < 3; i++) {
      set(Volts.of(targets[i]));
    }
  }

  /** set target RPM for all motors */
  public void set(DoubleSupplier rpm) {
    set(RPM.of(rpm.getAsDouble()));
  }

  public void set(AngularVelocity rpm) {
    controller.setSetpoint(rpm.magnitude(), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    targetRPM = rpm;
  }

  public void set(Voltage volts) {
    leftShooterMotor.setVoltage(volts.magnitude());
    middleShooterMotor.setVoltage(volts.magnitude());
    rightShooterMotor.setVoltage(volts.magnitude());
  }

  /** checks if each shooter has their velocity is within tolerance */
  public boolean isVelocityWithinTolerance() {
    return targetRPM.isNear(RPM.of(leftEncoder.getVelocity()), kVelocityTolerance);
  }

  /** sets voltage to shoot in front of Hub */
  public Command runCommand(AngularVelocity rpm) {
    return runOnce(() -> set(rpm));
  }

  /** sets voltage to shoot in front of Hub */
  public Command runTestCommand(DoubleSupplier rpm) {
    return runOnce(() -> set(rpm));
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> set(voltage));
  }

  @Override
  public Command idle() {
    return runOnce(() -> set(RPM.of(kStartingVelocity.magnitude())));
  }

  @Override
  public void periodic() {
    // uncomment below to tune
    // updateSpeedWithSmartDashboard();

    // telemetry
    SmartDashboard.putData(this);

  }

  public double progress() {
    if (isVelocityWithinTolerance()) {
      return 1.0;
    }

    AngularVelocity avgVelocity = RPM
        .of(Math.abs((leftEncoder.getVelocity() + middleEncoder.getVelocity() + rightEncoder.getVelocity()) / 3.0));

    double targetRpmMagnitude = targetRPM.magnitude();
    // dont divide by zero :)
    if (targetRpmMagnitude < 1e-9) {
      return 1.0;
    }

    double error = Math.abs(avgVelocity.magnitude() - targetRpmMagnitude);
    return MathUtil.clamp(1.0 - (error / targetRpmMagnitude), 0.0, 1.0);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Shooter");
    builder.addDoubleProperty("left RPM", () -> leftEncoder.getVelocity(), null);
    builder.addDoubleProperty("middle RPM", () -> middleEncoder.getVelocity(), null);
    builder.addDoubleProperty("right RPM", () -> rightEncoder.getVelocity(), null);
    // addd current property later
    builder.addBooleanProperty("is Velocity within tolerance", this::isVelocityWithinTolerance, null);
    builder.addDoubleProperty("voltage (each)", () -> voltage.magnitude(), null);
    builder.addDoubleProperty("Target rpm", () -> targetRPM.magnitude(), null);
  }
}