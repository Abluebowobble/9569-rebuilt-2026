// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
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
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {
  // motors
  private final SparkMax leftShooterMotor = new SparkMax(HardwareMap.SHOOTER_LEFT, MotorType.kBrushless);
  private final SparkMax middleShooterMotor = new SparkMax(HardwareMap.SHOOTER_MIDDLE, MotorType.kBrushless);
  private final SparkMax rightShooterMotor = new SparkMax(HardwareMap.SHOOTER_RIGHT, MotorType.kBrushless);
  private final SparkMax[] motors = { leftShooterMotor, middleShooterMotor, rightShooterMotor };

  // encoders
  private final RelativeEncoder leftEncoder = leftShooterMotor.getEncoder();
  private final RelativeEncoder middleEncoder = middleShooterMotor.getEncoder();
  private final RelativeEncoder rightEncoder = rightShooterMotor.getEncoder();
  private final RelativeEncoder[] encoders = { leftEncoder, middleEncoder, rightEncoder };

  private final SparkClosedLoopController controller = leftShooterMotor.getClosedLoopController();

  // pidf
  private double targetRPM = 0; // desired RPM we want the wheels to turn at

  private static final double kVelocityTolerance = 5; // if current RPM is within desired RPM +- velocity tolerance,
                                                      // then its within tolerance
  private static final double kTargetVelocity = 5000;
  private Voltage voltage = Volts.of(0);

  // speed for roller motor
  public enum Speed {
    STOP(Volts.of(0)),
    INFRONTOFHUB(Volts.of(11)); // to tune

    private final Voltage voltage;

    private Speed(Voltage voltage) {
      this.voltage = voltage;
    }

    public Voltage voltage() {
      return voltage;
    }
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // initialize motors
    SparkMaxConfig leaderConfig = new SparkMaxConfig();
    leaderConfig
        .inverted(false)
        .voltageCompensation(12.0);

    leaderConfig.closedLoop
        .pid(0.0002, 0.0, 0.0, ClosedLoopSlot.kSlot0).feedForward
        .sva(0.15, 12.0 / 6000.0, 0.03, ClosedLoopSlot.kSlot0);

    leftShooterMotor.configure(
        leaderConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig middleFollower = new SparkMaxConfig();
    middleFollower.follow(leftShooterMotor, true); // invert follow if needed
    middleShooterMotor.configure(
        middleFollower,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    SparkMaxConfig rightFollower = new SparkMaxConfig();
    rightFollower.follow(leftShooterMotor, true); // invert follow if needed
    rightShooterMotor.configure(
        rightFollower,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  /** sets voltage of all motors given Voltage enum */
  public void setVoltage(Voltage voltage) {
    leftShooterMotor.setVoltage(voltage.magnitude());
    middleShooterMotor.setVoltage(voltage.magnitude());
    rightShooterMotor.setVoltage(voltage.magnitude());
  }

  /**
   * updates speed of given motor via pidf, requires RelativeEncoder encoder,
   * SparkMax motor
   */
  // public void updateCurrentSpeedOfMotor(RelativeEncoder encoder, SparkMax
  // motor) {
  // double pidVoltage = controller.calculate(encoder.getVelocity(), targetRPM);
  // double ff = feedForward.calculate(targetRPM);
  // motor.setVoltage(ff + pidVoltage);
  // }

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

  /** update speed for all three motors */
  // public void updateCurrentSpeed() {
  // updateCurrentSpeedOfMotor(encoders[0], motors[0]);
  // for (int i = 0; i < 3; i++) {
  // updateCurrentSpeedOfMotor(encoders[i], motors[i]);
  // }
  // }

  /** set target RPM for all motors */
  public void set(double rpm) {
    controller.setSetpoint(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
  }

  public void set(Speed volts) {
    voltage = volts.voltage();
    leftShooterMotor.setVoltage(volts.voltage());
    middleShooterMotor.setVoltage(volts.voltage());
    rightShooterMotor.setVoltage(volts.voltage());
  }

  public void set(Voltage volts) {
    leftShooterMotor.setVoltage(volts.magnitude());
    middleShooterMotor.setVoltage(volts.magnitude());
    rightShooterMotor.setVoltage(volts.magnitude());
  }

  public void set(DoubleSupplier volts) {
    voltage = Volts.of(volts.getAsDouble());
    leftShooterMotor.setVoltage(volts.getAsDouble());
    middleShooterMotor.setVoltage(volts.getAsDouble());
    rightShooterMotor.setVoltage(volts.getAsDouble());
  }

  /** sets rpm to 0 */
  public void stop() {
    set(400);
  }

  /** checks if each shooter has their velocity is within tolerance */
  public boolean isVelocityWithinTolerance() {
    return MathUtil.isNear(kTargetVelocity, leftEncoder.getVelocity(), kVelocityTolerance);
  }

  /** sets voltage to shoot in front of Hub */
  public Command runCommand() {
    return runOnce(() -> set(2000))
        .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
    // return startEnd(() -> set(Speed.INFRONTOFHUB), () -> set(Speed.STOP));
  }

  public Command stopCommand() {
    return runOnce(() -> stop())
        .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
  }

  public Command runVoltageCommand(DoubleSupplier voltage) {
    return run(() -> set(voltage));
  }

  @Override
  public void periodic() {
    // pid
    // updateCurrentSpeed();

    // uncomment below to tune
    // updateSpeedWithSmartDashboard();

    // leftShooterLog.append(leftEncoder.getVelocity());

    // telemetry
    SmartDashboard.putNumber("left RPM", leftEncoder.getVelocity());
    SmartDashboard.putNumber("middle RPM", middleEncoder.getVelocity());
    SmartDashboard.putNumber("right RPM", rightEncoder.getVelocity());
    SmartDashboard.putBoolean("is Velocity within tolerance",
        isVelocityWithinTolerance());
    SmartDashboard.putNumber("shooter voltage (each)", voltage.magnitude());
    SmartDashboard.putNumber("Target rpm", targetRPM);
  }
}