// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/** debugging the shooter steps:
 * 1. remove feedforward
 * 2. 
 */

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

  // pidf
  private final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0, 0); // 0.15, 0.19, 0.58)
  private final PIDController controller = new PIDController(0.09, 0, 0); // to tune
    private double targetRPM = 0; // desired RPM we want the wheels to turn at


  private static final double kVelocityTolerance = 1; // if current RPM is within desired RPM +- velocity tolerance,
                                                      // then its within tolerance
  private static final double kRPMShooter = 1;


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    // initialize motors
    SparkBaseConfig config = new SparkMaxConfig();
    leftShooterMotor.configure(config.inverted(false), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    middleShooterMotor.configure(config.inverted(true), ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    rightShooterMotor.configure(config.inverted(true), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // for live debugging
    SmartDashboard.putNumber("Left Pivot RPM", 0);
    SmartDashboard.putNumber("Middle Pivot RPM", 0);
    SmartDashboard.putNumber("Right Pivot RPM", 0);
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
  public void updateCurrentSpeedOfMotor(RelativeEncoder encoder, SparkMax motor) {
    double pidVoltage = controller.calculate(encoder.getVelocity(), targetRPM);
    motor.setVoltage(feedForward.calculate(targetRPM) + pidVoltage);
  }

  /**
   * This function updates the rpm based on the value in smart dashboard. Updates
   * each motor separately.
   */
  public void updateSpeedWithSmartDashboard() {
    double targetL = SmartDashboard.getNumber("Left Pivot RPM", 0);
    double targetM = SmartDashboard.getNumber("Middle Pivot RPM", 0);
    double targetR = SmartDashboard.getNumber("Right Pivot RPM", 0);
    double[] targets = { targetL, targetM, targetR };

    for (int i = 0; i < 3; i++) {
      double pidVoltage = controller.calculate(encoders[i].getVelocity(), targets[i]) * motors[i].getBusVoltage();
      motors[i].setVoltage(feedForward.calculate(targets[i]) + pidVoltage);
    }
  }

  /** update speed for all three motors */
  public void updateCurrentSpeed() {
    for (int i = 0; i < 3; i++) {
      updateCurrentSpeedOfMotor(encoders[i], motors[i]);
    }
  }

  /** set target RPM for all motors */
  public void set(double rpm) {
    targetRPM = rpm;
  }

  /** sets rpm to 0 */
  public void stop() {
    set(0);
  }

  /** checks if each shooter has their velocity is within tolerance */
  public boolean isVelocityWithinTolerance() {
    for (RelativeEncoder e : encoders) {
      double currentRPM = e.getVelocity();
      if (!MathUtil.isNear(targetRPM, currentRPM, kVelocityTolerance)) {
        return false;
      }
    }

    return true;
  }

  /** sets rpm to double value given, ends when velocity is within tolerance */
  public Command runCommand(double rpm) {
    return runOnce(() -> set(rpm))
        .andThen(Commands.waitUntil(this::isVelocityWithinTolerance));
  }

  public Command runCommand() {
    return runCommand(kRPMShooter)
      .handleInterrupt(() -> set(0));
  }

  @Override
  public void periodic() {
    // pid
    updateCurrentSpeed();

    // uncomment below to tune
    // updateSpeedWithSmartDashboard();

    // telemetry
    SmartDashboard.putNumber("left RPM", leftEncoder.getVelocity());
    SmartDashboard.putNumber("middle RPM", middleEncoder.getVelocity());
    SmartDashboard.putNumber("right RPM", rightEncoder.getVelocity());
  }
}