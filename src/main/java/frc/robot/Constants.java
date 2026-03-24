// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.FeetPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_1_CONTROLLER_PORT = 0;
    public static final int DRIVER_2_CONTROLLER_PORT = 1;
    public static final double OVERRIDE_DEADBAND = 0.3;
    public static final double TURN_FACTOR = 0.5;
    public static final Time HOLD_DELAY = Seconds.of(0.3);

    public static final double driveSlewRateLimit = 25;
    public static final double driveJerkRateLimit = 55;
    public static final double autonSlewRateLimit = 15;
    public static final double autonJerkRateLimit = 30;

    public static final PIDController translationController = new PIDController(4, 0, 0, 0.15);
    public static final PIDController rotationController = new PIDController(8, 0, 0.15);
  }

  public static class SwerveConstants {
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(Units.feetToMeters(16.6));
    public static final AngularVelocity MAX_SWERVE_ANGULAR_VELOCITY = Degrees.of(10).per(Second);
    public static final Angle AIM_TOLERANCE = Degrees.of(5);
  }

  public static class BehaviourConstants {
    public static final Time DELAY_BEFORE_AGITATE = Seconds.of(2);
    public static final AngularVelocity TEMP_SHOOTER_VELOCITY = RPM.of(5300);
  }

  public static class HardwareMap {

    // NEO
    public static final int INTAKE_ROLLER = 16;
    public static final int INTAKE_PIVOT = 17;
    public static final int CONVEYOR = 14;
    public static final int FEEDER = 15;
    public static final int SHOOTER_LEFT = 11;
    public static final int SHOOTER_MIDDLE = 12;
    public static final int SHOOTER_RIGHT = 13;

    // PWM
    public static final int ACTUATOR_LEFT = 0;
    public static final int ACTUATOR_RIGHT = 1;
    public static final int LED = 9;
  }
}