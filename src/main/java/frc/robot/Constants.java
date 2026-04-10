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
import edu.wpi.first.math.geometry.Translation2d;
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
    public static final double OVERRIDE_DEADBAND = 0.1;
    public static final double DEADBAND = 0.05;
    public static final double TURN_FACTOR = 0.5;
    public static final Time HOLD_DELAY = Seconds.of(0.3);
  }

  public static class SwerveConstants {
    public static final LinearVelocity MAX_SPEED = MetersPerSecond.of(Units.feetToMeters(16.6));
    public static final AngularVelocity MAX_SWERVE_ANGULAR_VELOCITY = Degrees.of(10).per(Second);

    public static final double driveSlewRateLimit = 25;
    public static final double driveJerkRateLimit = 55;
    public static final double autonSlewRateLimit = 15;
    public static final double autonJerkRateLimit = 30;

    public static final PIDController translationController = new PIDController(4, 0, 0, 0.15);
    public static final PIDController rotationController = new PIDController(7, 0, 0.15); // use 7, 0, 0.15, try p =
                                                                                          // 0.14 if that doesnt work
  }

  public static class WaypointConstants {
    public static final Translation2d BLUE_1_START = new Translation2d(3.541, 5.180);
    public static final Translation2d BLUE_1_BACKUP = new Translation2d(3.2, 5.180); // do 2.5 for practice to ensure we
                                                                                   // get through everything? idk
    public static final Translation2d BLUE_1_RUNUP = new Translation2d(6, 5.180);
    public static final Translation2d BLUE_1_BEGIN_INTAKE = new Translation2d(8.375, 6.85);
    public static final Translation2d BLUE_1_FINISH_INTAKE = new Translation2d(8.375, 0); // 4.14
    public static final Translation2d BLUE_1_FINISH_INTAKE2 = new Translation2d(8.375, 4.14); // 4.14
    public static final Translation2d BLUE_1_PREPARE_BUMP = new Translation2d(6.5, 5.391);
    public static final Translation2d BLUE_1_RETURN = new Translation2d(3, 5.391);
    public static final Translation2d BLUE_1_SHOOT = new Translation2d(3.303, 3.97);
    public static final Translation2d BLUE_1_PREPARE_DEPOT_INTAKE = new Translation2d(1.52, 5.95);
    public static final Translation2d BLUE_1_DEPOT_INTAKE = new Translation2d(0.706, 5.95);
    public static final Translation2d TEMP = new Translation2d(10.6, 6.363);
    public static final Translation2d SHOOT_NO_INTAKE = new Translation2d(3.541, 4.043);
  }

  public static class BehaviourConstants {
    public static final Time DELAY_BEFORE_AGITATE = Seconds.of(0.5);
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
    public static final int SERVO_HUB = 18;
    public static final int LED = 9;
  }
}