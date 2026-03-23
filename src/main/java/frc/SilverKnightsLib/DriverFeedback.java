// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.SilverKnightsLib;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.Sendable;

public class DriverFeedback implements Sendable {

    // controllers
    private final CommandPS5Controller ps5Controller;
    private final CommandXboxController xboxController;
    private double startTimeInSeconds = Double.POSITIVE_INFINITY;

    /** Creates a new DriverFeedback. */
    public DriverFeedback(CommandPS5Controller ps5Controller2, CommandXboxController xboxController2) {
        this.ps5Controller = ps5Controller2;
        this.xboxController = xboxController2;
    }

    /** Calculates the amount of time (seconds) remaining before next shift */
    public double timeRemainingBeforeNextShift() {
        double[] shiftTimes = { 130, 105, 80, 55, 30 }; // seconds remaining

        double matchTime = Timer.getMatchTime(); // decreasing value that represents time remaining in match

        for (double shift : shiftTimes) {
            if (matchTime > shift) {
                return matchTime - shift;
            }
        }
        return 0;
    }

    // public void rumbleController() {
    // double t = timeRemainingBeforeNextShift();
    // if (t > 0 && t < 2) {
    // ps5Controller.setRumble(RumbleType.kBothRumble, 1.0);
    // xboxController.setRumble(RumbleType.kBothRumble, 1.0);
    // } else {
    // ps5Controller.setRumble(RumbleType.kBothRumble, 0.0);
    // xboxController.setRumble(RumbleType.kBothRumble, 0.0);
    // }
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Driver Feedback");
        builder.addDoubleProperty("time remaining before next shift", () -> timeRemainingBeforeNextShift(), null);
    }
}