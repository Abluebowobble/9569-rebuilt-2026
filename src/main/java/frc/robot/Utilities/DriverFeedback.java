// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.util.sendable.Sendable;

public class DriverFeedback {

  // controllers
  private final PS5Controller ps5Controller;
  private final XboxController xboxController;

  /** Creates a new DriverFeedback. */
  public DriverFeedback(PS5Controller ps5Controller, XboxController xboxController) {
    this.ps5Controller = ps5Controller;
    this.xboxController = xboxController;
  }

  /** Calculates the amount of time (seconds) remaining before next shift */
  public double timeRemainingBeforeNextShift() {
    double[] shiftTimes = {130, 105, 80, 55, 30}; // seconds remaining

    double matchTime = Timer.getMatchTime(); // decreasing value that represents time remaining in match

    if (matchTime < 0) {
      return 0;
    }

    for (double shift : shiftTimes) {
      if (matchTime > shift) {
        return matchTime - shift;
      }
    }
    return 0;
  }

  /** Updates the amount of time (seconds) remaining before next shift */
  public void update() {
    SmartDashboard.putNumber("time remaining before next shift", timeRemainingBeforeNextShift());
    // rumbleController();
  }

  // public void rumbleController() {
  //   double t = timeRemainingBeforeNextShift();
  //   if (t > 0 && t < 2) {
  //     ps5Controller.setRumble(RumbleType.kBothRumble, 1.0);
  //     xboxController.setRumble(RumbleType.kBothRumble, 1.0);
  //   } else {
  //     ps5Controller.setRumble(RumbleType.kBothRumble, 0.0);
  //     xboxController.setRumble(RumbleType.kBothRumble, 0.0);
  //   }
  // }
}
