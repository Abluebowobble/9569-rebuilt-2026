// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Utilities;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.util.sendable.Sendable;

public class DriverFeedback {

  // controllers
  private final CommandPS5Controller ps5Controller;
  private final CommandXboxController xboxController;

  /** Creates a new DriverFeedback. */
  public DriverFeedback(CommandPS5Controller ps5Controller, CommandXboxController xboxController) {
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
    double time = timeRemainingBeforeNextShift();
    SmartDashboard.putNumber("time remaining before next shift", time);
    rumbleController(time);
  }

  /** Vibrates controller every 0.2 seconds */
  public void rumbleController(double time) {
    if (time > 3 && time < 5) {
      // vibration pulses every 0.2 seconds
      double currentTime = Timer.getFPGATimestamp();
      double phase = currentTime % 0.4;
      double intensity = phase < 0.2 ? 1.0 : 0.0; // might tune intensity if controller vibrates too much

      ps5Controller.setRumble(RumbleType.kBothRumble, intensity);
      xboxController.setRumble(RumbleType.kBothRumble, intensity);
    } else {
      ps5Controller.setRumble(RumbleType.kBothRumble, 0.0);
      xboxController.setRumble(RumbleType.kBothRumble, 0.0);
    }
  }
}
