// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareMap;

public class LEDSubsystem extends SubsystemBase {
  // initialize led
  private final AddressableLED m_led = new AddressableLED(HardwareMap.LED);
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(51);

  // create separate sections
  // private final AddressableLEDBufferView sideView = buffer.createView(0, 70);
  // private final AddressableLEDBufferView middleView = buffer.createView(26,
  // 75);

  // patterns
  // private LEDPattern shooterPattern;
  private LEDPattern sidePattern;

  // alliance
  Optional<DriverStation.Alliance> alliance = Optional.empty();

  public enum Section {
    SHOOTER, SIDE, ALL
  };

  public LEDSubsystem() {
    m_led.setLength(buffer.getLength());

    m_led.setData(buffer);
    m_led.start();

    alliance = DriverStation.getAlliance();

    // runDefaultColor();
    setRainbowScrolling();
    // setOff();
  }

  @Override
  public void periodic() {
    // shooterPattern.applyTo(rightView);
    sidePattern.applyTo(buffer);
    // sidePattern.applyTo(rightView);

    m_led.setData(buffer);
  }

  public void runDefaultColor() {
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        setSolidColor(Color.kRed, Section.ALL);
      } else {
        setSolidColor(Color.kBlue, Section.ALL);
      }
    } else {
      setRainbowScrolling();
    }
  }

  public void setContinousGradientScrolling(Color color1, Color color2, double time, Section sec, boolean isReversed) {
    LEDPattern pattern = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, color1, color2).scrollAtAbsoluteSpeed(
        MetersPerSecond.of(time), Meters.of(1 / 120.0));

    if (isReversed) {
      pattern.reversed();
    }

    set(pattern, sec);
  }

  public Command flashbangCommand() {
    return run(() -> set(LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.05)), Section.ALL));
  }

  public void setRainbowScrolling() {
    // all hues at maximum saturation and half brightness
    LEDPattern m_rainbow = LEDPattern.rainbow(255, 28);

    // Our LED strip has a density of 50 LEDs per 21 inches
    Distance ledLength = Meters.of(Units.inchesToMeters(21));

    // Create a new pattern that scrolls the rainbow pattern across the LED strip,
    // moving at a speed of 1 meter per second.
    LEDPattern m_scrollingRainbow = m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.25),
        ledLength.div(50));

    // shooterPattern = m_scrollingRainbow;
    sidePattern = m_scrollingRainbow;
  }

  public void setSolidColor(Color color, Section sec) {
    LEDPattern pattern = LEDPattern.solid(color);
    set(pattern, sec);
  }

  public void setBreathe(Color color, double seconds, Section sec) {
    LEDPattern pattern = LEDPattern.solid(color).breathe(Second.of(seconds));
    set(pattern, sec);
  }

  public void setBlink(Color color, Time seconds, Section sec) {
    LEDPattern pattern = LEDPattern.solid(color).blink(seconds);
    set(pattern, sec);
  }

  public void setProgressMask(DoubleSupplier progress, Color color, Section sec) {
    LEDPattern base = LEDPattern.solid(color);
    // if (alliance.isPresent()) {
    // if (alliance.get() == DriverStation.Alliance.Blue) {
    // base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous,
    // Color.kAliceBlue, Color.kAzure);
    // } else {
    // base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kRed,
    // Color.kDarkRed);
    // }
    // } else {
    // base = LEDPattern.solid(Color.kWhite);
    // }

    LEDPattern pattern = LEDPattern.progressMaskLayer(progress);
    pattern = base.mask(pattern);

    set(pattern, sec);
  }

  private void set(LEDPattern pattern, Section sec) {
    sidePattern = pattern;
    // switch (sec) {
    // case SHOOTER:
    // shooterPattern = pattern;
    // break;
    // case SIDE:
    // sidePattern = pattern;
    // break;
    // case ALL:
    // shooterPattern = pattern;
    // sidePattern = pattern;
    // break;
    // default:
    // setOff();
    // }
  }

  public void setOff() {
    sidePattern = LEDPattern.solid(Color.kBlack);
    // shooterPattern = LEDPattern.solid(Color.kBlack);
  }

}
