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

import javax.print.attribute.standard.Sides;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
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
  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(101);

  // create separate sections
  private final AddressableLEDBufferView rightView = buffer.createView(0, 50);
  // private final AddressableLEDBufferView middleView = buffer.createView(26,
  // 75);
  private final AddressableLEDBufferView leftView = buffer.createView(51, 100);

  // patterns
  private LEDPattern shooterPattern;
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
    // setRainbowScrolling();
    setOff();
  }

  @Override
  public void periodic() {
    // shooterPattern.applyTo(rightView);
    sidePattern.applyTo(leftView);
    sidePattern.applyTo(rightView);

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

  public Command flashBangCommand() {
    return runOnce(() -> set(LEDPattern.solid(Color.kWhite).blink(Seconds.of(0.1)), Section.ALL));
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

    shooterPattern = m_scrollingRainbow;
    sidePattern = m_scrollingRainbow;
  }

  public void setSolidColor(Color color, Section sec) {
    LEDPattern pattern = LEDPattern.solid(color);
    pattern.atBrightness(Percent.of(50));
    set(pattern, sec);
  }

  public void setBreathe(Color color, double seconds, Section sec) {
    LEDPattern pattern = LEDPattern.solid(color).breathe(Second.of(seconds));
    pattern.atBrightness(Percent.of(50));
    set(pattern, sec);
  }

  public void setProgressMask(DoubleSupplier progress, Section sec) {
    LEDPattern base;
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Blue) {
        base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kAliceBlue, Color.kAzure);
      } else {
        base = LEDPattern.gradient(LEDPattern.GradientType.kContinuous, Color.kAliceBlue, Color.kAzure);
      }
    } else {
      base = LEDPattern.solid(Color.kWhite);
    }

    LEDPattern pattern = LEDPattern.progressMaskLayer(progress);
    pattern = base.mask(pattern);
    pattern.atBrightness(Percent.of(50));

    set(pattern, sec);
  }

  private void set(LEDPattern pattern, Section sec) {
    switch (sec) {
      case SHOOTER:
        shooterPattern = pattern;
        break;
      case SIDE:
        sidePattern = pattern;
        break;
      case ALL:
        shooterPattern = pattern;
        sidePattern = pattern;
        break;
      default:
        setOff();
    }
  }

  public void setOff() {
    sidePattern = LEDPattern.solid(Color.kBlack);
    shooterPattern = LEDPattern.solid(Color.kBlack);
  }

}
