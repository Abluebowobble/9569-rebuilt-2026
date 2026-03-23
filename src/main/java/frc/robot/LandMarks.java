package frc.robot;

import java.util.Optional;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class LandMarks {
    public static final AprilTagFieldLayout K_APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public static final double kFieldLength = K_APRIL_TAG_FIELD_LAYOUT.getFieldLength();
    public static final double kFieldWidth = K_APRIL_TAG_FIELD_LAYOUT.getFieldWidth();

    public static final double kAllianceFieldLength = 182.105;
    public static final Optional<Alliance> kAlliance = DriverStation.getAlliance();

    public static Translation2d hubPosition() {
        if (kAlliance.isPresent() && kAlliance.get() == Alliance.Blue) {
            return new Translation2d(Inches.of(182.105), Inches.of(158.845));
        }

        return new Translation2d(Inches.of(469.115), Inches.of(158.845));
    } 
}
