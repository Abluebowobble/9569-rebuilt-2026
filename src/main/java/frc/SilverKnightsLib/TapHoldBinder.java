package frc.SilverKnightsLib;

import java.util.concurrent.atomic.AtomicBoolean;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public final class TapHoldBinder {
    private TapHoldBinder() {
        // utility class
    }

    /**
     * Binds a button so that:
     * - quick tap (released before holdTimeSeconds) runs onTap
     * - hold (pressed continuously for at least holdTimeSeconds) runs onHold
     * - onHold ends automatically when the button is released
     */
    public static void bind(
            Trigger button,
            Command onTap,
            Command onHold,
            Time holdTimeSeconds) {

        AtomicBoolean holdTriggered = new AtomicBoolean(false);
        Trigger heldLongEnough = button.debounce(holdTimeSeconds.magnitude());

        // Reset every fresh press
        button.onTrue(
            Commands.runOnce(() -> holdTriggered.set(false))
        );

        // Once the hold threshold is reached, mark the hold as triggered
        heldLongEnough.onTrue(
            Commands.runOnce(() -> holdTriggered.set(true))
        );

        // Run hold command only while held after the debounce time
        heldLongEnough.whileTrue(onHold);

        // On release:
        // - if hold never triggered -> run tap
        // - if hold triggered -> do nothing
        button.onFalse(
            Commands.either(
                Commands.none(),
                onTap,
                holdTriggered::get
            )
        );
    }
}