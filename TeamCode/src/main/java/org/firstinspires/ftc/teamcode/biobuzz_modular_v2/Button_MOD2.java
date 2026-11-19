package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

/**
 * Makes a button act like a "click" instead of "held down".
 *
 *   private final Button aButton = new Button();
 *   ...
 *   if (aButton.pressed(gamepad1.a)) { ... }   // runs ONCE per press, not every loop
 *
 * Use one Button object per physical button.
 */
public class Button_MOD2 {
    private boolean wasDown = false;

    public boolean pressed(boolean isDownNow) {
        boolean justPressed = isDownNow && !wasDown;
        wasDown = isDownNow;
        return justPressed;
    }
}
