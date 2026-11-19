package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

/** Alliance / hive selection and "turn to face the hive" (uses Pedro's hold, like your original). */
public class HiveTargeting_MOD2 {

    // ---- hive coordinates (red values are still placeholders, as in your original) ----
    public static double BLUEHIVE1_X = 84,   BLUEHIVE1_Y = 86;
    public static double BLUEHIVE2_X = 84,   BLUEHIVE2_Y = 55;
    public static double REDHIVE1_X  = 57.5, REDHIVE1_Y  = 60;
    public static double REDHIVE2_X  = 57.5, REDHIVE2_Y  = 86;

    public static double HEADING_TOLERANCE = Math.toRadians(2);
    public static double CANCEL_STICK_PUSH = 0.3;   // push a stick past this to cancel the turn

    private boolean red = false;        // false = Blue
    private boolean secondHive = false; // false = Hive 1
    private double hiveX, hiveY;
    private boolean turning = false;
    private double targetHeading = 0, headingError = 0;

    public HiveTargeting_MOD2() { updateCoordinates(); }

    public boolean isRed()     { return red; }
    public boolean isTurning() { return turning; }

    public void toggleAlliance() { red = !red; secondHive = false; updateCoordinates(); }

    public void toggleHive() {
        if (turning) return;            // don't switch targets mid-turn
        secondHive = !secondHive;
        updateCoordinates();
    }

    private void updateCoordinates() {
        if (red) {
            hiveX = secondHive ? REDHIVE2_X  : REDHIVE1_X;
            hiveY = secondHive ? REDHIVE2_Y  : REDHIVE1_Y;
        } else {
            hiveX = secondHive ? BLUEHIVE2_X : BLUEHIVE1_X;
            hiveY = secondHive ? BLUEHIVE2_Y : BLUEHIVE1_Y;
        }
    }

    /** Start turning to face the hive. */
    public void startTurn(Follower follower) {
        if (turning) return;
        Pose p = follower.pose();
        targetHeading = GlobalState_MOD2.wrap(Math.atan2(hiveY - p.y(), hiveX - p.x()));
        follower.hold(new Pose(p.x(), p.y(), targetHeading));
        turning = true;
    }

    /** Call every loop. Ends the turn when lined up, or when the driver pushes a stick. */
    public void update(Follower follower, double stickPush) {
        if (!turning) return;
        headingError = GlobalState_MOD2.wrap(targetHeading - follower.pose().heading());
        if (Math.abs(headingError) < HEADING_TOLERANCE || stickPush > CANCEL_STICK_PUSH) {
            turning = false;
            follower.stop();
        }
    }

    public String status() {
        return String.format("%s / Hive %s  at (%.1f, %.1f)  turning=%b  error=%.1f deg",
                red ? "RED" : "BLUE", secondHive ? "2" : "1", hiveX, hiveY,
                turning, Math.toDegrees(headingError));
    }
}
