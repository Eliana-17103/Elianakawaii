package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/** Alliance/hive selection plus "turn to face the hive" using Pedro's hold(). */
public class HiveTargeting {
    // ---- coordinates ----
    public static double BLUEHIVE1_X = 84,   BLUEHIVE1_Y = 86;
    public static double BLUEHIVE2_X = 84,   BLUEHIVE2_Y = 55;
    // IMPORTANT: red values are still placeholders (as in your original)
    public static double REDHIVE1_X = 57.5,  REDHIVE1_Y = 60;
    public static double REDHIVE2_X = 57.5,  REDHIVE2_Y = 86;

    // ---- turning ----
    public static double HEADING_TOLERANCE = Math.toRadians(2);
    public static double LAUNCHER_FACING_OFFSET_RAD = 0; // Math.PI if the launcher faces the BACK
    public static long   TURN_TIMEOUT_MS = 2000;
    public static double CANCEL_STICK = 0.3;             // moving a stick past this cancels the turn

    private boolean red = false, alt = false;
    private double hiveX, hiveY;
    private boolean turning = false;
    private double targetHeading = 0, headingError = 0;
    private final ElapsedTime timer = new ElapsedTime();

    public HiveTargeting() { updateCoordinates(); }

    public boolean isTurning() { return turning; }
    public boolean isRed()     { return red; }

    public void toggleAlliance() {
        red = !red;
        alt = false;
        updateCoordinates();
    }

    public void toggleHive() {
        if (turning) return;   // don't change target mid-turn
        alt = !alt;
        updateCoordinates();
    }

    private void updateCoordinates() {
        if (red) { hiveX = alt ? REDHIVE2_X : REDHIVE1_X;   hiveY = alt ? REDHIVE2_Y : REDHIVE1_Y; }
        else     { hiveX = alt ? BLUEHIVE2_X : BLUEHIVE1_X; hiveY = alt ? BLUEHIVE2_Y : BLUEHIVE1_Y; }
    }

    public void startTurn(Follower follower) {
        if (turning) return;
        Pose p = follower.pose();
        targetHeading = GlobalState.wrap(Math.atan2(hiveY - p.y(), hiveX - p.x()) + LAUNCHER_FACING_OFFSET_RAD);
        follower.hold(new Pose(p.x(), p.y(), targetHeading));
        turning = true;
        timer.reset();
    }

    /** Call every loop. Ends the turn when aligned, timed out, or the driver grabs a stick. */
    public void update(Follower follower, double stickMagnitude) {
        if (!turning) return;
        headingError = GlobalState.wrap(targetHeading - follower.pose().heading());
        if (Math.abs(headingError) < HEADING_TOLERANCE
                || timer.milliseconds() > TURN_TIMEOUT_MS
                || stickMagnitude > CANCEL_STICK) {
            turning = false;
            follower.stop();
        }
    }

    public void addTelemetry(Telemetry t) {
        t.addLine("===== HIVE =====");
        t.addData("Alliance / Hive", "%s / %s", red ? "RED" : "BLUE", alt ? "2" : "1");
        t.addData("Hive X / Y", "%.1f / %.1f", hiveX, hiveY);
        t.addData("Target / error", "%.1f / %.1f deg", Math.toDegrees(targetHeading), Math.toDegrees(headingError));
        t.addData("Turning", turning);
    }
}
