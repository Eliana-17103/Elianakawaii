package org.firstinspires.ftc.teamcode.biobuzz_modular_v1;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * The driver's heading frame for field-centric drive.
 *
 * Heading is measured relative to the way the robot was facing at the START OF AUTO (carried over
 * via GlobalState), not relative to wherever the robot happens to point when TeleOp starts.
 */
public class HeadingFrame_MOD1 {
    public static boolean USE_PEDRO_FOR_AUTO_ROTATION = false; // false = Control Hub IMU, true = Pinpoint
    public static long MAX_CARRY_AGE_MS = 10 * 60 * 1000;      // ignore stale auto data older than this
    public static double BACK_IS_FRONT_OFFSET = Math.PI;       // robot's back is treated as the front

    private final IMU imu;
    private double zeroYaw = 0;
    private boolean carried = false;

    public HeadingFrame_MOD1(IMU imu) { this.imu = imu; }

    public static boolean autoDataAvailable() {
        return GlobalState_MOD1.autoStarted && GlobalState_MOD1.ageMs() < MAX_CARRY_AGE_MS;
    }

    /** Call once at TeleOp start. */
    public void establish(boolean useAutoData) {
        double yawNow = GlobalState_MOD1.yaw(imu);
        carried = useAutoData && autoDataAvailable();
        if (carried) {
            double rotation = USE_PEDRO_FOR_AUTO_ROTATION
                    ? GlobalState_MOD1.pedroRotationDuringAuto()
                    : GlobalState_MOD1.imuRotationDuringAuto();
            // The robot has turned `rotation` since the start of auto.
            zeroYaw = GlobalState_MOD1.wrap(yawNow - rotation);
        } else {
            zeroYaw = yawNow;   // old behaviour: "forward" = however it faces now
        }
    }

    public boolean isCarriedFromAuto() { return carried; }

    /** Treat the robot's current facing as the auto-start direction. */
    public void rezero() { zeroYaw = GlobalState_MOD1.yaw(imu); }

    public double heading()      { return GlobalState_MOD1.wrap(GlobalState_MOD1.yaw(imu) - zeroYaw); }
    public double driveHeading() { return heading() + BACK_IS_FRONT_OFFSET; }

    public void addTelemetry(Telemetry t) {
        t.addLine("===== FIELD-CENTRIC FRAME =====");
        t.addData("Carried from auto", carried);
        t.addData("Heading vs auto-start", "%.1f deg", Math.toDegrees(heading()));
    }
}
