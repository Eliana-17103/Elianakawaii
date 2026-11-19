package org.firstinspires.ftc.teamcode.biobuzz;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Data handed from Autonomous -> TeleOp.
 *
 * We store DATA (poses and angles), not the Follower object itself: a Follower holds hardware
 * handles created by the previous OpMode, which can go stale once that OpMode ends. TeleOp builds
 * a fresh Follower and is then told where the robot is.
 *
 * Static fields live until the robot controller app restarts / robot powers off.
 */
public final class GlobalState {
    private GlobalState() {}

    // ---- written by Auto ----
    public static boolean autoStarted = false;        // true once Auto's start() ran
    public static Pose startPose = new Pose(0, 0, 0); // Pedro field pose at the very start of auto
    public static double startImuYaw = 0;             // Control Hub IMU yaw (rad) at the start of auto
    public static Pose endPose = new Pose(0, 0, 0);   // Pedro pose when auto stopped
    public static double endImuYaw = 0;               // Control Hub IMU yaw (rad) when auto stopped
    public static long endTimeMs = 0;                 // wall clock when auto stopped (guards stale data)

    public static void reset() {
        autoStarted = false;
        startPose = new Pose(0, 0, 0);
        endPose = new Pose(0, 0, 0);
        startImuYaw = endImuYaw = 0;
        endTimeMs = 0;
    }

    /** How far the robot rotated during auto, measured by the Control Hub IMU (CCW positive, rad). */
    public static double imuRotationDuringAuto() { return wrap(endImuYaw - startImuYaw); }

    /** Same thing, measured by Pedro/Pinpoint. Use as a cross-check. */
    public static double pedroRotationDuringAuto() { return wrap(endPose.heading() - startPose.heading()); }

    public static long ageMs() { return System.currentTimeMillis() - endTimeMs; }

    public static Pose copy(Pose p) { return new Pose(p.x(), p.y(), p.heading()); }

    // ---- IMU helpers shared by Auto + TeleOp so both use the SAME mounting orientation ----
    public static IMU initImu(HardwareMap hw) {
        IMU imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        return imu;
    }

    public static double yaw(IMU imu) {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public static double wrap(double r) {
        while (r > Math.PI) r -= 2 * Math.PI;
        while (r < -Math.PI) r += 2 * Math.PI;
        return r;
    }
}
