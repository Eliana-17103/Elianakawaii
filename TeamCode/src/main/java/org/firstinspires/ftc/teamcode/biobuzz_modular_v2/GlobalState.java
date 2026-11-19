package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Data that Autonomous leaves behind for TeleOp (static = it survives between OpModes).
 * We save plain numbers/poses, NOT the Follower itself, because TeleOp builds its own Follower.
 */
public final class GlobalState {
    private GlobalState() {}

    public static boolean autoStarted = false;          // true once Auto pressed START
    public static Pose startPose = new Pose(0, 0, 0);   // Pedro pose at the start of auto
    public static Pose endPose   = new Pose(0, 0, 0);   // Pedro pose when auto stopped
    public static double startImuYaw = 0;               // IMU yaw (radians) at the start of auto
    public static double endImuYaw   = 0;               // IMU yaw (radians) when auto stopped

    public static void reset() {
        autoStarted = false;
        startPose = new Pose(0, 0, 0);
        endPose   = new Pose(0, 0, 0);
        startImuYaw = 0;
        endImuYaw = 0;
    }

    /** How far the robot turned during auto (radians, counter-clockwise positive). */
    public static double rotationDuringAuto() {
        return wrap(endImuYaw - startImuYaw);
    }

    public static Pose copy(Pose p) { return new Pose(p.x(), p.y(), p.heading()); }

    // ---- IMU helpers, shared so Auto and TeleOp use the SAME hub orientation ----
    // REV logo faces backward, USB port faces upward (same as your original TeleOp).
    public static IMU initImu(HardwareMap hardwareMap) {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)));
        return imu;
    }

    public static double yaw(IMU imu) {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    /** Keeps an angle between -PI and +PI. */
    public static double wrap(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
