package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import com.qualcomm.robotcore.hardware.IMU;

/**
 * Decides what "forward" means for field-centric driving.
 *
 * Old way: forward = however the robot was facing when TeleOp started (wrong if auto turned it).
 * This way: forward = the direction the robot faced at the START OF AUTO.
 *
 * The robot's BACK is treated as its front (same as your original TeleOp).
 */
public class HeadingFrame_MOD2 {

    private final IMU imu;
    private double zeroYaw = 0;
    private boolean usingAutoData = false;

    public HeadingFrame_MOD2(IMU imu) { this.imu = imu; }

    /** Call once when TeleOp starts. */
    public void establish(boolean useAutoData) {
        double yawNow = GlobalState_MOD2.yaw(imu);
        usingAutoData = useAutoData && GlobalState_MOD2.autoStarted;
        if (usingAutoData) {
            // The robot has turned rotationDuringAuto() since auto started, so back up by that much.
            zeroYaw = GlobalState_MOD2.wrap(yawNow - GlobalState_MOD2.rotationDuringAuto());
        } else {
            zeroYaw = yawNow;
        }
    }

    public boolean isUsingAutoData() { return usingAutoData; }

    /** "The robot is facing the way it did at the start of auto right now." */
    public void rezero() { zeroYaw = GlobalState_MOD2.yaw(imu); }

    public double heading()      { return GlobalState_MOD2.wrap(GlobalState_MOD2.yaw(imu) - zeroYaw); }
    public double driveHeading() { return heading() + Math.PI; }
}
