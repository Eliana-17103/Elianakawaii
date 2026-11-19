package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Speeds up the robot's main loop. Call Hubs.enableBulkReads(hardwareMap) once at the start of
 * every OpMode (Auto and TeleOp).
 *
 * WHAT IS A LynxModule?
 *   It's the code object for a REV hub: your Control Hub and Expansion Hub ("Lynx" is the name of the
 *   protocol REV hubs speak). Normally every sensor/encoder read (motor velocity, encoder position, ...)
 *   is its own separate trip to the hub, and each trip costs time. In AUTO bulk-caching mode, the
 *   hub hands over all its sensor data in ONE trip and the SDK reuses it, refreshing it when needed
 *   (for example after you change a motor power). Same numbers, noticeably faster loops, which
 *   helps Pedro and the flywheel velocity check react sooner.
 */
public final class Hubs_MOD2 {
    private Hubs_MOD2() {}

    public static void enableBulkReads(HardwareMap hardwareMap) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }
}
