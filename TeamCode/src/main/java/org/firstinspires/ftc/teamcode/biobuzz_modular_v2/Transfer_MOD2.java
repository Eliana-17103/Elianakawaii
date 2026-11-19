package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * The transfer motor: pushes balls into the launcher.
 *
 *   Direct methods (TeleOp):  run(), halt()
 *   Ivy commands (Auto):      feed(), stopFeed(), feedFor(ms), feedFor()
 */
public class Transfer_MOD2 {

    public static double POWER = -1.0;
    public static int RUN_MS = 3000;     // how long feedFor() runs (about 3 seconds)

    private final DcMotor motor;
    private boolean running = false;

    public Transfer_MOD2(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "transfer");
    }

    // ============ Direct methods (TeleOp) ============
    public void run()  { motor.setPower(POWER); running = true; }
    public void halt() { motor.setPower(0);     running = false; }

    // ============ Ivy commands (Auto) ============
    public Command feed()     { return instant(this::run); }     // start (keeps running)
    public Command stopFeed() { return instant(this::halt); }

    /** Run for a set time, then stop. */
    public Command feedFor(int ms) {
        return sequential(
                instant(this::run),
                waitMs(ms),
                instant(this::halt)
        );
    }
    public Command feedFor() { return feedFor(RUN_MS); }

    public String status() { return running ? "RUNNING" : "off"; }
}
