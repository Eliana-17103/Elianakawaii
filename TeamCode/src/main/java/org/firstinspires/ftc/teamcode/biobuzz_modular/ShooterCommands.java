package org.firstinspires.ftc.teamcode.biobuzz_modular;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Ivy commands for shooting (Auto uses these; TeleOp doesn't need Ivy). */
public class ShooterCommands {
    public static int STABLE_MS = 40;          // flywheel must stay in band this long
    public static int READY_TIMEOUT_MS = 2500; // transfer runs anyway after this, so auto never hangs
    public static int FINISH_WAIT_MS = 300;    // let the last ball clear the wheel

    private final Launcher launcher;
    private final Transfer transfer;
    private final ElapsedTime stable = new ElapsedTime(), timeout = new ElapsedTime();

    public ShooterCommands(Launcher launcher, Transfer transfer) {
        this.launcher = launcher;
        this.transfer = transfer;
    }

    public Command spinUp(double tps) {
        return instant(() -> launcher.setVelocity(tps));
    }

    /** Finishes once the flywheel has held its speed band for STABLE_MS (or on timeout). */
    public Command waitReady() {
        return Command.build()
                .setStart(() -> { stable.reset(); timeout.reset(); })
                .setExecute(() -> { if (!launcher.atSpeed()) stable.reset(); })
                .setDone(() -> stable.milliseconds() >= STABLE_MS
                            || timeout.milliseconds() >= READY_TIMEOUT_MS);
    }

    /** Wait for speed, run the transfer for Transfer.RUN_MS, stop it. */
    public Command launch() {
        return sequential(
                waitReady(),
                instant(transfer::run),
                waitMs(Transfer.RUN_MS),
                instant(transfer::stop),
                waitMs(FINISH_WAIT_MS)
        );
    }

    public Command stopAll() {
        return instant(() -> { transfer.stop(); launcher.stop(); });
    }
}
