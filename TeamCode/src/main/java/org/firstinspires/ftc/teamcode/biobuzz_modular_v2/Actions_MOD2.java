package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

/**
 * Multi-step robot actions, built from the Intake / Transfer / Launcher commands.
 * Autos just call these (actions.shoot(), ...) so they stay short.
 *
 * TO ADD A NEW ACTION: copy one of the methods below, rename it, and change the steps.
 */
public class Actions_MOD2 {

    private final Launcher_MOD2 launcher;
    private final Transfer_MOD2 transfer;
    private final Intake_MOD2 intake;

    public Actions_MOD2(Launcher_MOD2 launcher, Transfer_MOD2 transfer, Intake_MOD2 intake) {
        this.launcher = launcher;
        this.transfer = transfer;
        this.intake = intake;
    }

    /** Start the flywheel (it keeps spinning while the robot drives). */
    public Command spinUp() { return launcher.spinUp(); }

    /**
     * Shoot, assuming spinUp() was already called: wait until the flywheel is at speed,
     * run the transfer (~3 s), then spin the flywheel down.
     */
    public Command shoot() {
        return sequential(
                launcher.waitUntilReady(),
                transfer.feedFor(),
                launcher.spinDown()
        );
    }

    /** For when the robot is already parked: spin up, then shoot. */
    public Command spinUpAndShoot() {
        return sequential(
                launcher.spinUp(),
                shoot()
        );
    }

    /** Intake for a set time, then stop. */
    public Command intakeFor(int ms) { return intake.collectFor(ms); }

    /** Everything off. */
    public Command stopAll() {
        return sequential(
                launcher.spinDown(),
                transfer.stopFeed(),
                intake.off()
        );
    }
}
