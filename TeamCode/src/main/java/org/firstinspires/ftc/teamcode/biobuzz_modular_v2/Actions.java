package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

/**
 * Multi-step robot actions, built from the Intake / Transfer / Launcher commands.
 * Autos just call these (actions.shoot(), ...) so they stay short.
 *
 * TO ADD A NEW ACTION: copy one of the methods below, rename it, and change the steps.
 */
public class Actions {

    private final Launcher launcher;
    private final Transfer transfer;
    private final Intake intake;

    public Actions(Launcher launcher, Transfer transfer, Intake intake) {
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
