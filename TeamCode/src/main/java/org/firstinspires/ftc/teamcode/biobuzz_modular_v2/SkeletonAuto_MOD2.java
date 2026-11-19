// BIG FAT NOTE: READ THE SkeletonPaths.java TOP FEW LINES.
// APPLY THIS TO THE GUIDE AT THE BEGINNING OF THIS CLASS.
// ESSENTIALLY, CHANGE THE SkeletonPaths TO AutoPaths, AND
// YOU WON'T NEED TO COPY ANY PATHING FILE (LIKE SkeletonPaths)


// OTHER BIG FAT NOTE: WE COULD ALSO HAVE THEM BE: AUTO1, AND PATHS1,
// ETC., TO KEEP IT MODULAR. HOWEVER, THEN CERTAIN POSES THAT REMAIN
// UNCHANGED BETWEEN AUTOS ARE OVERUSED. FIGURE THIS OUT BY 11/19.












package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

// ============================================================================================
//  SKELETON AUTONOMOUS  (Pedro Pathing + Ivy)
//
//  QUICK START (about 2 minutes)
//    1. Copy this file and SkeletonPaths.java. Rename both (e.g. MyAuto / MyPaths) and change
//       the class names, the constructor, and `new SkeletonPaths(...)` below to match.
//    2. In MyPaths: add poses + paths for the route (see the examples in that file).
//    3. Below, uncomment the robot parts you use (section 1) and their lines in init() (section 3).
//    4. In routine() (section 2), uncomment / copy the steps you want, in order.
//    5. Pick BLUE or RED with the true/false in section 1. Done.
//
//  WHERE THINGS LIVE
//    MyPaths.java    = poses + paths                 (change a route here)
//    Actions.java    = multi-step actions, shoot()   (add a new action there)
//    this file       = the ORDER things happen
//
//  HOW IT WORKS
//    routine() is a list of commands that Ivy runs ONE AFTER ANOTHER (sequential).
//    loop() calls follower.update() (Pedro) and Scheduler.execute() (Ivy) every loop.
//
//  RULES
//    - Every step inside sequential(...) ends with a COMMA. Keep the last line `waitMs(0)`.
//    - Lines starting with // are examples. "LIVE" means it is real code already.
//    - Ivy extras (parallel, deadline, repeat, ...) need their import uncommented. Check the Ivy
//      docs (command-compositions / utilities-and-decorators) for their exact behavior.
// ============================================================================================

import static com.pedropathing.ivy.Scheduler.schedule;             // starts a command
import static com.pedropathing.ivy.commands.Commands.waitMs;       // wait for N milliseconds
import static com.pedropathing.ivy.groups.Groups.sequential;       // run commands one after another

// ---- Ivy extras: uncomment the import for any you use ----
// import static com.pedropathing.ivy.commands.Commands.instant;    // run one line of code once
// import static com.pedropathing.ivy.commands.Commands.waitUntil;  // wait until something is true
// import static com.pedropathing.ivy.groups.Groups.parallel;       // run commands AT THE SAME TIME
// import static com.pedropathing.ivy.groups.Groups.deadline;       // run until the FIRST command ends
// import static com.pedropathing.ivy.groups.Groups.repeat;         // run a command N times
// import static com.pedropathing.ivy.pedro.PedroCommands.hold;     // hold the robot's position

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;

@Autonomous(name = "SKELETON Auto", group = "Skeleton")
public class SkeletonAuto_MOD2 extends OpMode {

    // ==========================================================================================
    // 1. PATHS + ROBOT PARTS
    // ==========================================================================================
    private final SkeletonPaths_MOD2 paths = new SkeletonPaths_MOD2(false);   // LIVE: false = BLUE, true = RED

    private Follower follower;       // LIVE: Pedro (drives the robot)
    private IMU imu;                 // LIVE: saves the starting heading for TeleOp

    // Uncomment the parts you use (and their lines in init()):
    // private Launcher launcher;
    // private Transfer transfer;
    // private Intake intake;
    // private Defense defense;
    // private Actions actions;      // needs launcher + transfer + intake

    // ==========================================================================================
    // 2. THE ROUTINE  -- the whole autonomous, top to bottom
    //
    //    TYPICAL AUTO (copy these lines into the list below):
    //        actions.spinUp(),                    // flywheel on while we drive
    //        follow(follower, paths.toShoot()),
    //        actions.shoot(),                     // wait for speed, transfer ~3 s, flywheel off
    //        follow(follower, paths.toPark()),
    // ==========================================================================================
    private Command routine() {
        return sequential(

                // ----- DRIVING (paths come from SkeletonPaths.java) -----
                // follow(follower, paths.toShoot()),              // drive a path
                // follow(follower, paths.toPark()),

                // ----- WAITING -----
                // waitMs(500),                                     // pause 0.5 seconds

                // ----- ACTIONS (Actions.java) -----
                // actions.spinUp(),                                // start the flywheel
                // actions.shoot(),                                 // wait for speed, transfer ~3 s, spin down
                // actions.spinUpAndShoot(),                        // both, for when you're already parked
                // actions.intakeFor(2000),                         // intake for 2 s, then stop
                // actions.stopAll(),                               // everything off

                // ----- LAUNCHER (single parts, for custom timing) -----
                // launcher.spinUp(),                               // flywheel to Launcher.AUTO_SPEED
                // launcher.spinUp(1300),                           // flywheel to a specific speed
                // launcher.waitUntilReady(),                       // wait until at speed (gives up after 2.5 s)
                // launcher.spinDown(),

                // ----- TRANSFER -----
                // transfer.feedFor(),                              // run ~3 s (Transfer.RUN_MS), then stop
                // transfer.feedFor(1500),                          // run for 1.5 s, then stop
                // transfer.feed(),                                 // start and keep running...
                // transfer.stopFeed(),                             // ...until you stop it

                // ----- INTAKE -----
                // intake.collect(),                                // start pulling balls in
                // intake.eject(),                                  // push balls out
                // intake.off(),                                    // stop
                // intake.collectFor(2000),                         // intake for 2 s, then stop

                // ----- DEFENSE ARM -----
                // defense.raise(),
                // defense.lower(),

                // ----- IVY EXTRAS (uncomment the matching import at the top) -----
                // instant(() -> launcher.setVelocity(1400)),       // run any one line of code, once
                // waitUntil(launcher::atSpeed),                    // wait until something is true
                // repeat(transfer.feedFor(1000), 2),               // do a command 2 times
                // parallel(                                        // do things AT THE SAME TIME
                //         follow(follower, paths.toShoot()),
                //         actions.spinUp()
                // ),
                // deadline(                                        // stop when the FIRST one ends
                //         actions.shoot(),
                //         hold(follower)                           // ...while holding position
                // ),

                // ----- YOUR OWN COMMAND (see customExample() below) -----
                // customExample(),

                waitMs(0)   // <-- KEEP THIS LAST. It lets every line above end with a comma.
        );
    }

    // EXAMPLE: build your own command from scratch.
    //   setStart   = runs once when the command begins
    //   setExecute = runs every loop while it's active
    //   setDone    = the command ends when this returns true
    // private Command customExample() {
    //     return Command.build()
    //             .setStart(() -> { })
    //             .setExecute(() -> { })
    //             .setDone(() -> true);
    // }

    // ==========================================================================================
    // 3. OPMODE (LIVE)
    // ==========================================================================================
    @Override
    public void init() {
        Hubs_MOD2.enableBulkReads(hardwareMap);   // LIVE: faster loops (see Hubs.java)
        Scheduler.reset();                   // clear leftovers from any earlier OpMode

        // Save the starting pose + heading so TeleOp's field-centric can use them.
        GlobalState_MOD2.reset();
        imu = GlobalState_MOD2.initImu(hardwareMap);
        GlobalState_MOD2.startPose = GlobalState_MOD2.copy(paths.startPose);
        GlobalState_MOD2.startImuYaw = GlobalState_MOD2.yaw(imu);

        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPose);

        // Uncomment the parts you use (and their fields in section 1):
        // launcher = new Launcher(hardwareMap);
        // transfer = new Transfer(hardwareMap);
        // intake   = new Intake(hardwareMap);
        // defense  = new Defense(hardwareMap);
        // actions  = new Actions(launcher, transfer, intake);
    }

    @Override
    public void start() {
        GlobalState_MOD2.startImuYaw = GlobalState_MOD2.yaw(imu);   // re-read right at start
        GlobalState_MOD2.autoStarted = true;
        schedule(routine());                              // hand the routine to Ivy
    }

    @Override
    public void loop() {
        follower.update();           // Pedro: localization + path following (EVERY loop)
        Scheduler.execute();         // Ivy: runs the commands (EVERY loop)
        // launcher.update();        // battery-sag correction (uncomment if you use the launcher)

        telemetry.addData("Pose", "%.1f, %.1f, %.0f deg", follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        // telemetry.addData("Launcher", launcher.status());
        // telemetry.addData("Transfer", transfer.status());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Save where we ended up so TeleOp starts from the right place.
        if (follower != null) GlobalState_MOD2.endPose = GlobalState_MOD2.copy(follower.pose());
        if (imu != null) GlobalState_MOD2.endImuYaw = GlobalState_MOD2.yaw(imu);

        // Turn mechanisms off (uncomment with the parts you use):
        // if (launcher != null) launcher.halt();
        // if (transfer != null) transfer.halt();
        // if (intake != null) intake.halt();
    }
}
