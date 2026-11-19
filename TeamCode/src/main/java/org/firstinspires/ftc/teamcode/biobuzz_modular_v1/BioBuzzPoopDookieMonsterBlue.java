package org.firstinspires.ftc.teamcode.biobuzz_modular_v1;

import static com.pedropathing.api.Paths.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.*;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.pedro.PedroCommands.hold;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;

@Autonomous
public class BioBuzzPoopDookieMonsterBlue extends OpMode {

    // ============================== POSES (yours, unchanged) ==============================
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(82.5, 132.4, -90);
    private final Pose shootPose = p.of(82.5, 123.5, -90);
    private final Pose chillp    = p.of(127.4, 103.9, 180);
    private final Pose park      = p.of(127.5, 34.9, 180);
    private final Pose park2     = p.of(127.5, 32, 180);

    // ============================== TUNABLES ==============================
    private static final double LAUNCH_TPS = 1280;   // was lvelo

    // ============================== SUBSYSTEMS ==============================
    private Follower follower;
    private Launcher launcher;
    private Transfer transfer;
    private ShooterCommands shooter;
    private IMU imu;

    // ============================== PATHS (yours, unchanged) ==============================
    private Path shoot() { return line(startPose, shootPose).linear(startPose, shootPose); }
    private Path park2() { return line(park, park2).linear(park, park2); }
    private Path park()  { return curve(shootPose, chillp, park).linear(shootPose, park).tangent(); }

    // ============================== ROUTINE ==============================
    private Command routine() {
        return sequential(
                shooter.spinUp(LAUNCH_TPS),            // spin up NOW, while driving
                follow(follower, shoot()),
                // hold position until launch() finishes: wait for speed, then run transfer ~3 s
                deadline(shooter.launch(), hold(follower)),
                shooter.stopAll(),
                follow(follower, park()),
                follow(follower, park2())
        );
    }

    // ============================== OPMODE ==============================
    @Override
    public void init() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Scheduler.reset();   // Scheduler is static; clear anything left from a previous OpMode

        // ---- BEFORE ANYTHING ELSE: record the global reference frame for TeleOp ----
        GlobalState.reset();
        GlobalState.startPose = GlobalState.copy(startPose);
        imu = GlobalState.initImu(hardwareMap);
        GlobalState.startImuYaw = GlobalState.yaw(imu);

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        launcher = new Launcher(hardwareMap);
        transfer = new Transfer(hardwareMap);
        shooter  = new ShooterCommands(launcher, transfer);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Start pose", "%.1f, %.1f, %.0f deg",
                startPose.x(), startPose.y(), Math.toDegrees(startPose.heading()));
        telemetry.addData("IMU yaw (deg)", "%.1f", Math.toDegrees(GlobalState.yaw(imu)));
        telemetry.update();
    }

    @Override
    public void start() {
        // Re-capture right at start (robot may have been nudged during init) and mark auto as live.
        GlobalState.startImuYaw = GlobalState.yaw(imu);
        GlobalState.startPose = GlobalState.copy(startPose);
        GlobalState.autoStarted = true;

        schedule(routine());
    }

    @Override
    public void loop() {
        follower.update();    // localization + path following
        launcher.update();    // battery-voltage compensation of flywheel F
        Scheduler.execute();  // Ivy commands

        Pose pose = follower.pose();
        telemetry.addData("Pose", "%.1f, %.1f, %.0f deg", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        launcher.addTelemetry(telemetry);
        transfer.addTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Save where we ended up for TeleOp (runs even if auto is cut short).
        if (follower != null) GlobalState.endPose = GlobalState.copy(follower.pose());
        if (imu != null) GlobalState.endImuYaw = GlobalState.yaw(imu);
        GlobalState.endTimeMs = System.currentTimeMillis();

        if (transfer != null) transfer.stop();
        if (launcher != null) launcher.stop();
    }
}
