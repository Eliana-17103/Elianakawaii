package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedro.procedures.Constants;

@Autonomous
public class BioBuzzPoopDookieMonsterRed extends OpMode {

    // Poses and paths live in AutoPaths.java. Multi-step actions live in Actions.java.
    private final AutoPaths paths = new AutoPaths(true);   // true = RED (every pose mirrored)

    private Follower follower;
    private Launcher launcher;
    private Transfer transfer;
    private Intake intake;
    private Actions actions;
    private IMU imu;

    // ======================= ROUTINE =======================
    private Command routine() {
        return sequential(
                actions.spinUp(),                       // flywheel starts NOW, while we drive
                follow(follower, paths.toShoot()),
                actions.shoot(),                        // wait for speed, run transfer ~3 s, spin down
                follow(follower, paths.toPark()),
                follow(follower, paths.toPark2())
        );
    }

    // ======================= OPMODE =======================
    @Override
    public void init() {
        Hubs.enableBulkReads(hardwareMap);   // faster loops (see Hubs.java)
        Scheduler.reset();

        // Remember where/which way we started so TeleOp can use it
        GlobalState.reset();
        imu = GlobalState.initImu(hardwareMap);
        GlobalState.startPose = GlobalState.copy(paths.startPose);
        GlobalState.startImuYaw = GlobalState.yaw(imu);

        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPose);

        launcher = new Launcher(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake   = new Intake(hardwareMap);
        actions  = new Actions(launcher, transfer, intake);
    }

    @Override
    public void start() {
        GlobalState.startImuYaw = GlobalState.yaw(imu);   // re-read right at start
        GlobalState.autoStarted = true;
        schedule(routine());
    }

    @Override
    public void loop() {
        follower.update();
        launcher.update();     // battery-sag correction
        Scheduler.execute();

        telemetry.addData("Pose", "%.1f, %.1f, %.0f deg", follower.pose().x(), follower.pose().y(),
                Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Launcher", launcher.status());
        telemetry.addData("Transfer", transfer.status());
        telemetry.update();
    }

    @Override
    public void stop() {
        // Save where we ended up for TeleOp
        if (follower != null) GlobalState.endPose = GlobalState.copy(follower.pose());
        if (imu != null) GlobalState.endImuYaw = GlobalState.yaw(imu);
        if (transfer != null) transfer.halt();
        if (launcher != null) launcher.halt();
        if (intake != null) intake.halt();
    }
}
