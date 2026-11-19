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
public class BioBuzzPoopDookieMonsterBlue_MOD2 extends OpMode {

    // Poses and paths live in AutoPaths.java. Multi-step actions live in Actions.java.
    private final AutoPaths_MOD2 paths = new AutoPaths_MOD2(false);   // false = BLUE

    private Follower follower;
    private Launcher_MOD2 launcher;
    private Transfer_MOD2 transfer;
    private Intake_MOD2 intake;
    private Actions_MOD2 actions;
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
        Hubs_MOD2.enableBulkReads(hardwareMap);   // faster loops (see Hubs.java)
        Scheduler.reset();

        // Remember where/which way we started so TeleOp can use it
        GlobalState_MOD2.reset();
        imu = GlobalState_MOD2.initImu(hardwareMap);
        GlobalState_MOD2.startPose = GlobalState_MOD2.copy(paths.startPose);
        GlobalState_MOD2.startImuYaw = GlobalState_MOD2.yaw(imu);

        follower = Constants.create(hardwareMap);
        follower.setPose(paths.startPose);

        launcher = new Launcher_MOD2(hardwareMap);
        transfer = new Transfer_MOD2(hardwareMap);
        intake   = new Intake_MOD2(hardwareMap);
        actions  = new Actions_MOD2(launcher, transfer, intake);
    }

    @Override
    public void start() {
        GlobalState_MOD2.startImuYaw = GlobalState_MOD2.yaw(imu);   // re-read right at start
        GlobalState_MOD2.autoStarted = true;
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
        if (follower != null) GlobalState_MOD2.endPose = GlobalState_MOD2.copy(follower.pose());
        if (imu != null) GlobalState_MOD2.endImuYaw = GlobalState_MOD2.yaw(imu);
        if (transfer != null) transfer.halt();
        if (launcher != null) launcher.halt();
        if (intake != null) intake.halt();
    }
}
