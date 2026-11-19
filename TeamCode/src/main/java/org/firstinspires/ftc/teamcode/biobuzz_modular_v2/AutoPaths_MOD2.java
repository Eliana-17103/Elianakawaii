package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * ALL poses and paths for the main autonomous live here. This is the only file you edit to change a route.
 *
 *   new AutoPaths(false)  -> BLUE, exactly the numbers below
 *   new AutoPaths(true)   -> RED, every pose mirrored across the field (position AND heading)
 *
 * Draw everything for BLUE. Red is made automatically.
 *
 * TO CHANGE A ROUTE:   edit the numbers in the pose list.
 * TO ADD A NEW PATH:   1) add a pose (if you need a new place)   2) copy a path method and rename it.
 */
public class AutoPaths_MOD2 {

    public static double MIRROR_LINE = 70.75;   // center of the field (field is 141.5 x 141.5)

    // ========================= POSES (x, y, heading in degrees) =========================
    public final Pose startPose;
    public final Pose shootPose;
    public final Pose controlPose;   // the "bend" point of the park curve
    public final Pose parkPose;
    public final Pose park2Pose;

    public AutoPaths_MOD2(boolean mirrorForRed) {
        PoseFactory p = PoseFactory.degrees();
        if (mirrorForRed) p = p.mirrorX(MIRROR_LINE);

        startPose   = p.of(82.5, 132.4, -90);
        shootPose   = p.of(82.5, 123.5, -90);
        controlPose = p.of(127.4, 103.9, 180);
        parkPose    = p.of(127.5, 34.9, 180);
        park2Pose   = p.of(127.5, 32, 180);
    }

    // ========================= PATHS =========================
    //   line(a, b)        straight line
    //   curve(a, c, b)    curve from a to b, bending toward c
    //   .linear(a, b)     smoothly turn from a's heading to b's heading
    //   .tangent()        face the direction of travel

    public Path toShoot() {
        return line(startPose, shootPose).linear(startPose, shootPose);
    }

    public Path toPark() {
        return curve(shootPose, controlPose, parkPose).linear(shootPose, parkPose).tangent();
    }

    public Path toPark2() {
        return line(parkPose, park2Pose).linear(parkPose, park2Pose);
    }
}
