// BIG FAT NOTE:
// WE PROBABLY WON'T USE THIS CLASS. INSTEAD, USE THIS AS A
// GUIDE TO WRITE YOUR PATHS IN AutoPaths.java, AS IT WILL
// BE MUCH EASIER TO FIND AND EDIT.


// OTHER BIG FAT NOTE: WE COULD ALSO HAVE THEM BE: AUTO1, AND PATHS1,
// ETC., TO KEEP IT MODULAR. HOWEVER, THEN CERTAIN POSES THAT REMAIN
// UNCHANGED BETWEEN AUTOS ARE OVERUSED. FIGURE THIS OUT BY 11/19.












package org.firstinspires.ftc.teamcode.biobuzz_modular_v2;

// ============================================================================================
//  SKELETON PATHS  --  the one file you edit to change a route.
//
//  POSE  = a place on the field:  p.of(x, y, headingInDegrees).
//          Pedro's field is about 141.5 x 141.5. Get exact numbers from the Pedro Visualizer.
//  PATH  = how to drive between poses (a method that returns a Path).
//
//  Draw everything for BLUE. new SkeletonPaths(true) makes RED by mirroring every pose.
//
//  HOW TO ADD A PLACE:  1) add a  `public final Pose name;`  line   2) set it in the constructor
//  HOW TO ADD A PATH:   copy a commented example at the bottom, rename it, change the poses.
//  Lines starting with // are examples. Uncomment what you need.
// ============================================================================================

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public class SkeletonPaths {

    public static double MIRROR_LINE = 70.75;          // center of the field

    // ========================= POSES =========================
    public final Pose startPose;                        // LIVE: where the robot starts
    // public final Pose shootPose;
    // public final Pose pickupPose;
    // public final Pose controlPose;                   // a curve's "bend" point
    // public final Pose parkPose;

    public SkeletonPaths(boolean mirrorForRed) {
        PoseFactory p = PoseFactory.degrees();
        if (mirrorForRed) p = p.mirrorX(MIRROR_LINE);

        startPose = p.of(82.5, 132.4, -90);             // TODO: your starting position
        // shootPose   = p.of(82.5, 123.5, -90);        // where we shoot from
        // pickupPose  = p.of(100, 84, 0);              // where we pick up balls
        // controlPose = p.of(127.4, 103.9, 180);
        // parkPose    = p.of(127.5, 34.9, 180);
    }

    // ========================= PATHS =========================
    //   line(a, b)        straight line from a to b
    //   curve(a, c, b)    curve from a to b, bending toward c
    //   .linear(a, b)     smoothly turn from a's heading to b's heading while driving
    //   .tangent()        face the direction of travel

    // EXAMPLE: straight line
    // public Path toShoot() {
    //     return line(startPose, shootPose).linear(startPose, shootPose);
    // }

    // EXAMPLE: straight line to a pickup spot
    // public Path toPickup() {
    //     return line(shootPose, pickupPose).linear(shootPose, pickupPose);
    // }

    // EXAMPLE: curve
    // public Path toPark() {
    //     return curve(pickupPose, controlPose, parkPose).linear(pickupPose, parkPose).tangent();
    // }
}
