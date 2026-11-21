package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("left_front_drive");
        c.frontRightName.set("right_front_drive");
        c.backLeftName.set("left_back_drive");
        c.backRightName.set("right_back_drive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("poop");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(1.9198956827479086);
        c.yPodOffset.set(7.197626068836123);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });


    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2725659052525252);
                Controller secondaryTranslationalForward = Controller.proportional(0.10070581889568503);
                Controller primaryTranslationalLateral = Controller.proportional(0.3469017864336354);
                Controller secondaryTranslationalLateral = Controller.proportional(0.12817094070077803);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.017655651759483967));
                c.brake.set(Controller.proportionalFeedforward(0.01500730399556137));

                c.headingFeedback.set(Controller.proportional(4.939850884372395));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04093495182418609, 0.005738075016938826));

                c.linearBrakeCoefficients.set(Matrix.diag(0.06481540905980035, 0.057439717382121454));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0015911788031759662, 0.0014757961171201939));

                c.maxAchievableForwardVelocity.set(59.699773823009494);
                c.maxAchievableStrafeVelocity.set(48.61614969971913);
                c.naturalForwardDeceleration.set(30.54965791484597);
                c.naturalStrafeDeceleration.set(58.87283397986546);
            }
    );


    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}