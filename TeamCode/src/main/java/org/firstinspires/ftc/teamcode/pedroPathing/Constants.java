package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * pedro pathing configuration constants.
 * uses SwerveDrivetrain with unified SwerveConstants.
 */
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants();
    public static PinpointConstants pinpointConstants = new PinpointConstants();
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    /**
     * create follower with custom swerve drivetrain and pinpoint localizer
     */
    public static Follower createFollower(HardwareMap hardwareMap) {
        SwerveDrivetrain drivetrain = new SwerveDrivetrain(hardwareMap);
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setDrivetrain(drivetrain)
                .pinpointLocalizer(pinpointConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}