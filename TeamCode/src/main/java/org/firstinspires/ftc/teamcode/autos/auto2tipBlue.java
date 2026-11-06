package org.firstinspires.ftc.teamcode.autos;

import com.pedropathing.api.Paths;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "2tipBlue", group = "Autonomous")
public class auto2tipBlue extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.915, 131.2598, 90);
    private final Pose path1 = poseFactory.of(132.3114, 122.6853, 88.6189);
    private final Pose path1Control1 = poseFactory.of(82.916, 104.1176, 0);
    private final Pose path1Control2 = poseFactory.of(131.1777, 75.6628, 0);
    private final Pose point2 = poseFactory.of(132.2301, 130.9118, 90.5662);
    private final Pose point3 = poseFactory.of(82.9964, 15.5145, 90.3352);
    private final Pose point3Control1 = poseFactory.of(130.1517, 106.7118, 0);
    private final Pose point3Control2 = poseFactory.of(120.9349, 79.2577, 0);
    private final Pose point3Control3 = poseFactory.of(128.5202, 31.876, 0);
    private final Pose point3Control4 = poseFactory.of(82.9564, 22.3397, 0);
    private final Pose point4 = poseFactory.of(102.8247, 12.7713, -93.1585);
    private final Pose point4Control1 = poseFactory.of(104.067, 35.2839, 0);
    private final Pose point5 = poseFactory.of(83.3402, 11.3138, 90.1053);
    private final Pose point5Control1 = poseFactory.of(83.3012, 32.5022, 0);
    private final Pose point6 = poseFactory.of(131.3165, 46.3527, -0.1866);
    private final Pose point6Control1 = poseFactory.of(111.1303, 46.4185, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1Control2, path1).tangent();
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.curve(point2, point3Control1, point3Control2, point3Control3, point3Control4, point3).reverseTangent();
    }

    public Path path4() {
        return Paths.curve(point3, point4Control1, point4).tangent();
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).reverseTangent();
    }

    public Path path6() {
        return Paths.curve(point5, point6Control1, point6).tangent();
    }
}
