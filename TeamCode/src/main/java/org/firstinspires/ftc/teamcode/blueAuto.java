package org.firstinspires.ftc.teamcode;

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

@Autonomous(name = "b-1tip", group = "Autonomous")
public class blueAuto extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(82.915, 131.2598, -90);
    private final Pose moveToGarden = poseFactory.of(128.0973, 112.8543, 90);
    private final Pose moveToGardenControl1 = poseFactory.of(82.916, 104.1176, 0);
    private final Pose moveToGardenControl2 = poseFactory.of(113.9938, 99.5902, 0);
    private final Pose pickupFromGardenStart = poseFactory.of(128.0973, 112.8543, 90);
    private final Pose pickupFromGarden = poseFactory.of(132.2301, 130.9118, 90);
    private final Pose moveToFarHive = poseFactory.of(82.9964, 15.5145, 90.3352);
    private final Pose moveToFarHiveControl1 = poseFactory.of(130.1517, 106.7118, 0);
    private final Pose moveToFarHiveControl2 = poseFactory.of(120.9349, 79.2577, 0);
    private final Pose moveToFarHiveControl3 = poseFactory.of(128.5202, 31.876, 0);
    private final Pose moveToFarHiveControl4 = poseFactory.of(82.9564, 22.3397, 0);
    private final Pose parkStart = poseFactory.of(82.9964, 15.5145, 90);
    private final Pose park = poseFactory.of(131.3165, 46.3527, 180);
    private final Pose parkControl1 = poseFactory.of(111.1303, 46.4185, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, moveToGarden()),
                follow(follower, pickupFromGarden()),
                follow(follower, moveToFarHive()),
                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = PedroConstants.create(hardwareMap);
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

    public Path moveToGarden() {
        return Paths.curve(start, moveToGardenControl1, moveToGardenControl2, moveToGarden).linear(start, moveToGarden);
    }

    public Path pickupFromGarden() {
        return Paths.line(pickupFromGardenStart, pickupFromGarden).linear(pickupFromGardenStart, pickupFromGarden);
    }

    public Path moveToFarHive() {
        return Paths.curve(pickupFromGarden, moveToFarHiveControl1, moveToFarHiveControl2, moveToFarHiveControl3, moveToFarHiveControl4, moveToFarHive).reverseTangent();
    }

    public Path park() {
        return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);
    }
}
