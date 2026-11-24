package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class autoTesting extends LinearOpMode {

    private Follower follower;
    private Robot robot;

    protected PoseFactory poseFactory = PoseFactory.degrees();

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
                instant(() -> storeAllianceColor()), // Store alliance

                // Deploy Robot at starting pose
                instant(() -> robot.deploy()),
                waitMs(500), // allow time for elevator to raise
                instant(() -> robot.shooter.elevator.toggleBottomPos()),
                waitMs(1500), // ensure robot deployment

                // Launch 4 preloaded pollen
                instant(() -> robot.shooter.calculateShoot(4, 0)),
                instant(() -> robot.shooter.launcher.toggleShoot()),
                waitMs(3000), // wait during launch

                // Move to garden and intake the 4 pollen there
                follow(follower, moveToGarden()), // Lines up in front of garden
                instant(() -> robot.intake.toggle()), // Enable intake (matches TeleOP toggle if intake starts in rest)
                follow(follower, pickupFromGarden()), // Drives into pollen
                waitMs(2500), // Wait to ensure pollen are picked up

                // Go to other hive and shoot, hopefully after a tip
                follow(follower, moveToFarHive()),
                waitMs(500),
                // Launch 4 pollen
                instant(() -> robot.shooter.calculateShoot(4, 0)),
                instant(() -> robot.shooter.launcher.toggleShoot()),
                waitMs(3000), // wait during launch

                instant(() -> robot.intake.rest()), // Stop intake

                follow(follower, park())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = PedroConstants.create(hardwareMap);
        robot = new Robot(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();
            robot.update();

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

    public void storeAllianceColor() {}; // will be overridden via inhertiance for diff colors
}