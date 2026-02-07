package org.firstinspires.ftc.teamcode.Auto.PairedAuto.bttdb;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Pedro.Constants;
//import org.firstinspires.ftc.teamcode.Robot;
//import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
//import org.firstinspires.ftc.teamcode.Utils.Aliance;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous
public class TestAuto extends NextFTCOpMode{
    public TestAuto(){
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private Path scorePreload;
    private PathChain pickUpLastRow;
    private PathChain returnOne;
    private PathChain returnTwo;
    private PathChain pickUpLoadingZoneOne;
    private PathChain pickUpLoadingZoneTwo;


    private Pose startPose = new Pose(56,8);
    private Pose shootPose = new Pose();
    private Pose lastrowPose = new Pose(14.122, 35.816);
    private Pose loadingZonePose = new Pose(5,8);
    private Pose loadingZonePose2 =  new Pose(5.551, 20.265);
    private Pose endPose =  new Pose(72.020, 23.327);
    private Path moveForward;

    public void buildPath(){
          moveForward = new Path(new BezierCurve(
                new Pose(72,72),
                new Pose(72, 100)));
          moveForward.setConstantHeadingInterpolation(Math.toRadians(90));

    }
//    public void buildPaths(){
//        scorePreload = new Path(new BezierLine(startPose,shootPose));
//        pickUpLastRow = follower().pathBuilder().addPath(
//                        new BezierCurve(
//                                startPose,
//                                new Pose(51.245, 40.959),
//                                lastrowPose
//                        )
//                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
//                .build();
//
//        returnOne = follower().pathBuilder().addPath(
//                        new BezierLine(
//                                shootPose,
//                                startPose
//                        )
//                ).setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        pickUpLoadingZoneOne = follower().pathBuilder().addPath(
//                        new BezierLine(
//                                startPose,
//                                loadingZonePose
//                        )
//                ).setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        returnTwo = follower().pathBuilder().addPath(
//                        new BezierLine(
//                                loadingZonePose,
//                                startPose
//                        )
//                ).setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        pickUpLoadingZoneTwo = follower().pathBuilder().addPath(
//                        new BezierCurve(
//                                startPose,
//                                new Pose(31.276, 20.316),
//                                loadingZonePose2
//                        )
//                ).setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//
//        returnTwo = follower().pathBuilder().addPath(
//                        new BezierLine(
//                                loadingZonePose2,
//                                endPose)
//                ).setTangentHeadingInterpolation()
//                .build();
//    }
    public Command autoMoveForward(){
        return new FollowPath(moveForward);
    }

    @Override
    public void onStartButtonPressed(){
        buildPath();
        follower().setStartingPose(new Pose(72,72,Math.toRadians(90)));
        autoMoveForward().schedule();
    }
    @Override
    public void onUpdate(){
        telemetry.addData("X Position ", follower().getPose().getX());
        telemetry.addData("Y Position ", follower().getPose().getY());
        telemetry.addData("Get Pose", follower().getPose().getPose());
        telemetry.update();
    }
}
