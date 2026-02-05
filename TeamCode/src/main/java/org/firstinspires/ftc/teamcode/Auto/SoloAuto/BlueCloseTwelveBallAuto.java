package org.firstinspires.ftc.teamcode.Auto.SoloAuto;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Autonomous
public class BlueCloseTwelveBallAuto extends NextFTCOpMode {
    //Robot robot = new Robot(Aliance.BLUE);
    public BlueCloseTwelveBallAuto(){
        addComponents(
                new SubsystemComponent(Drivetrain.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private Path scorePreload;
    private PathChain pickUpFirstRow;
    private PathChain returnOne;
    private PathChain pickUpSecondRow;
    private PathChain gateOne;
    private PathChain gateTwo;
    private PathChain returnTwo;
    private PathChain pickUpThirdRow;
    private PathChain end;

//private Pose startPose = new Pose();
//private Pose firstRowPose = new Pose();
//private Pose returnPose = new Pose();
//private Pose secondRowPose = new Pose();
//private Pose thirdRowPose = new Pose();
//private Pose endPose = new Pose();
    public void buildPaths(){
        pickUpFirstRow = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(24.980, 127.469),
                                new Pose(73.112, 78.684),
                                new Pose(16.714, 83.755)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180))

                .build();

        returnOne = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.714, 83.755),

                                new Pose(62.061, 83.816)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        pickUpSecondRow = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(62.061, 83.816),
                                new Pose(61.306, 57.449),
                                new Pose(16.061, 59.735)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        gateOne = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.061, 59.735),

                                new Pose(16.224, 71.449)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        gateTwo = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.224, 71.449),

                                new Pose(15.347, 70.959)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                .build();

        returnTwo = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.347, 70.959),

                                new Pose(73.020, 71.510)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        pickUpThirdRow = follower().pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(73.020, 71.510),
                                new Pose(75.306, 35.633),
                                new Pose(13.367, 34.939)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        end = follower().pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.367, 34.939),

                                new Pose(72.918, 71.449)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(142))

                .build();
    }

    public Command autonomousRoutine(){
        return new SequentialGroup(
                new FollowPath(pickUpFirstRow),
                new Delay(0.1),
                new FollowPath(returnOne),
                new Delay(0.1),
                new FollowPath(pickUpSecondRow),
                new Delay(0.1),
                new FollowPath(gateOne),
                new Delay(0.1),
                new FollowPath(gateTwo),
                new Delay(0.1),
                new FollowPath(returnTwo),
                new Delay(0.1),
                new FollowPath(pickUpThirdRow),
                new Delay(0.1),
                new FollowPath(end)
        );
    }

    @Override
    public void onStartButtonPressed(){
        buildPaths();
        autonomousRoutine().schedule();
    }
}

