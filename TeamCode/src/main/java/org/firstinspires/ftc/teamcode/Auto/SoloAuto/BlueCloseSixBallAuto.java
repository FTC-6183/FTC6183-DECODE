package org.firstinspires.ftc.teamcode.Auto.SoloAuto;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.NextFTCPatch.SequentialGroupFixed;
import org.firstinspires.ftc.teamcode.Pedro.Constants;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@Autonomous
@Config
public class BlueCloseSixBallAuto extends NextFTCOpMode{
    public Path p1;
    public Path p2;
    public Path p3;
    public Path p4;
    public Path p5;
    public Path p6;
    public Path p7;

    public static double hoodPosition = 0;
    public static double velocity = 0;
    public static double transferFlickDelay = 0.4;
    public static double spindexerDelay = 0.6;

    public static Pose endPose = new Pose();

    public BlueCloseSixBallAuto(){
        addComponents(
                new SubsystemComponent(Spindexer.INSTANCE, Intake.INSTANCE, Turret.INSTANCE, Transfer.INSTANCE, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    public Command setToPositionOne(){
        return new InstantCommand(()->Spindexer.INSTANCE.setCurrentPosition(Spindexer.Position.POSITION_ONE));
    }

    public Command setToPositionTwo(){
        return new InstantCommand(()->Spindexer.INSTANCE.setCurrentPosition(Spindexer.Position.POSITION_TWO));
    }

    public Command setToPositionThree(){
        return new InstantCommand(()->Spindexer.INSTANCE.setCurrentPosition(Spindexer.Position.POSITION_THREE));
    }

    public Command transferFlick(){
        return new SequentialGroupFixed(
                Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown());

    }
    public Command shootThree(){
        return new SequentialGroupFixed(
                Intake.INSTANCE.on(),
                setToPositionOne(),
                new Delay(spindexerDelay),
                Turret.INSTANCE.waitToShoot(),
                Transfer.INSTANCE.transferUp(),
                new Delay(transferFlickDelay),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
                setToPositionTwo(),
                new Delay(spindexerDelay),
                Turret.INSTANCE.waitToShoot(),
                Transfer.INSTANCE.transferUp(),
                new Delay(transferFlickDelay),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
                setToPositionThree(),
                new Delay(spindexerDelay),
                Turret.INSTANCE.waitToShoot(),
                Transfer.INSTANCE.transferUp(),
                new Delay(transferFlickDelay),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY)),
                Intake.INSTANCE.idle()
                );
    }
    public Command intakeMode(){
        return new SequentialGroupFixed(
                Intake.INSTANCE.on(),
                new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE))
        );
    }


    public void buildPaths(){
        p1 = new Path(new BezierLine(
                new Pose(27.18367346938776, 130.04081632653063),
                new Pose(61.71467346938773, 94.22434693877551)
        ));
        p1.setLinearHeadingInterpolation(Math.toRadians(142), Math.toRadians(180));

        p2 = new Path(new BezierLine(
                new Pose(61.71467346938773, 94.22434693877551),
//                new Pose(63.55102040816326, 54.73469387755101),
                new Pose(61.559999999999995, 61.559999999999995)
        ));
        p2.setConstantHeadingInterpolation(Math.toRadians(180));

        p3 = new Path(new BezierLine(
                new Pose(61.559999999999995, 59.672000000000004),
                new Pose(20, 59.90399999999998)
        ));
        p3.setConstantHeadingInterpolation(Math.toRadians(180));

        p4 = new Path(new BezierLine(
                new Pose(20, 59.90399999999998),
                new Pose(61.72800000000001, 94.16000000000001)
        ));
        p4.setConstantHeadingInterpolation(Math.toRadians(180));

        p5 = new Path(new BezierLine(
                new Pose(61.72800000000001, 94.16000000000001),
                new Pose(61.583999999999996, 83.47999999999999)
                ));
        p5.setConstantHeadingInterpolation(Math.toRadians(180));

        p6 = new Path(new BezierLine(
                new Pose(61.583999999999996, 83.47999999999999),
                new Pose(20, 83.72800000000005)
                ));
        p6.setConstantHeadingInterpolation(Math.toRadians(180));

        p7 = new Path(new BezierLine(
                new Pose(20, 83.536),
                new Pose(61.693877551020414,94.04800000000002)
        ));
        p7.setConstantHeadingInterpolation(Math.toRadians(180));
    }

    public Command autonomousRoutine(){
        return new SequentialGroupFixed(
//                new NullCommand()
                new FollowPath(p1),
//                new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)),
//                shootThree(),
//                intakeMode(),
                new FollowPath(p2),
                new FollowPath(p3),
                new FollowPath(p4),
//                new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)),
//                shootThree(),
//                intakeMode()
                new FollowPath(p5),
                new FollowPath(p6),
                new FollowPath(p7)
//                new InstantCommand(() -> Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)),
//                shootThree()
        );
    }
    @Override
    public void onStartButtonPressed(){
        buildPaths();
        follower().setStartingPose(new Pose(27.184, 130.041, Math.toRadians(142)));
//        Turret.INSTANCE.setToZero().schedule();
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, 27.184, 130.041, AngleUnit.DEGREES, 142));
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        telemetry.addData("X Position ", follower().getPose().getX());
        telemetry.addData("Y Position ", follower().getPose().getY());
        telemetry.addData("Get Pose", follower().getPose());
        telemetry.addData("X Position ", Pinpoint.INSTANCE.getPosX());
        telemetry.addData("Y Position ", follower().getPose().getY());
        telemetry.addData("Get Pose", follower().getPose());
        Pinpoint.INSTANCE.periodic();
        Turret.INSTANCE.status(telemetry);
        Spindexer.INSTANCE.status(telemetry);
        telemetry.update();

        if(Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.INTAKE) {
            Intake.INSTANCE.on().schedule();
            if(Spindexer.INSTANCE.freePosition()!=-1){
                Spindexer.INSTANCE.setToPosition(Spindexer.Position.values()[Spindexer.INSTANCE.freePosition()]).schedule();
            }
        }

        else if(Spindexer.INSTANCE.getPositionType() == Spindexer.PositionType.SHOOT){
            Spindexer.INSTANCE.setToPosition(Spindexer.INSTANCE.getPosition()).schedule();
            Turret.INSTANCE.followGoalOdometryPositional(Aliance.BLUE,10).schedule();
            velocity = Turret.INSTANCE.distanceToVelocity(follower().getPose().getX(), follower().getPose().getY(), Aliance.BLUE);
            hoodPosition = Turret.INSTANCE.distanceToPosition(follower().getPose().getX(), follower().getPose().getY(), Aliance.BLUE);
        }
        endPose =  follower().getPose().getPose();
        Turret.INSTANCE.setVelocity(velocity).schedule();
        Turret.INSTANCE.setHoodPosition(hoodPosition).schedule();
        Turret.INSTANCE.periodic();
        Spindexer.INSTANCE.periodic();

    }
    public static Pose getEndPose(){
        return endPose;
    }
}

