package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.Subsystems.Spindexer.Position.POSITION_ONE;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Pinpoint;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Config
@Configurable
@TeleOp(name = "Blue TeleOp", group = "Blue")
public class BlueTeleOp extends NextFTCOpMode {
    Robot robot = new Robot(Aliance.BLUE);
    private double blueStartX = 0;
    private double blueStartY = 0;
    public BlueTeleOp(){
        addComponents(
                new SubsystemComponent(robot, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed(){
        robot.drive().schedule();
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, blueStartX, blueStartY, AngleUnit.DEGREES, 90));
        int pattern = Limelight.INSTANCE.patternFromObelisk();

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(robot.intakeMode())
                .whenBecomesFalse(robot.shootMode());

        Gamepads.gamepad1().cross()
                .whenBecomesTrue(robot.transferFlick());

        Gamepads.gamepad1().y()
                .whenBecomesTrue(robot.shootAllSequence());

        Gamepads.gamepad1().leftBumper()
                .whenBecomesTrue(Spindexer.Position::next);

        Gamepads.gamepad1().rightBumper()
                .whenBecomesTrue(Spindexer.Position::previous);

        Gamepads.gamepad1().rightTrigger().greaterThan(0.3)
                .whenBecomesTrue(robot.shootAllSequence());

        Gamepads.gamepad1().leftTrigger().greaterThan(0.3)
                .whenBecomesTrue(robot.shootMotifSequence());
    }
    @Override
    public void onUpdate(){
        BindingManager.update();
        robot.periodic();
        telemetry.addData("Mode", Spindexer.INSTANCE.getPositionType());
        telemetry.addData("Current Color", Spindexer.INSTANCE.readCurrentColor());
        telemetry.addData("Ball at Position One", Spindexer.INSTANCE.getBallAtPosition()[0]);
        telemetry.addData("Ball at Position Two", Spindexer.INSTANCE.getBallAtPosition()[1]);
        telemetry.addData("Ball at Position Three", Spindexer.INSTANCE.getBallAtPosition()[2]);
        telemetry.addData("Spindexer Position", Spindexer.INSTANCE.getPosition());

        telemetry.addData("x:", Pinpoint.INSTANCE.getPosX());
        telemetry.addData("y:", Pinpoint.INSTANCE.getPosY());

        telemetry.update();
    }
    @Override
    public void onStop(){
    }
}