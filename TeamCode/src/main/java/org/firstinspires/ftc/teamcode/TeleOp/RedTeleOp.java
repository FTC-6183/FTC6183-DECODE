package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Pinpoint;
import org.firstinspires.ftc.teamcode.Utils.Aliance;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Red TeleOp",group = "Red")
public class RedTeleOp extends NextFTCOpMode {
    Robot robot = new Robot(Aliance.RED);
    private double redStartX = 0;
    private double redStartY = 0;
    public RedTeleOp(){
        addComponents(
                new SubsystemComponent(robot, Pinpoint.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed(){
        robot.drive().schedule();
        Pinpoint.INSTANCE.updatePosition(new Pose2D(DistanceUnit.INCH, redStartX, redStartY, AngleUnit.DEGREES, 90));

        Gamepads.gamepad1().circle()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(robot.intake())
                .whenBecomesFalse(robot.intakeOff());



        Gamepads.gamepad1().x()
                .whenBecomesTrue(robot.shootOneSequence());

        Gamepads.gamepad1().y()
                .whenBecomesTrue(robot.shootAllSequence());
    }
    @Override
    public void onUpdate(){
        BindingManager.update();
        robot.periodic();
        telemetry.update();
    }
    @Override
    public void onStop(){
    }

}