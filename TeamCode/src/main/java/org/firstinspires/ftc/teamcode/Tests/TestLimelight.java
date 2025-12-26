package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
@TeleOp(name = "TestLimelight")
public class TestLimelight extends NextFTCOpMode {
    public TestLimelight(){
        addComponents(
                new SubsystemComponent(Limelight.INSTANCE,Drivetrain.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    @Override
    public void onStartButtonPressed(){
        Drivetrain.INSTANCE.startFieldDrive().schedule();
    }
    @Override
    public void onUpdate(){

        BindingManager.update();
        telemetry.addData("Distance From Blue Tag", Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Distance From Red Tag", Limelight.INSTANCE.distanceFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Angle From Blue Tag", Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID));
        telemetry.addData("Angle From Red Tag", Limelight.INSTANCE.angleFromTag(Limelight.RED_GOAL_ID));
        telemetry.addData("Pattern of Obelisk",
                Limelight.INSTANCE.patternFromObelisk() == Limelight.GPP_PATTERN_ID ? "GPP":
                Limelight.INSTANCE.patternFromObelisk() == Limelight.PGP_PATTERN_ID ? "PGP":
                Limelight.INSTANCE.patternFromObelisk() == Limelight.PPG_PATTERN_ID ? "PPG":
                "Can't be found"
        );
        telemetry.update();
    }
    @Override
    public void onStop(){
    }
}

