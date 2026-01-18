package org.firstinspires.ftc.teamcode.Subsystems;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.I2cDevice;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Pinpoint implements Subsystem {
    public static final Pinpoint INSTANCE = new Pinpoint();
    private Pinpoint() {}
    GoBildaPinpointDriver pinpoint;

    @Override
    public void initialize(){
      pinpoint = ActiveOpMode.hardwareMap().get(GoBildaPinpointDriver.class,"pinpoint");
      pinpoint.setOffsets(4.5,-7.125, DistanceUnit.INCH);
      pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
      pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    public void updatePosition(Pose2D position){
        pinpoint.setPosition(position);
    }

    public double getPosX(){
        return pinpoint.getPosX(DistanceUnit.INCH);
    }
    public double getPosY(){
        return pinpoint.getPosY(DistanceUnit.INCH);
    }
    public double getHeading(){
        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    @Override
    public void periodic() {
        pinpoint.update();
    }
}
