package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.utility.NullCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToPosition;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.CRServoEx;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class Turret implements Subsystem {
    private Aliance a;
    public static double angleGlobal = 0;
    public static PIDCoefficients shooterCoefficients = new PIDCoefficients(0.00001,0,0);
    public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.012,0,0);

    public static BasicFeedforwardParameters shooterff = new BasicFeedforwardParameters(0.00055, 0, 0.03);

    public static final Turret INSTANCE = new Turret(Aliance.BLUE);

    public double turretOffSet = 191;
    private Turret(Aliance a){
        this.a = a;
    };
    private double lastAngle = 0;
    private MotorEx shooterMotor1 = new MotorEx("shoot1");
    private MotorEx shooterMotor2 = new MotorEx("shoot2");
    private ServoEx hoodServo = new ServoEx("hood");
    private CRServoEx turret = new CRServoEx("turret");
    private AnalogInput encoder;

    public static final double ANGLE_TO_POSITION = (double) 1 /360;
    public static final int MAX_CW_SERVO = 1;
    public static final double MAX_CCW_SERVO = (double)(360/355) - 1 ;

    public static final double threshold = 200;

    private ControlSystem velocityControl = ControlSystem.builder()
            .velPid(shooterCoefficients)
            .basicFF(shooterff)
            .build();

    private ControlSystem turretControl = ControlSystem.builder()
            .angular(AngleType.DEGREES,
                     feedback -> feedback.posPid(turretCoefficients)
            )
            .build();

    @Override public void initialize(){
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
        turret.setPower(0);
        encoder = ActiveOpMode.hardwareMap().get(AnalogInput.class,"encoderServo");


    }
    //TODO: Add the actual regression equation from testing
    public double distanceToVelocity(){
        double distance = 0;
        if(a == Aliance.RED){
            distance = Limelight.INSTANCE.distanceFromTag(Limelight.RED_GOAL_ID);
        }
        else if(a == Aliance.BLUE){
            distance = Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID);
        }
        return -2000;
    }
    //TODO: Add the actual hood angle/position from testing
    public double distanceToPosition(){
        double distance = 0;
        if(a == Aliance.RED){
            distance = Limelight.INSTANCE.distanceFromTag(Limelight.RED_GOAL_ID);
        }
        else if(a == Aliance.BLUE){
            distance = Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID);
        }
        return 0;
    }
    public double headingToTurretPosition(){
        double measuredAngle = lastAngle;
        if(a == Aliance.RED){
            measuredAngle = Limelight.INSTANCE.angleFromTag(Limelight.RED_GOAL_ID);
        }
        else if(a == Aliance.BLUE){
            measuredAngle = Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID);
        }
        //angle += 180; //The angle output is -180 to 180, this resets the angle mapping to 0 to 360, which is the same range as the servo.
        //angle = Range.clip(angle,5,355);
        //double absolutePosition = angle * ANGLE_TO_POSITION;
        //Left Servo is Clockwise-Set Left to 0, when placing
        //Right Servo is Counterclockwise-Set Right to 1, when placing
        if (measuredAngle != -1) {
            lastAngle = measuredAngle;
        }
        return lastAngle;
    }
    public double headingToTurretPosition(double angle){
        angle += 180; //The angle output is -180 to 180, this resets the angle mapping to 0 to 360, which is the same range as the servo.
        angle = Range.clip(angle,5,355);
        double absolutePosition = angle * ANGLE_TO_POSITION;
        //Left Servo is Clockwise-Set Left to 0, when placing
        //Right Servo is Counterclockwise-Set Right to 1, when placing
        return absolutePosition;
    }

    public double getCurrentAngleFromEncoder() {
        if (encoder == null) return 0;
        return (encoder.getVoltage() / 3.3) * 360;
    }

    public double getAbsoluteAngleFromEncoder(){
        double angle = getCurrentAngleFromEncoder()-turretOffSet;
        double wrappedAngle = ((angle+180)%360+360)%360-180;
        return wrappedAngle;
    }

    public Command followAprilTag(){
        return new RunToPosition(turretControl,getAbsoluteAngleFromEncoder()+headingToTurretPosition());

    }
    public Command setHoodPosition(double position){
        return new SetPosition(hoodServo, position).requires(this);
    }

    public Command setVelocity(double velocity){
        return new RunToVelocity(velocityControl,velocity).requires(this);
    }


    public double getTurretPower(){
          return turret.getPower();
    }


    public double turretPIDCorrection(){
        return turretControl.calculate(new KineticState(getAbsoluteAngleFromEncoder()));
    }




    public double getPosition(){
        return hoodServo.getPosition();
    }


    public final Command off = new NullCommand();//RunToVelocity(velocityControl, 0).requires(this);
    public final Command on = new NullCommand();//new RunToVelocity(velocityControl,distanceToVelocity()).requires(this);
    public final Command testOnOneWay = new SetPower(shooterMotor1,1).and(new SetPower(shooterMotor2,-1));
    public final Command testOtherWay = new SetPower(shooterMotor1,-1).and(new SetPower(shooterMotor2,1));
    public final Command testOff = new SetPower(shooterMotor1,0).and(new SetPower(shooterMotor2,0));
    public final Command waitToShoot = new WaitUntil(()->(Math.abs(Turret.INSTANCE.getVelocityOne())-Math.abs(Turret.INSTANCE.distanceToVelocity())< Turret.threshold));
    public final Command testServoOn = new SetPower(turret,0.5);
    public final Command testServoOff = new SetPower(turret,0);


    public double getVelocityOne(){return shooterMotor1.getVelocity();}
    public double getVelocityTwo(){return shooterMotor2.getVelocity();}

    /*
    public String servoPosition(){
        return " Right Servo Position is " + rightTurretServo.getPosition();
    }

     */

    @Override
    public void periodic(){
        shooterMotor1.setPower(velocityControl.calculate(shooterMotor1.getState()));
        shooterMotor2.setPower(-velocityControl.calculate(shooterMotor1.getState()));
        turret.setPower(turretControl.calculate(new KineticState(getAbsoluteAngleFromEncoder())));

    }

}
