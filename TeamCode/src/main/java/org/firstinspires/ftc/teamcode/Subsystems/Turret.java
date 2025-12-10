package org.firstinspires.ftc.teamcode.Subsystems;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class Turret implements Subsystem {
    private Aliance a;
    public static final Turret INSTANCE = new Turret(Robot.aliance);
    private Turret(Aliance a){
        this.a = a;
    };
    private MotorEx shooterMotor1 = new MotorEx("shoot1");
    private MotorEx shooterMotor2 = new MotorEx("shoot2");
    private ServoEx hoodServo = new ServoEx("hood");
    private ServoEx leftTurretServo = new ServoEx("leftTurret");
    private ServoEx rightTurretServo = new ServoEx("rightTurret");
    @Override public void initialize(){
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);

    }
    public static final int ANGLE_TO_POSITION = (1/360);
    public static final int MAX_CW_SERVO = 1;
    public static final double MAX_CCW_SERVO = (double)(360/355) - 1 ;
    public static PIDCoefficients coefficients = new PIDCoefficients(0.00001,0,0);
    public static BasicFeedforwardParameters ff = new BasicFeedforwardParameters(0.00048, 0.02, 0.03);
    public static final double threshold = 200;

    private ControlSystem velocityControl = ControlSystem.builder()
            .velPid(coefficients)
            .basicFF(ff)
            .build();
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
        double angle = 0;
        if(a == Aliance.RED){
            angle = Limelight.INSTANCE.angleFromTag(Limelight.RED_GOAL_ID);
        }
        else if(a == Aliance.BLUE){
            angle = Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID);
        }
        angle += 180; //The angle output is -180 to 180, this resets the angle mapping to 0 to 360, which is the same range as the servo.
        angle = Range.clip(angle,0,355);
        double absolutePosition = angle * ANGLE_TO_POSITION;
        //Left Servo is Clockwise-Set Left to 0, when placing
        //Right Servo is Counterclockwise-Set Right to 1, when placing
        return absolutePosition;
    }
    public Command setVelocity(double velocity){
        return new RunToVelocity(velocityControl,velocity).requires(this);
    }
    public Command setPosition(double position){
        return new SetPosition(hoodServo, position).requires(this);
    }
    public double getPosition(){
        return hoodServo.getPosition();
    }
    public final Command off = new RunToVelocity(velocityControl, 0).requires(this);
    public final Command on = new RunToVelocity(velocityControl,distanceToVelocity()).requires(this);
    public final Command waitToShoot = new WaitUntil(()->(Math.abs(Turret.INSTANCE.getVelocity())-Math.abs(Turret.INSTANCE.distanceToVelocity())< Turret.threshold));

    public double getVelocity(){return shooterMotor1.getVelocity();}
    @Override
    public void periodic(){
        shooterMotor1.setPower(velocityControl.calculate(shooterMotor1.getState()));
        shooterMotor2.setPower(-velocityControl.calculate(shooterMotor2.getState()));
        leftTurretServo.setPosition(headingToTurretPosition());
        rightTurretServo.setPosition(headingToTurretPosition());
        hoodServo.setPosition(distanceToPosition());
        ActiveOpMode.telemetry().addData("Velocity Output Shooter Up",velocityControl.calculate(shooterMotor1.getState()));
        ActiveOpMode.telemetry().addData("Velocity Output Shooter Down",velocityControl.calculate(shooterMotor2.getState()));
    }

}
