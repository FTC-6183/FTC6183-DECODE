package org.firstinspires.ftc.teamcode.Subsystems;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Utils.Interpolator;
import org.firstinspires.ftc.teamcode.Vision.Limelight;


import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.AngleType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.WaitUntil;
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
    //public static double power = 0;
    public double previousHood = 0.1;
    public static double error = 0;
    public static double staticPosition = 0;

    public static double maxPower = 1;
    public static PIDCoefficients shooterCoefficients = new PIDCoefficients(0.00001,0,0);
    public static PIDCoefficients turretCoefficients = new PIDCoefficients(0.012,0,0);

    public static BasicFeedforwardParameters shooterff = new BasicFeedforwardParameters(0.00055, 0, 0.03);

    public static final Turret INSTANCE = new Turret(Aliance.BLUE);
    public static double turretVelocity = 0;

    private double turretAngleSet = 0;
    private double turretPowerSet = 0;


    public double turretOffSet = -65;
    private Turret(Aliance a){
        this.a = a;
    };
    private double lastAngle = 0;
    private MotorEx shooterMotor1 = new MotorEx("shoot1").floatMode();
    private MotorEx shooterMotor2 = new MotorEx("shoot2").floatMode();
    private ServoEx hoodServo = new ServoEx("hood");
    //private CRServoEx turret1 = new CRServoEx("turret1");
    //private CRServoEx turret2 = new CRServoEx("turret2");
    private ServoEx turret1 = new ServoEx("turret1");
    private ServoEx turret2 = new ServoEx("turret1");


    private AnalogInput encoder;

    public static final double ANGLE_TO_POSITION = (double) 1 /360;
    public static final int MAX_CW_SERVO = 1;
    public static final double MAX_CCW_SERVO = (double)(360/355) - 1 ;

    public static double threshold = 1;
    public static double RED_GOAL_X = 129;
    public static double RED_GOAL_Y = 129;
    public static double BLUE_GOAL_X = 28;
    public static double BLUE_GOAL_Y = 120;

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
        turret1.setPosition(0.52);
        turret2.setPosition(0.52);
        //turret1.setPower(0);
        //turret2.setPower(0);
        encoder = ActiveOpMode.hardwareMap().get(AnalogInput.class,"encoderServo");
        Interpolator shooter = new Interpolator();
        Interpolator hood = new Interpolator();

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
        return distance*6.5118+867.97272;
    }
    public static double quartic_polynomial(double x, double a, double b, double c, double d, double e) {
        return (((a * x + b) * x + c) * x + d) * x + e;
    }
    //TODO: Add the actual hood angle/position from testing
    public double distanceToPosition(){
        double distance = Limelight.INSTANCE.distanceFromTag(Limelight.BLUE_GOAL_ID);
        double currentHood = quartic_polynomial(distance,(-4.39 * Math.pow(10,-7)),0.0000535455,-0.00149178, -0.0154999,0.817171);
        if(distance == 0){
            return previousHood;
        }
        previousHood = currentHood;
        return currentHood;
    }
    public double headingToTurretPositionLL(){
        double measuredAngle = lastAngle;
        if(a == Aliance.RED){
            measuredAngle = Limelight.INSTANCE.angleFromTag(Limelight.RED_GOAL_ID);
        }
        else if(a == Aliance.BLUE){
            measuredAngle = Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID);
        }

        if (measuredAngle != -1) {
            lastAngle = measuredAngle;
        }
        return lastAngle;
    }
    public double headingToTurretPositionPinpoint(){
        // Get your robot's current position from odometry
        Pose2D robotPose = new Pose2D(DistanceUnit.INCH, Pinpoint.INSTANCE.getPosX(), Pinpoint.INSTANCE.getPosY(), AngleUnit.DEGREES, Pinpoint.INSTANCE.getHeading());
        double goalX = BLUE_GOAL_X;
        double goalY = BLUE_GOAL_Y;

        double deltaX = goalX - robotPose.getX(DistanceUnit.INCH);
        double deltaY = goalY - robotPose.getY(DistanceUnit.INCH);

        double angleToGoal = Math.atan2(deltaY, deltaX);
        angleToGoal = Math.toDegrees(angleToGoal);
        //angleToGoal = (angleToGoal + 360) % 360;
        return angleToGoal;
    }
    /*
    public Command followGoalPPLL(){
        double goalX, goalY;
        double limelight_distance = -1;
        if(a == Aliance.BLUE){
            limelight_distance = Limelight.INSTANCE.angleFromTag(Limelight.BLUE_GOAL_ID);
            goalX = BLUE_GOAL_X;
            goalY = BLUE_GOAL_Y;
        }
        else if(a == Aliance.RED){
            limelight_distance = Limelight.INSTANCE.angleFromTag(Limelight.RED_GOAL_ID);
            goalX = RED_GOAL_X;
            goalY = RED_GOAL_Y;
        }
        if(limelight_distance != -1){
            return followAprilTag();
        }
        return followGoalOdometryPositional();
    }
    */

    /*
    public Command followGoalOdometry(){
        double robotHeading = ((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360;
        double targetFieldAngle = headingToTurretPositionPinpoint();
        double turretAngle = targetFieldAngle + 90 - robotHeading;
        turretAngle = ((turretAngle % 360) + 360) % 360;
        turretAngleSet = turretAngle;

        double targetFieldAngle = headingToTurretPositionPinpoint();
        double turretAbsoluteAngle = getWrappedAngleFromEncoder();
        double turretRotationNeeded = targetFieldAngle - turretAbsoluteAngle;
        turretRotationNeeded = ((turretRotationNeeded + 180) % 360 + 360) % 360 - 180;
        error = turretRotationNeeded;
        return new RunToPosition(turretControl, getWrappedAngleFromEncoder() + turretRotationNeeded);

    }

        public Command followGoalOdometry(){
        double robotHeading = ((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360;
        double targetFieldAngle = headingToTurretPositionPinpoint();
        double turretAngle = targetFieldAngle + 90 - robotHeading;
        turretAngle = ((turretAngle % 360) + 360) % 360;
        turretAngleSet = turretAngle;
        return new RunToPosition(turretControl, turretAngle);
    */



    /*
  public Command followGoalOdometryPositional(){
      double robotHeading = ((Pinpoint.INSTANCE.getHeading() + 360) % 360);
      double position = angleToPosition(headingToTurretPositionPinpoint() - robotHeading);
      position = Range.clip(position,0.01,0.99);
      return new SetPosition(turret1,position).requires(this).and(new SetPosition(turret2,position)).requires(this);
  }

  public Command followGoalOdometryPositional() {
      double robotHeading = Pinpoint.INSTANCE.getHeading();
      double targetFieldAngle = headingToTurretPositionPinpoint();
      // Get robot-relative angle and wrap to -180 to 180
      double turretAngle = targetFieldAngle - robotHeading;
      turretAngle = ((turretAngle + 180) % 360 + 360) % 360 - 180;
      // Now wrap to 0-360 for angleToPosition
      turretAngle = ((turretAngle % 360) + 360) % 360;
      double position = angleToPosition(turretAngle);
      position = Range.clip(position, 0.01, 0.99);
      return new SetPosition(turret1,position).requires(this).and(new SetPosition(turret2,position)).requires(this);

  }
*/
    public Command followGoalOdometryPositional(){
        double robotHeading = ((Pinpoint.INSTANCE.getHeading() % 360) + 360) % 360;
        double targetFieldAngle = headingToTurretPositionPinpoint();
        double turretAngle = targetFieldAngle + 90 - robotHeading;
        turretAngle = ((turretAngle % 360) + 360) % 360;
        turretAngleSet = turretAngle;
        double position = angleToPosition(turretAngle);
        position = Range.clip(position, 0, 1);
        turretPowerSet = position;
        return new SetPosition(turret1,position).and(new SetPosition(turret2,position));
        //return new NullCommand();

    }

    public double getTurretAngleSet(){
        return turretAngleSet;
    }

    public double getTurretPowerSet(){
        return turretPowerSet;
    }

    public double getError(){
        return error;
    }
    /*
    public double headingToTurretPositionLL(double angle){
        angle += 180; //The angle output is -180 to 180, this resets the angle mapping to 0 to 360, which is the same range as the servo.
        angle = Range.clip(angle,5,355);
        double absolutePosition = angle * ANGLE_TO_POSITION;
        //Left Servo is Clockwise-Set Left to 0, when placing
        //Right Servo is Counterclockwise-Set Right to 1, when placing
        return absolutePosition;
    }
     */

    public double getNoOffsetAngleFromEncoder() {
        if (encoder == null) return 0;
        return (encoder.getVoltage() / 3.3) * 360;
    }

    public double getMaxVoltageFromEncoder(){
        return encoder.getMaxVoltage();
    }
    public double getVoltageFromEncoder(){
        return encoder.getVoltage();
    }

    public double getWrappedAngleFromEncoder(){
        double angle = getNoOffsetAngleFromEncoder()-turretOffSet;
        double wrappedAngle = ((angle+180)%360+360)%360-180;
        return wrappedAngle;
    }

    public double getNonWrappedAngleFromEncoder(){
        return  getNoOffsetAngleFromEncoder()-turretOffSet;
    }

/*
    public Command followAprilTag(){
        return new SetPosition(turret1,angleToPosition(turretOnePosition()+headingToTurretPositionLL())).and(new SetPosition(turret2,angleToPosition(turretOnePosition()+headingToTurretPositionLL())));
    }

 */


    public Command setHoodPosition(double position){
        return new SetPosition(hoodServo, position).requires(this);
    }

    public Command setVelocity(double velocity){
        turretVelocity = velocity;
        return new RunToVelocity(velocityControl,velocity).requires(this);
    }

/*
    public double getTurretPower(){
          return turret1.getPower();
    }

 */


    public double turretPIDCorrectionWrapped(){
        return turretControl.calculate(new KineticState(getWrappedAngleFromEncoder()));
    }
    public double turretPIDCorrectionNonWrapped(){
        return turretControl.calculate(new KineticState(getNonWrappedAngleFromEncoder()));
    }



    public double getPosition(){
        return hoodServo.getPosition();
    }


    public final Command off = new RunToVelocity(velocityControl, 0).requires(this);
    public Command on(){
        return new RunToVelocity(velocityControl,-1000,threshold).requires(this);
    }
    public final Command testOnOneWay = new SetPower(shooterMotor1,1).and(new SetPower(shooterMotor2,-1));
    public final Command testOtherWay = new SetPower(shooterMotor1,-1).and(new SetPower(shooterMotor2,1));
    public final Command testOff = new SetPower(shooterMotor1,0).and(new SetPower(shooterMotor2,0));
    public Command waitToShoot= new WaitUntil(() ->
            Math.abs(Turret.INSTANCE.getVelocityTwo() - turretVelocity)
                    < Turret.threshold
    );
    //public final Command testServoOn = new SetPower(turret1,0.5);
    //public final Command testServoOff = new SetPower(turret1,0);


    public double getVelocityOne(){return shooterMotor1.getVelocity();}
    public double getVelocityTwo(){return shooterMotor2.getVelocity();}


    public double turretOnePosition(){
        return turret1.getPosition();
    }

    public double turretTwoPosition(){
        return turret2.getPosition();
    }

    public double positionToAngle(double input){
        return ((288 + input * 316.286) % 360);

    }

    public double angleToPosition(double input){
        if (input >= 288){
            return ((input - 288) / 316.286);
        }
        else {
            return ((input + 72) / 316.286);
        }
    }

    @Override
    public void periodic(){
        shooterMotor1.setPower(velocityControl.calculate(shooterMotor1.getState()));
        shooterMotor2.setPower(-velocityControl.calculate(shooterMotor1.getState()));
        /*
        double power = turretControl.calculate(new KineticState(getWrappedAngleFromEncoder()));
        if(Math.abs(power) > maxPower){
            power = maxPower * Math.signum(power);
        }
        turret1.setPower(power);
        turret2.setPower(power);

         */

    }
    }


