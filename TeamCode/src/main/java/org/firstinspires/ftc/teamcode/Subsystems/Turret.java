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
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class Turret implements Subsystem {
    private Aliance a;
    public static double angleGlobal = 0;
    public static double power = 1;
    public double setpoint = 0;

    public double previousHood = 0.1;
    public static double angleOffset = 0;
    public static double error = 0;
    public static double maxPower = 1;
    public static PIDCoefficients shooterCoefficients = new PIDCoefficients(0.00005,0,0);
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
    private Interpolator shooter = new Interpolator();
    private Interpolator hood = new Interpolator();


    private MotorEx shooterMotor1 = new MotorEx("shoot1").floatMode();
    private MotorEx shooterMotor2 = new MotorEx("shoot2").floatMode();
    private ServoEx hoodServo = new ServoEx("hood");
    //private CRServoEx turret1 = new CRServoEx("turret1");
    //private CRServoEx turret2 = new CRServoEx("turret2");
    private ServoEx turret1 = new ServoEx("turret1");
    private ServoEx turret2 = new ServoEx("turret2" );


    private AnalogInput encoder;

    public static final double ANGLE_TO_POSITION = (double) 1 /360;
    public static final int MAX_CW_SERVO = 1;
    public static final double MAX_CCW_SERVO = (double)(360/355) - 1 ;

    public static double threshold = 1;
    //public static double RED_GOAL_X = 129;
    //public static double RED_GOAL_Y = 129;
    //public static double BLUE_GOAL_X = 28;
    //public static double BLUE_GOAL_Y = 129;
    public static double RED_GOAL_X = 144;
    public static double RED_GOAL_Y = 144;
    public static double BLUE_GOAL_X = 0;
    public static double BLUE_GOAL_Y = 144;

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
        shooter.addPoint(72.6291,72.2894,1100);
        shooter.addPoint(57,82.67,1100);
        shooter.addPoint(42,99,1000);
        shooter.addPoint(34,109,900);
        shooter.addPoint(71,86,1000);
        shooter.addPoint(73,107,1000);
        shooter.addPoint(79,126,1000);
        shooter.addPoint(93,80,1100);
        shooter.addPoint(100	,100,1200);
        shooter.addPoint(112,112,1300);
        shooter.addPoint(60	,109,1100);
        shooter.addPoint(55	,130,1000);
        shooter.addPoint(90	,110,1200);
        shooter.addPoint(100,127,1200);
        hood.addPoint(72.6291,72.2894,0.1);
        hood.addPoint(57,82.67,0.1);
        hood.addPoint(42,99,0.25);
        hood.addPoint(34,109,0.4);
        hood.addPoint(71,86,0.1);
        hood.addPoint(73,107,0.1);
        hood.addPoint(79,126,0.1);
        hood.addPoint(93,80,0.1);
        hood.addPoint(100,100,0.2);
        hood.addPoint(112,112,0.1);
        hood.addPoint(60,109,0.1);
        hood.addPoint(55,130,0.2);
        hood.addPoint(90,110,0.1);
        hood.addPoint(100,127,0.1);
    }
    //TODO: Add the actual regression equation from testing
    public double distanceToVelocity(double x, double y){
        return shooter.get(x,y);
    }
    public static double quartic_polynomial(double x, double a, double b, double c, double d, double e) {
        return (((a * x + b) * x + c) * x + d) * x + e;
    }
    //TODO: Add the actual hood angle/position from testing
    public double distanceToPosition(double x, double y){
        return hood.get(x,y);
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
        double turretAngle = targetFieldAngle + 90 - robotHeading + angleOffset;
        turretAngle = ((turretAngle % 360) + 360) % 360;
        turretAngleSet = turretAngle;
        double position = angleToPosition(turretAngle);
        position = Range.clip(position, 0, 1);
        turretPowerSet = position;
        return new SetPosition(turret1,position).and(new SetPosition(turret2,position));
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

    public Command setTurretPosition(double position) {
    return new SetPosition(turret1,position).and(new SetPosition(turret2,position));
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


    public double getVelocity(){return shooterMotor1.getVelocity();}
    public double getVelocityTwo(){return shooterMotor2.getVelocity();}


    public double turretOnePosition(){
        return turret1.getPosition();
    }

    public double turretTwoPosition(){
        return turret2.getPosition();
    }

    public double angleToPosition(double angle) {
        angle = Math.max(0, Math.min(360, angle));
        /*
        double zeroPosition = 0.21;
        double ninetyPosition = 0.5;
        double oneEightyPosition = 0.79;
        */
        double tickPerDegree = (0.35) / 90;
        double zeroPosition = 0.83;
        double ninetyPosition = 0.5;
        double oneEightyPosition = 0.79;
        if (angle <= positionToAngle(0)){
            Drivetrain.INSTANCE.setTurnSpeed(1);
            return 0.83 - (tickPerDegree * angle);
        }
        //The turret CANNOT reach the range of 204.827586207(1) to 254.482758621(0)
        //This rounds to the nearest angle it can reach
        else if(angle >= (positionToAngle(0)) && angle <= ((positionToAngle(0))+(positionToAngle(1) - positionToAngle(0))/2)){
            Drivetrain.INSTANCE.setTurnSpeed(0.5);
            return 0;
        }
        else if(angle <= positionToAngle(1)){
            Drivetrain.INSTANCE.setTurnSpeed(0.5);
            return 1;
        }
        Drivetrain.INSTANCE.setTurnSpeed(1);
        return tickPerDegree * (angle - positionToAngle(1)) + 0.83;
    }
    public double positionToAngle(double position) {
        double degreesPerTick = 90/(0.35);
        return (degreesPerTick*(0.83-position)% 360 + 360) % 360;
    }


    @Override
    public void periodic(){
        //shooterMotor1.setPower(velocityControl.calculate(shooterMotor2.getState()));
        //shooterMotor2.setPower(-velocityControl.calculate(shooterMotor2.getState()));
        if(turretVelocity < getVelocity()){
            power = 0;
        } else if (turretVelocity > getVelocity()) {
            power = 1;
        }
        shooterMotor1.setPower(power);
        shooterMotor2.setPower(-power);

        //shooterMotor1.setPower(staticPosition);
        //shooterMotor2.setPower(-staticPosition);
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


