package org.firstinspires.ftc.teamcode.Vision;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class Limelight implements Subsystem {
    public static final Limelight INSTANCE = new Limelight();
    private Limelight(){};
    Limelight3A limelight;
    public static final int BLUE_GOAL_ID = 20;
    public static final int RED_GOAL_ID = 24;
    public static final int GPP_PATTERN_ID = 21;
    public static final int PGP_PATTERN_ID = 22;
    public static final int PPG_PATTERN_ID = 23;

    @Override public void initialize(){
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class,"limelight");
        limelight.setPollRateHz(100);
    }

    public Command start(){
        return new InstantCommand(()-> limelight.start());
    }
    public Command stop(){
        return new InstantCommand(()-> limelight.stop());
    }
    public double angleFromTag(double tagID){
        limelight.start();
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();
        if(r.isEmpty()) return -1;
        LLResultTypes.FiducialResult target = null;
        for(LLResultTypes.FiducialResult fiducial : r){
            int id = fiducial.getFiducialId();
            if(tagID == id){
                target = fiducial;
                break;
            }
        }
        if(target != null){
            return target.getTargetXDegrees();
        }
        return -1;
    }
    public double distanceFromTag(double tagID){
        limelight.start();
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();
        if(r.isEmpty()) return 0;
        LLResultTypes.FiducialResult target = null;
        for(LLResultTypes.FiducialResult fiducial : r){
            int id = fiducial.getFiducialId();
            if(tagID == id){
                target = fiducial;
                break;
            }
        }

        if(target != null){
            double x  = (target.getCameraPoseTargetSpace().getPosition().x/DistanceUnit.mPerInch)+16;
            double z  = (target.getCameraPoseTargetSpace().getPosition().z/DistanceUnit.mPerInch)+16;

            Vector distance = new Vector();
            distance.setOrthogonalComponents(x,z);
            return distance.getMagnitude();
        }
        return 0;
    }
    public double patternFromObelisk(){
        limelight.start();
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();
        if(r.isEmpty()) return -1;
        for(LLResultTypes.FiducialResult fiducial : r){
            int id = fiducial.getFiducialId();
            if(id==GPP_PATTERN_ID||id==PGP_PATTERN_ID||id==PPG_PATTERN_ID){
                return id;
            }
        }
        return -1;
    }
    //TODO: Implement Kalman Filter
    public Pose relocalizeFromCameraPedro() {
        limelight.start();
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();
        if (r.isEmpty()) return null;
        Pose3D redGoal = null;
        Pose3D blueGoal = null;
        Pose2D blueGoalPose = null;
        Pose2D redGoalPose = null;
        for (LLResultTypes.FiducialResult fiducial : r) {
            int id = fiducial.getFiducialId();
            if(id == RED_GOAL_ID){
                redGoal = fiducial.getRobotPoseFieldSpace();
            }
            if(id == BLUE_GOAL_ID){
                blueGoal = fiducial.getRobotPoseFieldSpace();
            }
        }
        //Basic Implementation, Will fix later with Kalman Filter and Sensor integration
        redGoalPose = (redGoal != null) ? new Pose2D(redGoal.getPosition().unit, redGoal.getPosition().x,redGoal.getPosition().y, AngleUnit.DEGREES,redGoal.getOrientation().getYaw()):null;
        blueGoalPose = (blueGoal != null) ?  new Pose2D(blueGoal.getPosition().unit, blueGoal.getPosition().x,blueGoal.getPosition().y,AngleUnit.DEGREES, blueGoal.getOrientation().getYaw()):null;
        if(redGoalPose == null && blueGoalPose != null){
            return PoseConverter.pose2DToPose(blueGoalPose, InvertedFTCCoordinates.INSTANCE);
        }
        if(redGoalPose != null && blueGoalPose == null){
            return PoseConverter.pose2DToPose(redGoalPose, InvertedFTCCoordinates.INSTANCE);
        }
        if(distanceFromTag(RED_GOAL_ID)>=distanceFromTag(BLUE_GOAL_ID) && redGoalPose != null){
            return PoseConverter.pose2DToPose(redGoalPose, InvertedFTCCoordinates.INSTANCE);
        }
        if(distanceFromTag(BLUE_GOAL_ID)>distanceFromTag(RED_GOAL_ID) && blueGoalPose == null) {
            return PoseConverter.pose2DToPose(blueGoalPose, InvertedFTCCoordinates.INSTANCE);
        }
        return null;
    }

    public Pose2D relocalizeFromCameraRegular() {
        limelight.start();
        List<LLResultTypes.FiducialResult> r = limelight.getLatestResult().getFiducialResults();
        if (r.isEmpty()) return null;
        Pose3D redGoal = null;
        Pose3D blueGoal = null;
        Pose2D blueGoalPose = null;
        Pose2D redGoalPose = null;
        for (LLResultTypes.FiducialResult fiducial : r) {
            int id = fiducial.getFiducialId();
            if(id == RED_GOAL_ID){
                redGoal = fiducial.getRobotPoseFieldSpace();
            }
            if(id == BLUE_GOAL_ID){
                blueGoal = fiducial.getRobotPoseFieldSpace();
            }
        }
        //Basic Implementation, Will fix later with Kalman Filter and Sensor integration
        redGoalPose = (redGoal != null) ? new Pose2D(DistanceUnit.INCH, redGoal.getPosition().x/DistanceUnit.mPerInch,redGoal.getPosition().y/DistanceUnit.mPerInch, AngleUnit.DEGREES,redGoal.getOrientation().getYaw()):null;
        blueGoalPose = (blueGoal != null) ?  new Pose2D(DistanceUnit.INCH, blueGoal.getPosition().x/DistanceUnit.mPerInch,blueGoal.getPosition().y/DistanceUnit.mPerInch,AngleUnit.DEGREES, blueGoal.getOrientation().getYaw()):null;

        if(redGoalPose == null && blueGoalPose != null){
            return redGoalPose;
        }
        if(redGoalPose != null && blueGoalPose == null){
            return blueGoalPose;
        }
        if(distanceFromTag(RED_GOAL_ID)>=distanceFromTag(BLUE_GOAL_ID) && redGoalPose != null){
            return redGoalPose;
        }
        if(distanceFromTag(BLUE_GOAL_ID)>distanceFromTag(RED_GOAL_ID) && blueGoalPose == null) {
            return blueGoalPose;
        }
        return null;
    }
}
