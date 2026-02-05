package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Subsystems.Turret.turretVelocity;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Spindexer;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Utils.Aliance;
import org.firstinspires.ftc.teamcode.Vision.Limelight;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.conditionals.IfElseCommand;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.commands.utility.PerpetualCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.subsystems.SubsystemGroup;

public class Robot extends SubsystemGroup {
    public static Aliance aliance;
    public int pattern = -1;
    public Robot(Aliance a){
        super(
                Drivetrain.INSTANCE,
                Intake.INSTANCE,
                Spindexer.INSTANCE,
                Transfer.INSTANCE,
                Turret.INSTANCE,
                Limelight.INSTANCE
        );
        aliance = a;
    }
    /*
    public Command intakeOn(){
        Command intakeSequence = new ParallelGroup(
                Intake.INSTANCE.on(),
                new PerpetualCommand(Spindexer.INSTANCE.setToFreePosition())
        );
        return intakeSequence;
    }
    */
    public Command intakeOn(){
        return Intake.INSTANCE.on();
    }
    public Command intakeOff(){
        return Intake.INSTANCE.idle();
    }

    public Command transferFlick(){
        return new SequentialGroup(Transfer.INSTANCE.transferUp(),
                new Delay(0.5),
                Transfer.INSTANCE.transferDown(),
                new InstantCommand(()->Spindexer.INSTANCE.setColor(Spindexer.INSTANCE.getPosition(), Spindexer.DetectedColor.EMPTY))
        );
    }

    public Command intakeMode(){
        return new SequentialGroup(
                new InstantCommand(()->Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.INTAKE)), Intake.INSTANCE.on());
    }

    public Command shootMode(){
        return new SequentialGroup(
                new InstantCommand(()->Spindexer.INSTANCE.setPositionType(Spindexer.PositionType.SHOOT)), Intake.INSTANCE.idle());

    }
    public Command shootOneSequence(){
        Command shootSequence = new SequentialGroup(
                intakeOff(),
                Turret.INSTANCE.on(),
                //Spindexer.INSTANCE.setToFilledPosition(),
                Turret.INSTANCE.waitToShoot,
                Transfer.INSTANCE.transferUp(),
                Transfer.INSTANCE.transferDown(),
                //Spindexer.INSTANCE.setToFilledPosition(),
                Turret.INSTANCE.off
                );
        return shootSequence;
    }
    public static Command runUntilEmpty(){
        return new LambdaCommand()
                .setUpdate(new SequentialGroup(
                        //Spindexer.INSTANCE.setToFilledPosition(),
                        Turret.INSTANCE.waitToShoot,
                        Transfer.INSTANCE.transferUp(),
                        Transfer.INSTANCE.transferDown()))
                .setIsDone(()->Spindexer.INSTANCE.empty);
    }

    public static Command runMotif(int pattern){
        return new LambdaCommand()
                .setUpdate(new SequentialGroup(
                        //Spindexer.INSTANCE.setToFilledPosition(pattern),
                        Turret.INSTANCE.waitToShoot,
                        Transfer.INSTANCE.transferUp(),
                        Transfer.INSTANCE.transferDown()))
                .setIsDone(()->Spindexer.INSTANCE.empty);
    }

    public Command shootAllSequence(){
        Command fullShootSequence = new SequentialGroup(
                intakeOff(),
                Turret.INSTANCE.setVelocity(turretVelocity),
                runUntilEmpty(),
                Turret.INSTANCE.off
        );
        return fullShootSequence;
    }
    public Command shootMotifSequence(){
        Command fullShootSequence = new SequentialGroup(
                intakeOff(),
                Turret.INSTANCE.on(),
                runMotif(pattern),
                Turret.INSTANCE.off
        );
        return fullShootSequence;

    }
    public Command drive(){
       return Drivetrain.INSTANCE.startRobotDrive();
    }

    @Override
    public void periodic(){
        pattern = Limelight.INSTANCE.patternFromObelisk();
    }
}
