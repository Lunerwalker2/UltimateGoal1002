package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Robot.RoadRunner.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Util.Alliance;
import org.firstinspires.ftc.teamcode.Util.PoseStorage;

import java.io.File;
import java.io.IOException;

import static java.lang.Math.toRadians;
import static java.lang.Double.parseDouble;

import static org.firstinspires.ftc.teamcode.Robot.DriveFields.movement_turn;
import static org.firstinspires.ftc.teamcode.Robot.DriveFields.movement_x;
import static org.firstinspires.ftc.teamcode.Robot.DriveFields.movement_y;

import static org.firstinspires.ftc.teamcode.Robot.Odometry.world_x;
import static org.firstinspires.ftc.teamcode.Robot.Odometry.world_y;
import static org.firstinspires.ftc.teamcode.Robot.Odometry.world_r;


/*
   Class that forms the base of the robot
 */
public class Robot {


    //The current OpMode
    public OpMode opMode;

    //Robot Components
    public DriveBase driveBase;

    public BulkData bulkData;

    public Odometry odometry;

    public SampleMecanumDrive roadRunnerBase;

    public Shooter shooter;


    //Road Runner Component

    //FTC Dashboard
    public FtcDashboard dashboard = FtcDashboard.getInstance();

    //Timer for the loop time
    private ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


    //The dashboard telemetry packet
    public TelemetryPacket packet = new TelemetryPacket();

    //The current Alliance...... may need to change for remote
    public Alliance alliance;

    //Use this in a teleop run where odometry tracking doesn't matter (i.e. testing)

    //IMPORTANT:If a position is NOT given in the constructor, the starting position will
    //be read from a file that indicates the last known position of the robot at the last run of auto.

    public Robot(OpMode opMode, Alliance alliance) {
        this(opMode, alliance, PoseStorage.currentPose);
    }

    public Robot(OpMode opMode, Alliance alliance, Pose2d startingPosition) {
        this.opMode = opMode;
        this.alliance = alliance;

        //Initialize components

        driveBase = new DriveBase(this);
        bulkData = new BulkData(this);
        //Set the cache mode to manual
        bulkData.setManual();
        shooter = new Shooter(this);

        odometry = new Odometry(this, startingPosition);


//        if (alliance.isAuto) {
//            odometry = alliance == Alliance.RED ? new Odometry(this,
//                    redTraditionalStartingPosition) : new Odometry(this, blueTraditionalStartingPosition);
//        } else {
//            odometry = new Odometry(this, startingPosition);
//        }


        //Initialize RR
        roadRunnerBase = new SampleMecanumDrive(this);


        //Since we use odometry in both auto and teleop in this game, there is no need to tell the
        //drive base if we are in auto or not


        //Set the dashboard update time to 25 ms
        dashboard.setTelemetryTransmissionInterval(50);

        //Set the loop time action for telemetry up
        opMode.telemetry.addData("Loop Time (ms)", "%.2f", () -> timer.milliseconds());
    }


    //Called every loop to update the robot's components
    public void update() {
        //Update the bulk data cache before any other hardware updates
        bulkData.update();


        //Update the odometry
        odometry.update();


        if (alliance.isAuto) {
            roadRunnerBase.update();
        }

        //Update drive motors

        //If it's teleop, update using the x, y, and turn components
        if (!alliance.isAuto) {
            driveBase.updateHolonomic();
        } else { //Otherwise assume the powers have already been updated by roadrunner
            driveBase.updatePowers();
        }

        //Compile some telemetry to send (dashboard for now)
        compileTelemetry();

        //Reset the loop time timer
        timer.reset();
    }

    //Stops all motors and writes the current pose to a static variable for teleop
    public void end(){
        shooter.turnOnFlywheel(false);
        driveBase.stop();

        if(alliance.isAuto) PoseStorage.currentPose = Odometry.world_pose;
    }

    private void compileTelemetry() {

        opMode.telemetry.addData("Movement X", "%.2f", movement_x);
        opMode.telemetry.addData("Movement Y", "%.2f", movement_y);
        opMode.telemetry.addData("Movement Turn", "%.2f", movement_turn);

        opMode.telemetry.addData("World X", "%.2f", world_x);
        opMode.telemetry.addData("World Y", "%.2f", world_y);
        opMode.telemetry.addData("World Heading", "%.2f", world_r);

        opMode.telemetry.update();

        dashboard.sendTelemetryPacket(packet);
    }

    //This method writes the contents of a position object to a text file
    public static void writeLastKnownPosition(Pose2d lastKnownPosition) {
        File file = AppUtil.getInstance().getSettingsFile("LastKnownAutoPose.txt");
        String values = lastKnownPosition.getX() + "," + lastKnownPosition.getY() + "," + lastKnownPosition.getHeading();
        ReadWriteFile.writeFile(file, values);
    }

    //This method reads the contents of the text file and returns the position object
    public static Pose2d readLastKnownPosition() {
        File file = AppUtil.getInstance().getSettingsFile("LastKnownAutoPose.txt");
        String contents = ReadWriteFile.readFile(file);
        String[] values = contents.split(",");
        return new Pose2d(parseDouble(values[0]), parseDouble(values[1]), parseDouble(values[2]));
    }


}
