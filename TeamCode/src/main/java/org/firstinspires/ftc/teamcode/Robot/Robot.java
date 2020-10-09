package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Util.Alliance;

import static org.firstinspires.ftc.teamcode.Robot.DriveFields.movement_turn;
import static org.firstinspires.ftc.teamcode.Robot.DriveFields.movement_x;
import static org.firstinspires.ftc.teamcode.Robot.DriveFields.movement_y;


/*
   Class that forms the base of the robot
 */
public class Robot {


    //The current OpMode
    public OpMode opMode;

    //Robot Components
    public DriveBase driveBase;


    //Road Runner Component

    //FTC Dashboard
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    //Timer for the loop time
    private  ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    //The starting positions for the robot.
    //TRADITIONAL ONLY
    private static final Pose2d redStartingPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(90));

    private static final Pose2d blueStartingPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(-90));

    //TODO: Decide on how to implement a coordinate system to the remote field.

    //REMOTE ONLY- to be changed
    private static final Pose2d remoteStartingPosition = new Pose2d(0, 0, Rotation2d.fromDegrees(-90));

    //To be used mostly in teleop - maybe use for remote??????????
    public static Pose2d userStartingPosition = new Pose2d(0,0, new Rotation2d(0));

    //The dashboard telemetry packet
    public TelemetryPacket packet = new TelemetryPacket();

    //The current Alliance...... may need to change for remote
    public Alliance alliance;


    public Robot(OpMode opMode, Alliance alliance){
        this.opMode = opMode;
        this.alliance = alliance;

        //Initialize components

        //Drive Base
        driveBase = new DriveBase(this);


        //Since we use odometry in both auto and teleop in this game, there is no need to tell the
        //drive base if we are in auto or not


        //Set the dashboard update time to 25 ms
        dashboard.setTelemetryTransmissionInterval(25);

        //Set the loop time action for telemetry up
        opMode.telemetry.addData("Loop Time (ms)", "%.2f", () -> timer.milliseconds());
    }

    //Called every loop to update the robot's components
    public void update(){
        //Update the bulk data cache before any other hardware updates


        //Update the odometry


        //TODO: MAKE SURE THIS ACTUALLY DOES NOTHING IN TELEOP
        //Update RoadRunner. If roadrunner isn't being used then it does nothing

        //Update drive motors

        //If it's teleop, update using the x, y, and turn components
        if ((alliance == Alliance.OTHER) || (alliance == Alliance.REMOTE_TELEOP)) {
            driveBase.updateHolonomic();
        } else { //Otherwise assume the powers have already been updated by roadrunner
            driveBase.updatePowers();
        }

        //Compile some telemetry to send (dashboard for now)
        compileTelemetry();

        //Reset the loop time timer
        timer.reset();
    }

    private void compileTelemetry(){

        opMode.telemetry.addData("Movement X", "%.2f", movement_x);
        opMode.telemetry.addData("Movement Y", "%.2f", movement_y);
        opMode.telemetry.addData("Movement Turn", "%.2f", movement_turn);

        opMode.telemetry.update();
    }






}
