package org.firstinspires.ftc.teamcode.TeleOp

import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.control.PIDFController
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Robot.DriveFields
import org.firstinspires.ftc.teamcode.Robot.Odometry
import org.firstinspires.ftc.teamcode.Robot.RoadRunner.DriveConstants
import org.firstinspires.ftc.teamcode.Robot.Robot
import org.firstinspires.ftc.teamcode.Util.Alliance
import org.firstinspires.ftc.teamcode.Util.Toggle

@TeleOp(name = "Main TeleOp")
@Config
class MainTeleOp : LinearOpMode() {



    companion object {
        //The controller that rotates the robot to aim the shooter automatically
        @JvmField
        var autoAimCoeffs = PIDCoefficients(1.0, 0.0, 0.0)

        @JvmField
        var autoAimkV = 1.0


        //TODO: Fill this in
        //To be used when auto hasn't been run first, ask at the start of teleop whether to use this or the saved pose
        var defaultStartingPose = Pose2d(0.0, 0.0, 0.0)
    }

    //Enum to represent the position of the high goal and powershots per side
    //X increases towards goals, and y increases towards the blue alliance station
    enum class AllianceTarget(val highGoal: Vector2d, val leftPowershot: Vector2d, val centerPowershot: Vector2d, val rightPowershot: Vector2d) {
        RED(Vector2d(1.0, 1.0), Vector2d(1.0, 1.0), Vector2d(1.0, 1.0), Vector2d(1.0, 1.0)),
        BLUE(Vector2d(1.0, 1.0), Vector2d(1.0, 1.0), Vector2d(1.0, 1.0), Vector2d(1.0, 1.0)),
        REMOTE(Vector2d(72.0, 36.0), Vector2d(1.0, 1.0), Vector2d(1.0, 1.0), Vector2d(1.0, 1.0));
    }


    //Enum to represent the rotation control of the robot for the shooter; manual really just means "nothing"
    enum class AimMode {
        MANUAL,
        AUTO
    }

    //The (mostly P) controller to control the rotation of the robot to point the shooter
    private lateinit var autoAimController: PIDFController


    //The current target to aim for. This should always be in sync with the shooter target in
    //Shooter.kt, therefore changing this should only be done with changeTarget().
    private var target: Vector2d = AllianceTarget.REMOTE.highGoal



    //Stores the method of rotating the robot for the shooter
    private var aimMode = AimMode.MANUAL


    //Our robot instance
    private lateinit var robot: Robot


    //Toggles for various controls

    private lateinit var aimModeToggle: Toggle

    private lateinit var flywheelToggle: Toggle


    private var slowModeMult: Double = 1.0



    /**
     * Let's keep a list of the current robot controls here:
     *
     * Gamepad 1 (Driver)
     * left stick x = robot translational
     * left stick y = robot translational
     * right stick x = robot rotational
     * left bumper = slow mode
     * right bumper = aim mode toggle (Manual or auto)
     *
     *
     *
     *
     * Gamepad 2 (Manipulator)
     *
     *
     * Wobble goal arm up & down toggle = x
     * Wobble goal claw open & close toggle = b
     *
     * Hopper up and down toggle = a
     * flicker cycle = right bumper
     *
     *
     * left bumper = flywheel toggle
     *
     *
     *
     * left trigger = intake
     * left trigger + a = outtake (reversed)
     *
     * dpad up/down = cycle through shooter targets (high goal, left powershot,
     * central powershot, and right powershot) //REMOVED
     *
     */
    @Throws(InterruptedException::class)
    override fun runOpMode() {

        //This lets the driver select which starting position to use

        telemetry.addLine("Please indicate the starting position!")
        telemetry.addLine("(Gamepad 1) Press X to use the auto position, and Y to use" +
                "the default teleop position of 0, 0 facing towards the goal.")
        telemetry.update()
        while(!gamepad1.x || !gamepad1.y){
            if(gamepad1.x){
                robot = Robot(this, Alliance.TELEOP)
                telemetry.addLine("Starting position set as last known position in auto!")
            }
            if(gamepad1.y){
                robot = Robot(this, Alliance.TELEOP, defaultStartingPose)
                telemetry.addLine("Starting position is set as default!")
            }
        }
        telemetry.update()

        //Initialize the robot with the given position
        robot = Robot(this, Alliance.TELEOP, defaultStartingPose)


        //Start the P controller that controls rotation during auto-aim
        autoAimController = PIDFController(autoAimCoeffs)
        autoAimController.setOutputBounds(-1.0, 1.0)
        autoAimController.setInputBounds(-Math.PI, Math.PI) //Limits of Euler angles

        //Set up auto-aim toggle
        aimModeToggle = Toggle(
                { aimMode = AimMode.AUTO },
                { aimMode = AimMode.MANUAL },
                { gamepad1.right_bumper }
        )

        //Srt up flywheel toggle
        flywheelToggle = Toggle(
                { robot.shooter.turnOnFlywheel(true) },
                { robot.shooter.turnOnFlywheel(false) },
                { gamepad2.left_bumper }
        )

        //Initialization done, wait for start
        waitForStart()


        //Loops throughout teleop
        while (opModeIsActive() && !isStopRequested) {
            if (isStopRequested) return



            //Set the slow mode to actual change the drive powers if the bumper is pressed
            slowModeMult = if(gamepad1.left_bumper) 0.5 else 1.0

            aimModeToggle.update()

            flywheelToggle.update()

            //TODO: Some way to change the target


            when(aimMode){
                AimMode.MANUAL -> {

                    //Give the translation controls of the robot
                    DriveFields.movement_x = -gamepad1.left_stick_x.toDouble() * slowModeMult
                    DriveFields.movement_y = -gamepad1.left_stick_y.toDouble() * slowModeMult
                    // We reverse our rotation empirically for our dt
                    DriveFields.movement_turn = gamepad1.right_stick_x.toDouble() * slowModeMult
                }
                AimMode.AUTO -> {

                    // Create a vector from the gamepad x/y inputs which is the field relative movement
                    // Then, rotate that vector by the inverse of that heading for field centric control
                    var fieldFrameInput = Vector2d(
                            (-gamepad1.left_stick_y).toDouble(),
                            (-gamepad1.left_stick_x).toDouble()
                    )

                    var robotFrameInput = fieldFrameInput.rotated(-Odometry.world_pose.heading)

                    // Difference between the target vector and the bot's position

                    // Difference between the target vector and the bot's position
                    val difference: Vector2d = target.minus(Odometry.world_pose.vec())
                    // Obtain the target angle for feedback and derivative for feedforward
                    val theta = difference.angle()


                    // Not technically omega because its power. This is the derivative of atan2
                    val thetaFF = -fieldFrameInput.rotated(-Math.PI / 2).dot(difference) / (difference.norm() * difference.norm())

                    // Set the target heading for the heading controller to our desired angle
                    autoAimController.targetPosition = theta

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward

                    // Set desired angular velocity to the heading controller output + angular
                    // velocity feedforward
                    val headingInput: Double = ((autoAimController.update(Odometry.world_pose.heading)
                            * autoAimkV + thetaFF)
                            * DriveConstants.TRACK_WIDTH
                            )

                    // Pass our x/y gamepad inputs and our angular velocity
                    DriveFields.movement_x = robotFrameInput.x
                    DriveFields.movement_y = robotFrameInput.y
                    DriveFields.movement_turn = headingInput

                }
            }


            //Update the auto aim controller with our current heading each time
            autoAimController.update(Odometry.world_pose.heading)


            robot.update()
        }
        robot.end()
    }

}