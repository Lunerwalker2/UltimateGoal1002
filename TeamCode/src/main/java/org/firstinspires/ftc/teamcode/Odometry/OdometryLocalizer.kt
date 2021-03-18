package org.firstinspires.ftc.teamcode.Odometry

import androidx.annotation.NonNull
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.Util.HardwareNames


class OdometryLocalizer(val hardwareMap: HardwareMap):
        ThreeTrackingWheelLocalizer(listOf(
                Pose2d(0.0, design.LATERAL_DISTANCE / 2, 0.0),  // left
                Pose2d(0.0, -design.LATERAL_DISTANCE / 2, 0.0),  // right
                Pose2d(design.FORWARD_OFFSET, 0.0, Math.toRadians(90.0)) // front
        )) {

    companion object {
        private val design = DesignCharacteristics(15.0802, 4.25) //3.55598425197

        private val odometers = OdometerCharacteristics(8192.0, 1.0, 1.0)


        fun ticksToInches(ticks: Int): Double {
            return odometers.WHEEL_RADIUS * 2 * Math.PI * odometers.GEAR_RATIO * ticks / odometers.TICKS_PER_REV
        }
    }

    private val left_y_encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, HardwareNames.Odometry.LEFT_Y_ENCODER))
    private val right_y_encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, HardwareNames.Odometry.RIGHT_Y_ENCODER))
    private val x_encoder = Encoder(hardwareMap.get(DcMotorEx::class.java, HardwareNames.Odometry.X_ENCODER))


    private val X_MULTIPLIER = 1;
    private val Y_MULTIPLIER = 1;

    init {

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
        left_y_encoder.direction = Encoder.Direction.REVERSE
        right_y_encoder.direction = Encoder.Direction.REVERSE

    }


    @NonNull
    override fun getWheelPositions(): List<Double> {
         return listOf(
                 ticksToInches(left_y_encoder.currentPosition) * X_MULTIPLIER,
                 ticksToInches(right_y_encoder.currentPosition) * X_MULTIPLIER,
                 ticksToInches(x_encoder.currentPosition) * Y_MULTIPLIER //returns in radians
         )
    }

    override fun getWheelVelocities(): List<Double> {
        return listOf(
                ticksToInches(left_y_encoder.correctedVelocity.toInt()) * X_MULTIPLIER,
                ticksToInches(right_y_encoder.correctedVelocity.toInt()) * X_MULTIPLIER,
                ticksToInches(x_encoder.correctedVelocity.toInt()) * Y_MULTIPLIER
        )
    }

}


/**
 * TRACK_WIDTH = the trackwidth of the two y encoders
 * FORWARD_OFFSET = the offset of the x encoder
 */
data class DesignCharacteristics(val LATERAL_DISTANCE: Double, val FORWARD_OFFSET: Double)
data class OdometerCharacteristics(val TICKS_PER_REV: Double, val WHEEL_RADIUS: Double, val GEAR_RATIO: Double)