package org.firstinspires.ftc.teamcode.Vision.pipelines


import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

/*
This class is the pipeline used for detecting the ring stack in auto.
 */
class RingStackPipeline(): OpenCvPipeline() {

    companion object {
        //Dimensions for the sample region at the top of the stack
        var topRingRegionX: Int = 200
        var topRingRegionY: Int = 200
        //Maybe make var??
        val topRingRegionWidth: Int = 40
        val topRingRegionHeight: Int = 70

        //Dimensions for the sample region at the bottom of the stack
        var bottomRingRegionX: Int = 200
        var bottomRingRegionY: Int = 300
        val bottomRingRegionWidth: Int = 40
        val bottomRingRegionHeight: Int = 70

        //If the mean value of the cb mat is below this, a ring is detected
        var thresholdValue: Int = 100


    }

    //Enum to represent the wobble goal zone
    enum class DeliveryZone {
        A,
        B,
        C
    }

    //Define some Mats we can reuse.
    var yCbCrMat = Mat()
    var cBmat = Mat()


    //The will hold the corresponding delivery zone for the ring stack
    var deliveryZone: DeliveryZone = DeliveryZone.A




    /*
    The main pipeline for the vision. This should run in the init phase of auto. We are given an
    inout frame and we must return one of the same size/color space that will go to the viewport.

    To ensure we don't mess up anything, we will do as little as possible with the input frame,
    outside of same graphical feedback that shows the results of the pipeline.
     */
    override fun processFrame(input: Mat): Mat {

        //Copy the original frame to one we can alter
        input.copyTo(yCbCrMat)


        //Convert the input mat from RGB to YCbCr
        Imgproc.cvtColor(yCbCrMat, yCbCrMat, Imgproc.COLOR_RGB2YCrCb)

        //Extract the Cb channel
        Core.extractChannel(yCbCrMat, cBmat, 2)

        //Make the sample rectangles
        val topSampleRect = Rect(topRingRegionX, topRingRegionY, topRingRegionWidth, topRingRegionHeight)
        val bottomSampleRect = Rect(bottomRingRegionX, bottomRingRegionY, bottomRingRegionWidth, bottomRingRegionHeight)

        //Submat the Cb Channel into the sample regions
        val topSampleRegion = cBmat.submat(topSampleRect)
        val bottomSampleRegion = cBmat.submat(bottomSampleRect)

        //Average the values for the sample regions
        val topRegionMean: Scalar = Core.mean(topSampleRegion)
        val bottomRegionMean: Scalar = Core.mean(bottomSampleRegion)

        //Compare the means to a threshold value to determine the presence of a ring in that area.
        val topRingDetected: Boolean = (topRegionMean.`val`[0] < thresholdValue)
        val bottomRingDetected: Boolean = (bottomRegionMean.`val`[0] < thresholdValue)


        /*
        Use some logic to determine which delivery zone is needed

        0 rings = Zone A
        1 ring = Zone B
        4 rings = Zone C
         */
        deliveryZone = when {
            topRingDetected && bottomRingDetected -> DeliveryZone.C
            !topRingDetected && bottomRingDetected -> DeliveryZone.B
            else -> DeliveryZone.A
        }

        //Now that we have all the data we need we can draw some stuff on the viewport

        //Draw the sample regions and indicate whether there is a ring detected or not

        //Top region
        when(deliveryZone){
            DeliveryZone.A, DeliveryZone.B -> Imgproc.rectangle(input, topSampleRect, Scalar(190.0, 40.0, 70.0), 2)
            DeliveryZone.C -> Imgproc.rectangle(input, topSampleRect, Scalar(20.0, 220.0, 70.0), 2)
        }

        //Bottom region
        when(deliveryZone){
            DeliveryZone.A -> Imgproc.rectangle(input, bottomSampleRect, Scalar(190.0, 40.0, 70.0), 2)
            DeliveryZone.B, DeliveryZone.C -> Imgproc.rectangle(input, bottomSampleRect, Scalar(20.0, 220.0, 70.0), 2)
        }






        return input
    }
}