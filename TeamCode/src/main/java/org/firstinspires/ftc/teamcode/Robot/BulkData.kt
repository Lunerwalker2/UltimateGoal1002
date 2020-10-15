package org.firstinspires.ftc.teamcode.Robot

import com.qualcomm.hardware.lynx.LynxModule
import org.firstinspires.ftc.teamcode.Util.HardwareNames
import org.openftc.revextensions2.ExpansionHubEx

class BulkData(robot: Robot): Component(robot) {



    val HUB_LEFT: ExpansionHubEx

    val HUB_RIGHT: ExpansionHubEx

    private val modules: MutableList<ExpansionHubEx> = ArrayList(2)

    init {
        HUB_LEFT = hardwareMap.get(ExpansionHubEx::class.java, HardwareNames.Hubs.HUB_LEFT)
        HUB_RIGHT = hardwareMap.get(ExpansionHubEx::class.java, HardwareNames.Hubs.HUB_RIGHT)

        //add to list for easy access
        modules.add(HUB_LEFT)
        modules.add(HUB_RIGHT)

    }

    /**
     *     Update "everything" lol. Basically just clear the cache
     */
    fun update() {
        clearCache()
    }

    /**
     * Clear the cache
     */
    private fun clearCache() {
        modules.forEach {module: ExpansionHubEx -> module.standardModule.clearBulkCache()}

    }

    /**
     * Set the mode as MANUAL
     */
    private fun setManual() {
        modules.forEach { module: ExpansionHubEx -> module.standardModule.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL   }
    }

    /**
     * Set the mode as AUTO
     */
    fun setAuto() {
        modules.forEach { module: ExpansionHubEx -> module.standardModule.bulkCachingMode = LynxModule.BulkCachingMode.AUTO   }
    }

    /**
     * Set the mode as OFF
     */
    fun setOff() {
        modules.forEach { module: ExpansionHubEx -> module.standardModule.bulkCachingMode = LynxModule.BulkCachingMode.OFF   }
    }

    

}