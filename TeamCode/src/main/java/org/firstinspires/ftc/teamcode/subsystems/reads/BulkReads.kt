package org.firstinspires.ftc.teamcode.subsystems.reads

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

class BulkReads(hardwareMap: HardwareMap, val initManual: Boolean = true){
    private val lynxModules:  List<LynxModule> = listOf(
        hardwareMap.get(LynxModule::class.java, "Expansion Hub 2")
    )

    init {
        if (initManual) {
            hardwareMap.get(LynxModule::class.java, "Control Hub").bulkCachingMode = LynxModule.BulkCachingMode.MANUAL // just in case we accidentally plug something into the control hub it won't be a silent error
            for (module in lynxModules) {
                module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
            }
        }
    }
    fun update(){
        for (module in lynxModules){
            module.clearBulkCache()
        }
    }

}