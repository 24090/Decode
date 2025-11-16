package org.firstinspires.ftc.teamcode.subsystems.reads

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.HardwareMap

class BulkReads(hardwareMap: HardwareMap){
    private val lynxModules = hardwareMap.getAll(LynxModule::class.java)
    init {
        for (module in lynxModules) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }
    }
    fun update(){
        for (module in lynxModules){
            module.clearBulkCache()
        }
    }

}