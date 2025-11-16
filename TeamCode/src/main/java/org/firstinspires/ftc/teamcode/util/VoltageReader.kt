package org.firstinspires.ftc.teamcode.util

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor


object VoltageReader {
    var controlHubVoltage: Double = 13.0
        private set
    var expansionHubVoltage: Double = 13.0
        private set

    fun updater(hwMap: HardwareMap): () -> Unit {
        val controlHubVoltageSensor = hwMap.get(VoltageSensor::class.java, "Control Hub")
        val expansionHubVoltageSensor = hwMap.get(VoltageSensor::class.java, "Expansion Hub")
        return {
            controlHubVoltage = controlHubVoltageSensor.voltage
            expansionHubVoltage = expansionHubVoltageSensor.voltage
        }
    }
}