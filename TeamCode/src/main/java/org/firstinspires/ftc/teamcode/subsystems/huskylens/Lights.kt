package org.firstinspires.ftc.teamcode.subsystems.huskylens

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx

class Lights(hwMap: HardwareMap) {
    val left: ServoImplEx = hwMap.get(ServoImplEx::class.java, "lightLeft")
    val right: ServoImplEx = hwMap.get(ServoImplEx::class.java, "lightRight")

    init {
        left.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        right.pwmRange = PwmControl.PwmRange(500.0, 2500.0)
        left.position = 0.0
        right.position = 0.0
    }

    fun turnOn(){
        left.position = 1.0
        right.position = 1.0
    }

    fun turnOff(){
        left.position = 0.0
        right.position = 0.0
    }
}