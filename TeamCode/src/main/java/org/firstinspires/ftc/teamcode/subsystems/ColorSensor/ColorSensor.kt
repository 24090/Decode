package org.firstinspires.ftc.teamcode.subsystems.ColorSensor

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.teamcode.util.BallColor
import java.util.Optional

class ColorSensor {
    val left: ColorSensor = hardwareMap.get(ColorSensor::class.java, "colorSensorLeft")
    val right: ColorSensor = hardwareMap.get(ColorSensor::class.java, "colorSensorRight")
    
    var leftColor: Optional<BallColor> = Optional.empty()
        private set
    var rightColor: Optional<BallColor> = Optional.empty()
        private set
    
    fun getColor(argb:Int): Optional<BallColor>{
        TODO("we NEED to TUNE!!!!")
    }
    
    fun read(){
        leftColor = getColor(left.argb())
        rightColor = getColor(right.argb())
    }
    
}