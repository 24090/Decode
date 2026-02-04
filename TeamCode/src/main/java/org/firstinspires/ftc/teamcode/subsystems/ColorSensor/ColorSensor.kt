package org.firstinspires.ftc.teamcode.subsystems.ColorSensor

import android.graphics.Color
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.teamcode.util.BallColor
import java.util.Optional
import kotlin.math.abs


class ColorSensor(hwMap: HardwareMap) {
    val left: NormalizedColorSensor = hwMap.get(NormalizedColorSensor::class.java, "colorSensorLeft")
    val right: NormalizedColorSensor = hwMap.get(NormalizedColorSensor::class.java, "colorSensorRight")
    var leftColor: Optional<BallColor> = Optional.empty()
        private set
    var rightColor: Optional<BallColor> = Optional.empty()
        private set

    fun read(){
        leftColor = getColor(left.normalizedColors.toColor())
        rightColor = getColor(right.normalizedColors.toColor())
    }
    private fun getColor(color: Int): Optional<BallColor> {
        val hsv = FloatArray(3)
        Color.colorToHSV(color, hsv)
        if (hsv[1] < 0.01) {
            return Optional.empty()
        }
        if (abs(hsv[0] - 210) < 30) {
            return Optional.of(BallColor.PURPLE)
        }
        if (abs(hsv[0] - 160) < 30) {
            return Optional.of(BallColor.GREEN)
        }
        return Optional.empty()
    }
}

@TeleOp
class ColorSensorTesting: LinearOpMode(){
    override fun runOpMode() {
        val colorSensor = ColorSensor(hardwareMap)
        val left: com.qualcomm.robotcore.hardware.ColorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor::class.java, "colorSensorLeft")
        val right: com.qualcomm.robotcore.hardware.ColorSensor = hardwareMap.get(com.qualcomm.robotcore.hardware.ColorSensor::class.java, "colorSensorRight")

        waitForStart()

        while(opModeIsActive()){
            colorSensor.read()
            val alpha = left.alpha()
            val argb_left = left.argb()
            val hsv_left = FloatArray(3)
            Color.colorToHSV(Color.rgb(left.red(), left.green(), left.blue()), hsv_left)
            telemetry.addData("left - detected color", colorSensor.leftColor)
            telemetry.addData("left hsva", "${hsv_left[0]}, ${hsv_left[1]}, ${hsv_left[2]}, ${left.alpha()}")
            telemetry.addLine(if (left.alpha() > 1000) "there" else if (left.alpha() > 100) "hole" else "none")
            telemetry.update()
        }
    }
}