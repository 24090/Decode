package org.firstinspires.ftc.teamcode.subsystems.ColorSensor

import android.graphics.Color
import androidx.core.graphics.toColor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import org.firstinspires.ftc.teamcode.util.BallColor
import java.util.Optional


class ColorSensor(hwMap: HardwareMap) {
    val left: NormalizedColorSensor = hwMap.get(NormalizedColorSensor::class.java, "colorSensorLeft")
    val right: NormalizedColorSensor = hwMap.get(NormalizedColorSensor::class.java, "colorSensorRight")
    var leftColor: Optional<BallColor> = Optional.empty()
        private set
    var rightColor: Optional<BallColor> = Optional.empty()
        private set

    public enum class ColorSensorOut{
        GREEN,
        PURPLE,
        NONE
    }
    fun getColor(argb:Int): Optional<BallColor>{
        TODO("we NEED to TUNE!!!!")
    }
    
    fun read(){
        leftColor = getColor(left.normalizedColors.toColor())
        rightColor = getColor(right.normalizedColors.toColor())
    }
    fun getSensedColor(color_sensor: NormalizedColorSensor): ColorSensorOut? {
        val color_int: Int = color_sensor.getNormalizedColors().toColor()
        val hsv = FloatArray(3)
        Color.colorToHSV(color_int, hsv)
        if (hsv[1] < 0.3) {
            return ColorSensorOut.NONE
        }
        if (hsv[0] <= 45 || hsv[0] > 290) {
            return ColorSensorOut.GREEN
        }
        if (hsv[0] >= 45 && hsv[0] <= 140) {
            return ColorSensorOut.PURPLE
        }
        return ColorSensorOut.NONE
    }
}

@TeleOp
class ColorSensorTesting: LinearOpMode(){
    val left: NormalizedColorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensorLeft")
    val right: NormalizedColorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "colorSensorRight")
    override fun runOpMode() {
        waitForStart();

        while(opModeIsActive()){
            val hsv_left = FloatArray(3)
            Color.colorToHSV(left.getNormalizedColors().toColor(), hsv_left)

            val hsv_right = FloatArray(3)
            Color.colorToHSV(right.getNormalizedColors().toColor(), hsv_right)

            telemetry.addData("left", hsv_left)
            telemetry.addData("right", hsv_right)
            telemetry.update()
        }
    }
}