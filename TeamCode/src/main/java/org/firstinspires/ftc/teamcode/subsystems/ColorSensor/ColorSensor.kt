package org.firstinspires.ftc.teamcode.subsystems.ColorSensor

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor

@TeleOp(name = "ColorSensorTest")
class ColorSensorTest: LinearOpMode() {
    override fun runOpMode() {
        val colorSensor = hardwareMap.get(ColorSensor::class.java, "colorSensorLeft")
        while (opModeInInit()){
            telemetry.addData("r", colorSensor.red())
            telemetry.addData("g", colorSensor.green())
            telemetry.addData("b", colorSensor.blue())
            telemetry.update()
        }
        waitForStart()
    }

}