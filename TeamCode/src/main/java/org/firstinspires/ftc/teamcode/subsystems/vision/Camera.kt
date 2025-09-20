package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap


class Camera(hwMap: HardwareMap) {
    val limelight: Limelight3A = hwMap.get(Limelight3A::class.java, "limelight")
    init {
        limelight.setPollRateHz(5) // This sets how often we ask Limelight for data (100 times per second)
        limelight.start() // This tells Limelight to start looking!
        limelight.pipelineSwitch(0);
    }

    fun getAprilTag(): Int? {
        for (fiducialResult in limelight.latestResult.fiducialResults){
            when (fiducialResult.fiducialId){
                22 -> return 0
                23 -> return 1
                24 -> return 2
            }
        }
        return null
    }
}

@TeleOp(name = "VisionTesting")
class VisionTesting: LinearOpMode() {
    override fun runOpMode() {
        waitForStart()
        while (opModeIsActive()){
            telemetry.addData("G index", Camera(hardwareMap).getAprilTag())
            telemetry.update()
        }
    }
}