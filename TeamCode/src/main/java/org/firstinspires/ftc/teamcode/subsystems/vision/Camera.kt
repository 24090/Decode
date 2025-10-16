package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.drivetrain.Vector

import org.firstinspires.ftc.teamcode.util.Ball

class Camera(hwMap: HardwareMap) {
    val limelight: Limelight3A = hwMap.get(Limelight3A::class.java, "limelight")
    init {
        initAprilTag()
    }

    fun initAprilTag() {
        limelight.setPollRateHz(5)
        limelight.start()
        limelight.pipelineSwitch(0)
    }

    fun initChaseCheck() {
        limelight.setPollRateHz(100)
        limelight.start()
        limelight.pipelineSwitch(1)
    }

    fun getChase(): Array<Pair<Vector, Ball>>{
        TODO()
    }

    fun getCheck(): Array<Ball> {
        TODO()
    }

    fun getAprilTag(): Int? {
        for (fiducialResult in limelight.latestResult.fiducialResults){
            when (fiducialResult.fiducialId){
                21 -> return 0
                22 -> return 1
                23 -> return 2
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