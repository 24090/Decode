package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.drivetrain.Pose
import org.firstinspires.ftc.teamcode.drivetrain.Vector

import org.firstinspires.ftc.teamcode.util.BallColor
import org.firstinspires.ftc.teamcode.util.Observation
import org.firstinspires.ftc.teamcode.util.Pattern

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
        limelight.setPollRateHz(12)
        limelight.start()
        limelight.pipelineSwitch(1)
    }

    fun getChase(): Array<Pair<Vector, BallColor>>{
        TODO()
    }

    fun getCheck() = Observation.Camera(limelight.latestResult.pythonOutput[0].toInt())

    fun getPattern(): Pattern? {
        for (fiducialResult in limelight.latestResult.fiducialResults){
            when (fiducialResult.fiducialId){
                21 -> return Pattern.GPP
                22 -> return Pattern.PGP
                23 -> return Pattern.PPG
            }
        }
        return null
    }
    fun getPoseFromAprilTag(): Pose? {
        for (fiducialResult in limelight.latestResult.fiducialResults){
            when(fiducialResult.fiducialId){
                24, 25 -> {
                    return Pose(fiducialResult.robotPoseFieldSpace.position.x,fiducialResult.robotPoseFieldSpace.position.y,fiducialResult.robotPoseFieldSpace.orientation.yaw)
                }
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
            telemetry.addData("Pattern", Camera(hardwareMap).getPattern())
            telemetry.update()
        }
    }
}