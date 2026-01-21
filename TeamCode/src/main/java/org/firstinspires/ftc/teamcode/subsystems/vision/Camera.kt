package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.commands.*
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector

import org.firstinspires.ftc.teamcode.util.Observation
import org.firstinspires.ftc.teamcode.util.Pattern
import kotlin.math.PI


class Camera(hwMap: HardwareMap) {
    enum class Pipeline {
        Pattern,
        Localize,
        RampAnalysis,
        FieldAnalysis
    }
    val limelight: Limelight3A = hwMap.get(Limelight3A::class.java, "limelight")
    var currentPipeline: Pipeline = Pipeline.Pattern

    init {
        initPattern()
    }

    fun initLocalize() {
        limelight.setPollRateHz(20)
        limelight.start()
        limelight.pipelineSwitch(0)
        currentPipeline = Pipeline.Localize
    }
    fun initPattern() {
        limelight.setPollRateHz(5)
        limelight.start()
        limelight.pipelineSwitch(0)
        currentPipeline = Pipeline.Pattern
    }

    fun initRampAnalysis() {
        limelight.setPollRateHz(12)
        limelight.start()
        limelight.pipelineSwitch(1)
        currentPipeline = Pipeline.RampAnalysis
    }

    fun getFieldAnalysis(): Vector? {
        if (currentPipeline != Pipeline.FieldAnalysis) error("Wrong Pipeline")
        return Vector.fromCartesian(limelight.latestResult.pythonOutput[0],limelight.latestResult.pythonOutput[1])
    }

    fun getRampAnalysis(): Observation.Camera? {
        if (currentPipeline != Pipeline.RampAnalysis) error("Wrong Pipeline")
        return Observation.Camera(limelight.latestResult.pythonOutput[0].toInt())
    }

    fun getPattern(): Pattern? {
        if (currentPipeline != Pipeline.Pattern) error("Wrong Pipeline")
        for (fiducialResult in limelight.latestResult.fiducialResults){
            when (fiducialResult.fiducialId){
                21 -> return Pattern.GPP
                22 -> return Pattern.PGP
                23 -> return Pattern.PPG
            }
        }
        return null
    }

    fun getPose(): Pose? {
        if (currentPipeline != Pipeline.Localize) error("Wrong Pipeline")
        for (fiducialResult in limelight.latestResult.fiducialResults){
            when(fiducialResult.fiducialId){
                20, 19 -> {
                    return Pose(
                        fiducialResult.robotPoseFieldSpace.position.x * 39.37 + 72.0,
                        fiducialResult.robotPoseFieldSpace.position.y * 39.37,
                        fiducialResult.robotPoseFieldSpace.orientation.yaw / 360.0 * 2*PI
                    )
                }
            }
        }
        return null
    }

    fun aprilTagRelocalize(localizer: Localizer,  timeout: Double = 1.0): Command = Sequence(
        Instant{ initPattern() },
        Race(
            Sleep(timeout),
            WaitUntil {
                val pose = getPose()
                if (pose!=null)  localizer.pose = localizer.pose * 3.0/4.0 + pose * 1.0/4.0
                return@WaitUntil (pose != null)
            }
        )
    )
}

@TeleOp(name = "VisionTesting")
class VisionTesting: LinearOpMode() {
    override fun runOpMode() {
        val camera = Camera(hardwareMap)
        camera.initLocalize()
        waitForStart()
        while (opModeIsActive()){
            if (gamepad1.xWasPressed()){camera.initLocalize()}
            if (gamepad1.yWasPressed()){camera.initPattern()}
            if (gamepad1.aWasPressed()){camera.initRampAnalysis()}
            if (gamepad1.bWasPressed()){}
            when (camera.currentPipeline) {
                Camera.Pipeline.Localize -> {
                    telemetry.addData("Pose", camera.getPose())
                }
                Camera.Pipeline.Pattern -> {
                    telemetry.addData("Pattern", camera.getPattern())
                }
                Camera.Pipeline.RampAnalysis -> {
                    telemetry.addData("Ramp", camera.getRampAnalysis())
                }

                Camera.Pipeline.FieldAnalysis -> TODO()
            }
            telemetry.update()
        }
    }
}