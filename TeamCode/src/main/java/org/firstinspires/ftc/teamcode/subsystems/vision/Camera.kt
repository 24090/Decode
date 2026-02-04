package org.firstinspires.ftc.teamcode.subsystems.vision

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.commands.Command
import org.firstinspires.ftc.teamcode.commands.Instant
import org.firstinspires.ftc.teamcode.commands.Race
import org.firstinspires.ftc.teamcode.commands.Sequence
import org.firstinspires.ftc.teamcode.commands.Sleep
import org.firstinspires.ftc.teamcode.commands.WaitUntil
import org.firstinspires.ftc.teamcode.subsystems.drive.Localizer
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Pose
import org.firstinspires.ftc.teamcode.subsystems.drive.pathing.Vector
import org.firstinspires.ftc.teamcode.util.Observation
import org.firstinspires.ftc.teamcode.util.Pattern
import java.net.HttpURLConnection
import java.net.URL
import java.nio.charset.StandardCharsets
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
        sendPostRequest("/update-imumode", "3")
    }

    fun initLocalize() {
        limelight.pipelineSwitch(2)
        limelight.setPollRateHz(20)
        limelight.start()
        currentPipeline = Pipeline.Localize
    }
    fun initPattern() {
        limelight.pipelineSwitch(2)
        limelight.setPollRateHz(5)
        limelight.start()
        currentPipeline = Pipeline.Pattern
    }

    fun initRampAnalysis() {
        limelight.setPollRateHz(12)
        limelight.pipelineSwitch(1)
        limelight.start()
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

    fun sendHeading(heading: Double){
        limelight.updateRobotOrientation(heading)
    }
    fun getPose(): Pose? {
        val result = limelight.latestResult
        if (result == null || !result.isValid){
            return null
        }
        val botPose = result.botpose_MT2
        if (result.botpose == null){
            return null
        }

        if (currentPipeline != Pipeline.Localize) error("Wrong Pipeline")

        return botPose.let { Pose(
            it.position.x * 39.37 + 72.0,
            it.position.y * 39.37,
            it.orientation.yaw / 360.0 * 2*PI
        ) }
    }

    fun aprilTagRelocalize(localizer: Localizer,  timeout: Double = 1.0): Command = Sequence(
        Instant{
            initPattern()
            sendHeading(localizer.heading)
        },
        Race(
            Sleep(timeout),
            WaitUntil {
                val pose = getPose()
                if (pose!=null)  localizer.pose = localizer.pose * 3.0/4.0 + pose * 1.0/4.0
                return@WaitUntil (pose != null)
            }
        )
    )

    /**
     * Sends a POST request to the specified endpoint.
     *
     * @param endpoint The endpoint to send the request to.
     * @param data The data to send in the request body.
     * @return true if successful, false otherwise.
     */
    private fun sendPostRequest(endpoint: String, data: String?): Boolean {
        var connection: HttpURLConnection? = null
        try {
            val urlString: String = "limelight.local:5807" + endpoint
            val url = URL(urlString)
            connection = url.openConnection() as HttpURLConnection?
            connection!!.setRequestMethod("POST")
            connection.setDoOutput(true)
            connection.setRequestProperty("Content-Type", "application/json")
            connection.setReadTimeout(15000)
            connection.setConnectTimeout(100)

            if (data != null) {
                connection.getOutputStream().use { os ->
                    val input = data.toByteArray(StandardCharsets.UTF_8)
                    os.write(input, 0, input.size)
                }
            }

            val responseCode = connection.getResponseCode()
            if (responseCode == HttpURLConnection.HTTP_OK) {
                return true
            } else {
                //System.out.println("HTTP POST Error: " + responseCode);
            }
        } catch (e: Exception) {
            //e.printStackTrace();
        } finally {
            if (connection != null) {
                connection.disconnect()
            }
        }
        return false
    }
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