package org.firstinspires.ftc.teamcode.subsystems.huskylens

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.BallColor
import org.firstinspires.ftc.teamcode.util.Pattern
import java.util.Optional
import kotlin.jvm.optionals.getOrNull

class HuskyLens(hwMap: HardwareMap) {
    val camera: HuskyLens = hwMap.get(HuskyLens::class.java, "huskyLens")

    var left: Optional<BallColor>? = null
        private set
    var right: Optional<BallColor>? = null
        private set

    init {
        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION)
    }
    fun read(){
        left = null
        right = null

        val valid_blocks = camera
            .blocks()
            .filter{ block -> block.width * block.height >= 400 }
            .sortedBy{ block -> -(block.width*block.height*(240 - block.y)) }

        for (block in valid_blocks) {
            if ((left != null) && (right != null)){
                break
            }
            val color = when (block.id){
                3 -> Optional.empty()
                2 -> Optional.of(BallColor.PURPLE)
                1 -> Optional.of(BallColor.GREEN)
                else -> throw UnsupportedOperationException("id > 2 is invalid")
            }

            if ((block.x < 160) && (right == null)) {
                right = color
            }

            if ((block.x > 160) && (left == null)) {
                left = color
            }
        }
    }

    // LEFT RIGHT 3 -> GPP/PGP/PPG
    fun getHeldPattern() = when (BallColor.GREEN) {
        left?.getOrNull() -> {
            Pattern.GPP
        }
        right?.getOrNull() -> {
            Pattern.PGP
        }
        else -> {
            Pattern.PPG
        }
    }
}

@TeleOp
class HuskyLensTesting: LinearOpMode() {
    override fun runOpMode() {
        val huskyLens = HuskyLens(hardwareMap)
        val lights = Lights(hardwareMap)
        val dash = FtcDashboard.getInstance()
        lights.turnOn()
        waitForStart()
        while (opModeIsActive()){
            val p = TelemetryPacket()
            huskyLens.read()
            p.put("Pattern", huskyLens.getHeldPattern().toString())
            telemetry.addData("PATTERN", huskyLens.getHeldPattern())
            telemetry.update()
            dash.sendTelemetryPacket(p)
        }
    }

}