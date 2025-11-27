package org.firstinspires.ftc.teamcode.subsystems.huskylens

import com.qualcomm.hardware.dfrobot.HuskyLens
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
            .sortedBy{ block -> -(block.width*block.height) }

        for (block in valid_blocks) {
            if ((left != null) && (right != null)){
                break
            }
            val color = when (block.id){
                0 -> Optional.empty()
                1 -> Optional.of(BallColor.PURPLE)
                2 -> Optional.of(BallColor.GREEN)
                else -> throw UnsupportedOperationException("id > 2 is invalid")
            }

            if ((block.x > 400) && (right == null)) {
                right = color
            }

            if ((block.x < 200) && (left == null)) {
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