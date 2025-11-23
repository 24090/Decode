package org.firstinspires.ftc.teamcode.subsystems.huskylens

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.BallColor
import java.util.Optional

class HuskyLens(hwMap: HardwareMap) {
    val camera = hwMap.get(HuskyLens::class.java, "huskyLens")
    var left: Optional<BallColor>? = null
    var right: Optional<BallColor>? = null
    init {
        camera.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION)
    }
    fun read(){
        val comparator = Comparator {
                a: HuskyLens.Block, b: HuskyLens.Block ->  return@Comparator  a.width * a.height - b.width * b.height
        }
        val valid_blocks = camera
            .blocks()
            .filter{ block -> block.width * block.height >= 400 }
            .sortedBy{ block -> -(block.width*block.height) }
        left = null
        right = null
        for (block in valid_blocks) {
            if ((left != null) && (right != null)){
                break
            }
            val color = when (block.id){
                0 -> Optional.empty()
                1 -> Optional.of(BallColor.GREEN)
                2 -> Optional.of(BallColor.PURPLE)
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

    fun getColorPair(): Pair<Optional<BallColor>,Optional<BallColor>> {
        return Pair<Optional<BallColor>,Optional<BallColor>>(left,right)
    }
}