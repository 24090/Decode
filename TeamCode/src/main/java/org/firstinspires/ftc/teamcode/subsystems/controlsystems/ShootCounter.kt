package org.firstinspires.ftc.teamcode.subsystems.controlsystems

class ShootCounter(val threshold: Double = 100.0) {
    private var last = 0.0
    var count = 0
    fun update(newState: Double, target: Double){
        if ((newState < target - threshold) && !(last < target - threshold) && (newState - last) < threshold){
            count += 1
        }
        last = newState
    }

}