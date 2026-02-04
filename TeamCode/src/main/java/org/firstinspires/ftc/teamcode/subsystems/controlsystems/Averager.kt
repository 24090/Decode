package org.firstinspires.ftc.teamcode.subsystems.controlsystems

import java.lang.Double.sum
import java.util.LinkedList

class Averager(val storeCount: Int) {
    private var pastStates = LinkedList<Double>()
    var lastTime: Long = 0
    var thisTime: Long = 0
    fun update(newState: Number, time: Long){
        lastTime = thisTime
        if (pastStates.size >= storeCount){
            pastStates.removeAt(0)
        }
        pastStates.add(newState.toDouble())
        thisTime = time
    }
    fun get() = if (pastStates.isEmpty()) 0.0 else pastStates.sum()/ pastStates.size
    fun deriv() = if(pastStates.size<20) 0.0 else (pastStates[pastStates.size-1]-pastStates[pastStates.size-20])/(thisTime-lastTime)

    fun derivSpike() = this.deriv() > 17

    fun isStalling(): Boolean {
        if (this.derivSpike()){
            val current = System.currentTimeMillis()
            while (System.currentTimeMillis()-current > 50){
            }
            if ((this.get() < 1050)&&(this.get() > 900)&&(Math.abs(this.deriv())<15)){
                return true
            } else {
                return false
            }
        } else {
            return (this.get() < 1050)&&(this.get() > 900)&&(Math.abs(this.deriv())<15)
        }
    }

}