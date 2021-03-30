package org.firstinspires.ftc.teamcode.Util



class Toggle(private val action1: () -> Unit, private val action2: () -> Unit, private val control: () -> Boolean) {


    private var state: Boolean = false
    private var changed: Boolean = false

    fun update(){
        state = if(control.invoke() && !state){
            takeAction()
            true
        } else {
            false
        }
    }

    private fun takeAction(){
        changed = if(!changed){
            action1.invoke()
            true
        } else {
            action2.invoke()
            false
        }
    }

}