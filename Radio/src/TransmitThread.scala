import java.nio.ByteBuffer

import com.digi.xbee.api.exceptions.{InterfaceNotOpenException, TimeoutException}

import scala.collection.mutable.ListBuffer

object TransmitThread extends Thread {
  @Override
  override def run(): Unit = {
    Controller.say("Transmission thread is online")
    while (true) {
      Thread.sleep(50)
      try {
        val lst: ListBuffer[Byte] = new ListBuffer[Byte]
        lst += 'N' += 'U' += ' ' //carrier code
        lst += 'p' += '/' += {
          if (Controller.trm.useGroundPIDCheckBox.isSelected) 0x1
          else 0x0
        }
        lst += 'f' += '/' += {
          if (Controller.trm.forceOverrideTrackingCheckBox.isSelected) 0x1
          else 0x0
        }
        if (Controller.trm.forceOverrideTrackingCheckBox.isSelected) lst += 's' += '/' += {
          if (Controller.trm.trackingCheckBox.isSelected) 0x1
          else 0x0
        }
        if (Controller.trm.useGroundPIDCheckBox.isSelected) {
          lst += 'P' += '/' += ((Controller.p >> 8) & 0xFF).asInstanceOf[Byte] += (Controller.p & 0xFF).asInstanceOf[Byte] //PID values
          //lst += 'P' += '/' += 0x00 += 0x01
          lst += 'I' += '/' += ((Controller.i >> 8) & 0xFF).asInstanceOf[Byte] += (Controller.i & 0xFF).asInstanceOf[Byte]
          lst += 'D' += '/' += ((Controller.d >> 8) & 0xFF).asInstanceOf[Byte] += (Controller.d & 0xFF).asInstanceOf[Byte]
        }
        lst += 'C' += '/' += ((Controller.challenge >> 8) & 0xFF).asInstanceOf[Byte] += (Controller.challenge & 0xFF).asInstanceOf[Byte]
        try {
          Controller.device.sendBroadcastData(lst.toArray)
          Controller.transmissionSuccesses += 1
        } catch {
          case e: TimeoutException => Controller.transmissionFailures += 1
        }
      } catch {
        case e: InterfaceNotOpenException => Controller.say("Connection to XBee lost! You may need to restart the program.")
        //case e: Exception => Controller.say("Unexpected exception in transmit thread: " + e)
      }
    }
  }

  def sendTrackingImperatives(): Unit = {
    try {
      Controller.trackingTargets(0) = Integer.parseInt(Controller.trm.target0.getText())
      Controller.trackingTargets(1) = Integer.parseInt(Controller.trm.target1.getText())
      Controller.trackingTargets(2) = Integer.parseInt(Controller.trm.target2.getText())
      Controller.trackingTargets(3) = Integer.parseInt(Controller.trm.target3.getText())
      Controller.trackingTargets(4) = Integer.parseInt(Controller.trm.target4.getText())
      Controller.trackingTargets(5) = Integer.parseInt(Controller.trm.target5.getText())
      Controller.trackingTimes(0) = Integer.parseInt(Controller.trm.time0.getText())
      Controller.trackingTimes(1) = Integer.parseInt(Controller.trm.time1.getText())
      Controller.trackingTimes(2) = Integer.parseInt(Controller.trm.time2.getText())
      Controller.trackingTimes(3) = Integer.parseInt(Controller.trm.time3.getText())
      Controller.trackingTimes(4) = Integer.parseInt(Controller.trm.time4.getText())
      Controller.trackingTimes(5) = Integer.parseInt(Controller.trm.time5.getText())
    } catch {
      case e: Exception => Controller.say("Invalid tracking input!")
    }

    Controller.say("Sending tracking instructions.")
    var trackingAlwaysFailed = true
    for (i <- 0 to 20) {
      try {
        Thread.sleep(50)
        val lst: ListBuffer[Byte] = new ListBuffer[Byte]
        lst += 'N' += 'X' += ' ' //special carrier code
        lst += 'a'
        for (i <- 0 until 6) {
          val k = Controller.trackingTargets(i)
          lst += ((k >> 8) & 0xFF).asInstanceOf[Byte]
          lst += (k & 0xFF).asInstanceOf[Byte]
        }
        lst += 'i'
        for (i <- 0 until 6) {
          val k = Controller.trackingTimes(i)
          lst += ((k >> 8) & 0xFF).asInstanceOf[Byte]
          lst += (k & 0xFF).asInstanceOf[Byte]
        }
        try {
          Controller.device.sendBroadcastData(lst.toArray)
          Controller.transmissionSuccesses += 1
          trackingAlwaysFailed = false
        } catch {
          case e: TimeoutException => Controller.transmissionFailures += 1
        }
      } catch {
        case e: InterfaceNotOpenException => Controller.say("Connection to XBee lost! You may need to restart the program.")
        case e: Exception => Controller.say("Unexpected exception in transmit thread: " + e)
      }
    }
    if (trackingAlwaysFailed) {
      Controller.say("WARNING: Tracking instructions failed to send!")
    } else {
      Controller.say("Tracking instructions sent.")
    }
  }
}
