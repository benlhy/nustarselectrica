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
          lst += 'I' += '/' += ((Controller.i >> 8) & 0xFF).asInstanceOf[Byte] += (Controller.i & 0xFF).asInstanceOf[Byte]
          lst += 'D' += '/' += ((Controller.d >> 8) & 0xFF).asInstanceOf[Byte] += (Controller.d & 0xFF).asInstanceOf[Byte]
        }
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
    Controller.say("Sending tracking instructions.")
    var trackingAlwaysFailed = true
    for (i <- 0 to 20) {
      try {
        Thread.sleep(50)
        val lst: ListBuffer[Byte] = new ListBuffer[Byte]
        lst += 'N' += 'X' += ' ' //special carrier code
        lst += 'a'
        for (i <- 0 until 6) {
          lst += (Controller.trackingTargets(i) & 0xFF).asInstanceOf[Byte]
        }
        lst += 'i'
        for (i <- 0 until 6) {
          lst += (Controller.trackingTimes(i) & 0xFF).asInstanceOf[Byte]
        }
        try {
          Controller.device.sendBroadcastData(lst.toArray)
          Controller.transmissionSuccesses += 1
          trackingAlwaysFailed = false
        } catch {
          case e: TimeoutException =>Controller.transmissionFailures += 1
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
