import java.nio.ByteBuffer

import com.digi.xbee.api.exceptions.{InterfaceNotOpenException, TimeoutException}

import scala.collection.mutable.ListBuffer

object TransmitThread extends Thread {
  @Override
  override def run(): Unit = {
    Controller.say("Transmission thread is online")
    while (true) {
      try {
        Thread.sleep(50)
        val lst: ListBuffer[Byte] = new ListBuffer[Byte]
        lst += 'N' += 'U' += ' ' += 'P' += '/' += ((Controller.p >> 8) & 0xFF).asInstanceOf[Byte] += (Controller.p & 0xFF).asInstanceOf[Byte]
        try {
          Controller.device.sendBroadcastData(lst.toArray)
          Controller.transmissionSuccesses += 1
        } catch {
          case e: TimeoutException => Controller.transmissionFailures += 1
        }
      } catch {
        case e: InterfaceNotOpenException => Controller.say("Connection to XBee lost! You may need to restart the program.")
        case e: Exception => Controller.say("Unexpected exception in transmit thread: " + e)
      }

    }
  }
}
