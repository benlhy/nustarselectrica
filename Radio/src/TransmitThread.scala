import java.nio.ByteBuffer

import com.digi.xbee.api.exceptions.TimeoutException

import scala.collection.mutable.ListBuffer

object TransmitThread extends Thread {
  @Override
  override def run(): Unit = {
    Controller.say("Transmission thread is online")
    while (true) {
      Thread.sleep(50)
      val lst: ListBuffer[Byte] = new ListBuffer[Byte]
      lst += 'P' += '/' += ((Controller.p >> 8) & 0xFF).asInstanceOf[Byte] += (Controller.p & 0xFF).asInstanceOf[Byte]
      try {
        Controller.device.sendBroadcastData(lst.toArray)
        Controller.transmissionSuccesses += 1
      } catch {
        case e: TimeoutException => Controller.transmissionFailures += 1
      }

    }
  }
}
