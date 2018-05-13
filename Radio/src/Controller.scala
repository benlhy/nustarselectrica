import java.awt.event.ActionEvent
import java.awt.event.ActionListener
import java.awt.{BorderLayout, FlowLayout, Font, GridLayout}

import com.digi.xbee.api.XBeeDevice
import com.digi.xbee.api.exceptions.InvalidInterfaceException
import com.digi.xbee.api.packet.XBeePacket
import javax.swing._

import scala.collection.mutable.ListBuffer

object Controller extends JFrame {
  val DEBUG_NO_XBEE = false


  //XBEE INFORMATION
  val port: String = "COM8"
  val baud: Integer = 9600


  var msg: String = ""
  val labels: List[String] = List("T", "X", "Tr", "Ln", "Lt")
  var data: ListBuffer[String] = ListBuffer.fill(5){"N/A"}

  var p: Int = 0
  var i: Int = 0
  var d: Int = 0

/*
  commitButton.addActionListener(new ActionListener() {
    def actionPerformed(e: ActionEvent): Unit = {
      try {
        p = (1000 * pSet.getText.toDouble).toInt
        i = (1000 * iSet.getText.toDouble).toInt
        d = (1000 * dSet.getText.toDouble).toInt
      } catch {
        case e: Exception => System.out.println("WARNING: GOT INVALID PID STUFF")
      }
    }
  })
*/

  def say(s: String): Unit = {
    System.out.println(s)
    trm.console.append(s + '\n')
  }
  //so Java can communicate with us
  def getData(x: Int): String = {
    data(x)
  }

  def parseMsg(label: String): String = {
    if (msg.length > 0) {
      val ix = msg.indexOf(label + ":")
      var is = 0
      if (ix >= 0) {
        is = msg.substring(ix).indexOf("/")
      } else {
        is = 0
      }
      if (ix >= 0 && is >= 0) {
        try {
          return msg.substring(ix + label.length + 1, ix + is)
        } catch {
          case e: Exception =>
            System.out.println("Warning: encountered substring exception!")
            return "N/A"

        }
      }
    }
    "N/A"
  }

  lazy val device = new XBeeDevice(port, baud)

  val window = new JFrame("Terminal")
  val trm = new Terminal

  def main(args: Array[String]) : Unit = {

    window.setContentPane(trm.mainPanel)
    window.pack()
    window.setVisible(true)
    say("Interface loaded!")

    //go on the XBee
    if (!DEBUG_NO_XBEE) {
      try {
        device.open()
        say("XBee online.")
        device.addDataListener(DataListener)
        TransmitThread.start()
      } catch {
        case e: InvalidInterfaceException => say("COULD NOT INITIALIZE XBEE: XBee not found on " + port)
        case e: Exception => say("COULD NOT INITIALIZE XBEE: UNKNOWN ERROR!")
      }
    } else {
      say("NOT initializing XBee, debug flag is set!")
    }
    while(true) {
      Thread.sleep(50)
      for (i <- data.indices) {
        val thisMsg = parseMsg(labels(i))
        if (!thisMsg.equals("N/A")) data(i) = thisMsg
      }
      trm.updateUI()
    }
  }
}
