import com.digi.xbee.api.XBeeDevice
import com.digi.xbee.api.exceptions.InvalidInterfaceException
import javax.swing._

import scala.collection.mutable.ListBuffer

object Controller extends JFrame {
  val DEBUG_NO_XBEE = false //don't try to load the xbee
  lazy val comDevice

  //XBEE INFORMATION
  val baud: Integer = 9600 //leave this


  var msg: String = "" //starts blank

  /*
  T: Time
  X, Y, Z: Rotation
  Tr: Tracking target for X
  Ln, Lt: Longitude, Latitude
  A: Altitude
  Tn: Transmisison number
   */
  val labels: List[String] = List("T", "X", "Y", "Z", "Tr", "Ln", "Lt", "Lp", "A", "Tn")
  var data: ListBuffer[String] = ListBuffer.fill(labels.length) {
    "N/A"
  }

  //the pid values which will be updated by the receiver
  private var _p: Int = 0

  def p: Int = _p

  def p_=(x: Int): Unit = {
    _p = x
  }

  private var _i: Int = 0

  def i: Int = _i

  def i_=(x: Int): Unit = {
    _i = x
  }

  private var _d: Int = 0

  def d: Int = _d

  def d_=(x: Int): Unit = {
    _d = x
  }

  def setPID(p: Int, i: Int, d: Int): Unit = {
    _p = p
    _i = i
    _d = d
  }

  private var _transmissionSuccesses = 0

  def transmissionSuccesses: Int = _transmissionSuccesses

  def transmissionSuccesses_=(x: Int): Unit = {
    _transmissionSuccesses = x
  }

  private var _transmissionFailures = 0

  def transmissionFailures: Int = _transmissionFailures

  def transmissionFailures_=(x: Int): Unit = {
    _transmissionFailures = x
  }

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


  val window = new JFrame("Terminal")
  val trm = new Terminal

  def control(device: XBeeDevice, port: String): Unit = {
    try {
      device.open()
      say("XBee online.")
      device.addDataListener(DataListener)
      TransmitThread.start()
    } catch {
      case e: InvalidInterfaceException => say("COULD NOT INITIALIZE XBEE: XBee not found on " + port)
      case e: Exception => say("COULD NOT INITIALIZE XBEE: " + e)
    }
    while (true) {
      Thread.sleep(50)
      for (i <- data.indices) {
        val thisMsg = parseMsg(labels(i))
        if (!thisMsg.equals("N/A")) data(i) = thisMsg
      }
      trm.updateUI()
    }
  }

  def main(args: Array[String]): Unit = {
    window.setContentPane(trm.mainPanel)
    window.pack()
    window.setVisible(true)
    say("Interface loaded!")

    //this should block at control() and will only repeat for the xbee having invalid values
    while (true) {
      Thread.sleep(500)
      val port = trm.serialCombo.getSelectedItem.toString
      val device = new XBeeDevice(port, baud)
      //go on the XBee
      control(device, port)
    }
  }
}
