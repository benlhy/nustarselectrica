import java.awt.{BorderLayout, FlowLayout, Font}
import java.util

import com.digi.xbee.api.XBeeDevice
import com.digi.xbee.api.packet.XBeePacket
import javax.swing._

import scala.collection.mutable.ListBuffer

object Terminal extends JFrame {

  //XBEE INFORMATION
  val port: String = "COM8"
  val baud: Integer = 9600

  //UI OBJECTS
  val buttonPane = new JPanel()
  val xRot = new JLabel()
  val time = new JLabel()
  val tracking = new JLabel()
  val longitude = new JLabel()
  val latitude = new JLabel()

  val pSet = new JTextField()
  val iSet = new JTextField()
  val dSet = new JTextField()

  var msg: String = ""
  var data: ListBuffer[String] = ListBuffer.fill(5){""}

  this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE)
  this.setLayout(new BorderLayout())

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
  def main(args: Array[String]) : Unit = {
    System.out.println("Ready!")

    xRot.setText("SAMPLE TEXT!")
    this.add(buttonPane, BorderLayout.WEST)
    buttonPane.setLayout(new BoxLayout(buttonPane, BoxLayout.Y_AXIS))
    val f: Font = new Font("Arial", Font.PLAIN, 20)
    time.setFont(f)
    xRot.setFont(f)
    tracking.setFont(f)
    longitude.setFont(f)
    latitude.setFont(f)
    buttonPane.add(time)
    buttonPane.add(xRot)
    buttonPane.add(tracking)
    buttonPane.add(longitude)
    buttonPane.add(latitude)
    buttonPane.add(pSet)
    buttonPane.add(iSet)
    buttonPane.add(dSet)
    this.setVisible(true)


    //go on the XBee
    val device = new XBeeDevice(port, baud)
    device.open()
    device.addDataListener(DataListener)

    while(true) {
      Thread.sleep(50)
      val labels: List[String] = List("T", "X", "Tr", "Ln", "Lt")
      for (i <- data.indices) {
        val thisMsg = parseMsg(labels(i))
        if (!thisMsg.equals("N/A")) data(i) = thisMsg
      }

      time.setText("Time: " + data(0))
      xRot.setText("X rotation: " + data(1))
      tracking.setText("Tracking goal: " + data(2))
      longitude.setText("Longitude: " + data(3))
      latitude.setText("Latitude: " + data(4))
    }
  }
}
