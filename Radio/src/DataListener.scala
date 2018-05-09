import com.digi.xbee.api.listeners.IDataReceiveListener
import com.digi.xbee.api.models.XBeeMessage
import com.digi.xbee.api.utils.HexUtils

object DataListener extends IDataReceiveListener {
  def dataReceived(xbeeMessage: XBeeMessage): Unit = {
      Terminal.msg = new String(xbeeMessage.getData)
  }
}
