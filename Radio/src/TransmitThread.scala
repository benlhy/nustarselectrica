object TransmitThread extends Thread {
  @Override
  override def run(): Unit = {
    Controller.say("Transmission thread is online")
    while (true) {
      val pBytes = java.nio.ByteBuffer.allocate(200)

        pBytes.putChar('N')
        pBytes.putChar('U')
        pBytes.putChar(' ')
        pBytes.putChar('P')
        pBytes.putChar('/')
        pBytes.putInt(Controller.p)
        pBytes.putChar('I')
        pBytes.putChar('/')
        pBytes.putInt(Controller.i)
        pBytes.putChar('D')
        pBytes.putChar('/')
        pBytes.putInt(Controller.d)
       /*catch {
        case e: Exception => System.out.println("oh noOOOOO")
      }*/
      try {
        Controller.device.sendBroadcastData(pBytes.array())
      } catch {
        case e: Exception => "oh no"
      }

    }
  }
}
