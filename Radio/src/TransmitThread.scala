object TransmitThread extends Thread {
  @Override
  override def run(): Unit = {
    while (true) {
      val pBytes = java.nio.ByteBuffer.allocate(200)


        pBytes.putChar('P')
        pBytes.putChar('/')
        pBytes.putInt(Terminal.p)
        pBytes.putChar('I')
        pBytes.putChar('/')
        pBytes.putInt(Terminal.i)
        pBytes.putChar('D')
        pBytes.putChar('/')
        pBytes.putInt(Terminal.d)
       /*catch {
        case e: Exception => System.out.println("oh noOOOOO")
      }*/
      try {
        Terminal.device.sendBroadcastData(pBytes.array())
      } catch {
        case e: Exception => "oh no"
      }

    }
  }
}
