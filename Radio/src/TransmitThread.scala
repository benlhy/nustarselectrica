object TransmitThread extends Thread {
  @Override
  override def run(): Unit = {
    while (true) {
      try {
        Terminal.device.sendBroadcastData(Array[Byte]('P', '/', Terminal.p.asInstanceOf[Byte], '/', 'I', '/', Terminal.i.asInstanceOf[Byte], '/', 'D', '/', Terminal.d.asInstanceOf[Byte], '/', '\n'))
      } catch {
        case e: Exception => System.out.println("oh no")
      }
    }
  }
}
