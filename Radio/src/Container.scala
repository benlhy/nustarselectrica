import Controller._

object Container extends Thread {
  override def run(): Unit = {
    while (true) {
      Thread.sleep(50)
      for (i <- data.indices) {
        val thisMsg = parseMsg(labels(i))
        if (!thisMsg.equals("N/A")) data(i) = thisMsg
      }

      trm.updateUI()
      window.repaint()
    }
  }
}
