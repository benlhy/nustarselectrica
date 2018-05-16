import javax.swing.*;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

public class Terminal {
    private JLabel xRot;
    private JLabel time;
    private JLabel tracking;
    private JLabel latitude;
    private JLabel longitude;
    private JTextField textFieldP;
    private JTextField textFieldI;
    private JTextField textFieldD;
    private JButton pidCommit;
    public JPanel mainPanel;
    private JLabel pLabel;
    private JLabel iLabel;
    private JLabel dLabel;
    private JLabel pidInfoLabel;
    private JLabel timeText;
    private JLabel xRotText;
    private JLabel trackingText;
    private JLabel latText;
    private JLabel lngText;
    public JTextArea console;
    private JSpinner comSpinner;
    private JButton openXBeeButton;
    private JLabel transmissionRatio;
    private JLabel loopDelay;
    private JLabel yRot;
    private JLabel zRot;
    private JLabel yRotText;
    private JLabel zRotText;


    public Terminal() {
        pidCommit.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent h) {
                try {
                    int tmpI = 1000 * Integer.parseInt(textFieldP.getText());
                    int tmpP = 1000 * Integer.parseInt(textFieldI.getText());
                    int tmpD = 1000 * Integer.parseInt(textFieldD.getText());
                    if (tmpP > 0xFFFF || tmpI > 0xFFFF || tmpD > 0xFFFF) {
                        Controller.say("PID Value too high!");
                        return;
                    }
                    Controller.setPID(tmpI, tmpP, tmpD);
                } catch (Exception e){
                    Controller.say("Invalid PID inputs!");
                }
            }
        });
    }

    public void updateUI() {
        time.setText(Controller.getData(0));
        xRot.setText(Controller.getData(1));
        yRot.setText(Controller.getData(2));
        zRot.setText(Controller.getData(3));
        tracking.setText(Controller.getData(4));
        longitude.setText(Controller.getData(5));
        latitude.setText(Controller.getData(6));
        loopDelay.setText(Controller.getData(7));

        transmissionRatio.setText(String.format("%s%.2f", "Transmission fail ratio: ", (double)Controller.transmissionFailures()/(Controller.transmissionSuccesses()+Controller.transmissionFailures())));
    }
}
