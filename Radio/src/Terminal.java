import com.digi.xbee.api.XBeeDevice;

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
    public JButton openXBeeButton;
    private JLabel transmissionRatio;
    private JLabel loopDelay;
    private JLabel yRot;
    private JLabel zRot;
    private JLabel yRotText;
    private JLabel zRotText;
    private JButton zeroButton;
    private JLabel altitudeText;
    private JLabel altitude;
    public JCheckBox useGroundPIDCheckBox;
    public JComboBox serialCombo;
    public JTextField time0;
    public JTextField target0;
    public JTextField time1;
    public JTextField target1;
    public JTextField time2;
    public JTextField target2;
    public JTextField time3;
    public JTextField target3;
    public JTextField time4;
    public JTextField target4;
    public JTextField time5;
    public JTextField target5;
    public JButton sendTrackingButton;
    public JCheckBox forceOverrideTrackingCheckBox;
    public JCheckBox trackingCheckBox;


    public Terminal() {
        pidCommit.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent h) {
                try {
                    int tmpI = (int)(1000 * Double.parseDouble(textFieldP.getText()));
                    int tmpP = (int)(1000 * Double.parseDouble(textFieldI.getText()));
                    int tmpD = (int)(1000 * Double.parseDouble(textFieldD.getText()));
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
        zeroButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                Controller.setPID(0, 0, 0);
            }
        });
        openXBeeButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                String port = serialCombo.getSelectedItem().toString();
                XBeeDevice device = new XBeeDevice(port, 9600);
                Controller.control(device, port);
            }
        });
        sendTrackingButton.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                TransmitThread.sendTrackingImperatives();
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
        altitude.setText(Controller.getData(8));
        transmissionRatio.setText(String.format("%s%.2f", "Transmission fail ratio: ", (double)Controller.transmissionFailures()/(Controller.transmissionSuccesses()+Controller.transmissionFailures())));

    }
}
