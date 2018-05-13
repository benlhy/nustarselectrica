import javax.swing.*;

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

    public void updateUI() {
        time.setText(Controller.getData(0));
        xRot.setText(Controller.getData(1));
        tracking.setText(Controller.getData(2));
        longitude.setText(Controller.getData(3));
        latitude.setText(Controller.getData(4));
    }
}
