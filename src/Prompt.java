import javax.swing.*;
import java.awt.*;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;

public class Prompt {
    private final JTextField textInput;
    public boolean promptComplete = false;
    public String input = "";
    public Prompt(String[] validInput) {
        JFrame promptFrame = new JFrame();
        promptFrame.setBounds(100, 100, 200, 150);
        promptFrame.setBackground(Color.white);
        JPanel promptPanel = new JPanel();
        promptPanel.setBounds(100, 100, 200, 150);
        promptPanel.setBackground(Color.white);
        promptPanel.setLayout(new FlowLayout(FlowLayout.CENTER));
        promptFrame.add(promptPanel);
        promptFrame.setVisible(true);
        JTextField textInput = new JTextField();
        textInput.setBounds(25, 25, 130, 50);
        textInput.setFocusable(true);
        textInput.requestFocusInWindow();
        textInput.setFocusTraversalKeysEnabled(false);
        textInput.requestFocus();
        SwingUtilities.invokeLater(textInput::requestFocusInWindow);
        textInput.setBackground(Color.gray);
        promptPanel.add(textInput);
        promptPanel.repaint();
        promptFrame.repaint();
        this.textInput = textInput;
        textInput.addKeyListener(new KeyListener(){
            @Override
            public void keyTyped(KeyEvent e) {
                if (!promptComplete && e.getKeyChar() == '\n') {
                    for (String validResponse : validInput) {
                        if (textInput.getText().equals(validResponse)) {
                            promptComplete = true;
                            promptFrame.dispose();
                            input = textInput.getText();
                            Main.beginRunning();
                        }
                    }

                }
            }

            @Override
            public void keyPressed(KeyEvent e) {

            }

            @Override
            public void keyReleased(KeyEvent e) {

            }
        });
    }
}
