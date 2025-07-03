package PhysicsSim;
import java.awt.event.*;
import javax.swing.*;
import java.awt.*;

class Display extends JPanel implements ActionListener, MouseListener, MouseMotionListener {
    private JFrame frame = new JFrame();
    private double mouseX = 0.0;
    private double mouseY = 0.0;
    private boolean mousePressed = false;
    public static Double[] pointOfContactDraw = new Double[]{0.0,0.0};
    private double timeElapsed = 0.0;
    private double lastTime = System.currentTimeMillis() / 1000.0;
    public Display(int screenWidth, int screenHeight) {
        setBounds(0,0,screenWidth,screenHeight);
        setBackground(Color.white);
        addMouseMotionListener(this);
        addMouseListener(this);

        frame.setBounds(100,100,screenWidth,screenHeight);
        frame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

        frame.setVisible(true);
        frame.add(this);
    }
    public void mouseDragRigidbodies(double dt) {
        timeElapsed = System.currentTimeMillis() - lastTime;
        if (timeElapsed / 1000.0 > dt) {
            Rigidbody.moveByMouse(mouseX, mouseY, true, mousePressed, dt);
            Point.moveByMouse(mouseX, mouseY, true, mousePressed, dt);
        }
        else {
            Rigidbody.moveByMouse(mouseX, mouseY, false, mousePressed, dt);
            Point.moveByMouse(mouseX, mouseY, false, mousePressed, dt);
        }
    }
    public void refresh() {
        repaint();
    }

    @Override
    public void paintComponent(Graphics g) {
        super.paintComponent(g);
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            g.setColor(Rigidbody.get(i).getColor());
            int[] x = Rigidbody.get(i).getX();
            int[] y = Rigidbody.get(i).getY();
            int length = x.length;
            g.fillPolygon(x, y, length);
            //draw center of mass unless that drawing would be larger than the shape
            if (Rigidbody.get(i).getLargestDistance() > 15) {
                g.setColor(Color.red);
                g.fillOval((int) Rigidbody.get(i).getPosX() - 5, (int) Rigidbody.get(i).getPosY() - 5, 10, 10);
            }
        }
        //drawing each point
        for (int i = 0; i < Point.num; i = i + 1) {
            g.setColor(Point.get(i).getColor());
            int radius = (int)Point.get(i).getRadius();
            g.fillOval((int)Point.get(i).getX() - radius, (int)Point.get(i).getY() - radius, 2 * radius, 2 * radius);
            //draw the center of mass unless the Point is too small (in which case the center of mass circle would be larger than the Point)
            if (radius > 15) {
                g.setColor(Color.red);
                g.fillOval((int)Point.get(i).getX() - 5, (int)Point.get(i).getY() - 5, 10, 10);
            }
            //lines to show spring attachments
            if (Point.get(i).isAttached()) {
                for (int j = 0; j < Point.get(i).getAttachmentNum(); j = j + 1) {
                    if (Point.get(i).ID < Point.get(i).getAttachment(j).ID) {
                        g.drawLine((int)Point.get(i).getX(), (int)Point.get(i).getY(), (int)Point.get(i).getAttachment(j).getX(), (int)Point.get(i).getAttachment(j).getY());
                    }
                }
            }
        }
        g.setColor(Color.black);
        if (false && Rigidbody.num > 0) {
            g.fillOval((int) (pointOfContactDraw[0] - 5), (int) (pointOfContactDraw[1] - 5), 10, 10);
            g.setColor(Color.red);
            int index = 2;
            double[] X = Rigidbody.get(0).getTriangles().get(index).getX();
            int[] x = new int[]{(int)X[0], (int)X[1], (int)X[2]};
            double[] Y = Rigidbody.get(0).getTriangles().get(index).getY();
            int[] y = new int[]{(int)Y[0], (int)Y[1], (int)Y[2]};
            g.fillPolygon(x, y, 3);
        }

        g.setColor(Color.red);
        g.fillOval((int) mouseX - 5, (int) mouseY - 5, 10, 10);
    }

    @Override
    public void actionPerformed(ActionEvent e) {

    }

    @Override
    public void mouseClicked(MouseEvent e) {

    }

    @Override
    public void mousePressed(MouseEvent e) {
        mousePressed = true;
        lastTime = System.currentTimeMillis();
        mouseX = e.getX();
        mouseY = e.getY();
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        mousePressed = false;
        lastTime = System.currentTimeMillis();
        mouseX = e.getX();
        mouseY = e.getY();
    }

    @Override
    public void mouseEntered(MouseEvent e) {

    }

    @Override
    public void mouseExited(MouseEvent e) {

    }

    @Override
    public void mouseDragged(MouseEvent e) {
        lastTime = System.currentTimeMillis();
        mouseX = e.getX();
        mouseY = e.getY();
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        mouseX = e.getX();
        mouseY = e.getY();
    }
}
