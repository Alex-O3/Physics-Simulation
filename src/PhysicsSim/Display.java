package PhysicsSim;
import java.awt.event.*;
import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

class Display extends JPanel implements MouseListener, MouseMotionListener, MouseWheelListener, KeyListener {
    public final int simID;
    public int debugVector = 0;
    public double debugVectorScaling = 0.001;
    public final JLabel fps = new JLabel();

    public boolean mouse = false;
    public double resolutionScaling = 1.0;
    public double shiftX = 0.0;
    public double shiftY = 0.0;
    public double pixelShiftX = 0.0;
    public double pixelShiftY = 0.0;
    public double resolutionCenterX = 250.0;
    public double resolutionCenterY = 250.0;
    private boolean startedScrolling = true;
    private double mouseX = 0.0;
    private double mouseY = 0.0;
    private double mouseXDisplay = 0.0;
    private double mouseYDisplay = 0.0;
    private double vmXDisplay = 0.0;
    private double vmYDisplay = 0.0;
    private boolean mousePressed = false;
    private double timeElapsed = 0.0;
    private double lastTime = System.currentTimeMillis() / 1000.0;

    public boolean hasController = false;
    public char keyPressed;
    public final ArrayList<Character> keysCache = new ArrayList<>();
    public boolean firstPress = true;
    public boolean keyReleased = true;
    public boolean keyReleasedFirstTime = false;
    public char keyReleasedForTheFirstTime;
    public double lastKeyTime = System.currentTimeMillis();

    public boolean playerLock = false;

    public Display(int x, int y, int screenWidth, int screenHeight, int simID, boolean mouse) {
        this.simID = simID;
        setBounds(0,0,screenWidth,screenHeight);
        setBackground(Color.white);
        setLayout(new FlowLayout(FlowLayout.LEFT));
        this.mouse = mouse;
        if (mouse) {
            addMouseMotionListener(this);
            addMouseListener(this);
            addMouseWheelListener(this);
        }

        JFrame frame = new JFrame();
        frame.setBounds(x,y,screenWidth,screenHeight);
        frame.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);

        add(fps);
        fps.setText("FPS: ");

        frame.setVisible(true);
        frame.add(this);
        this.setFocusable(true);
        this.requestFocusInWindow();
        this.setFocusTraversalKeysEnabled(false);
        this.requestFocus();
        SwingUtilities.invokeLater(this::requestFocusInWindow);
    }
    public void mouseDrag(double dt, int stepsPerFrame) {
        dt = dt * stepsPerFrame;
        timeElapsed = System.currentTimeMillis() - lastTime;
        //System.out.println(mouseX + ", " + mouseY);
        boolean isInside = false;
        boolean results1;
        boolean results2;
        if (timeElapsed / 1000.0 > dt) {
            results1 = Rigidbody.moveByMouse(mouseX, mouseY, true, mousePressed, dt, simID);
            results2 = Point.moveByMouse(mouseX, mouseY, true, mousePressed, dt, simID);
            vmXDisplay = 0.0;
            vmYDisplay = 0.0;
        }
        else {
            results1 = Rigidbody.moveByMouse(mouseX, mouseY, false, mousePressed, dt, simID);
            results2 = Point.moveByMouse(mouseX, mouseY, false, mousePressed, dt, simID);
        }
        isInside = (results1 || results2);
        if (mousePressed && !isInside && timeElapsed > 0.0) {
            pixelShiftX = pixelShiftX + vmXDisplay * (dt / stepsPerFrame);
            pixelShiftY = pixelShiftY + vmYDisplay * (dt / stepsPerFrame);
        }
    }
    public void refresh() {
        if (playerLock) {
            removeMouseListener(this);
            removeMouseMotionListener(this);
            //removeMouseWheelListener(this);
            mouse = false;
        }

        repaint();
    }

    @Override
    public void paintComponent(Graphics g) {
        if (Simulation.get(simID).physicsObjects.isEmpty()) return;
        super.paintComponent(g);
        for (int i = 0; i < Rigidbody.num; i = i + 1) {
            try {
                if (Simulation.get(simID).getObject("Rigidbody", i).material.name.contains("Player")) {
                    playerLock = true;
                    resolutionCenterX = Rigidbody.get(i).getPosX();
                    resolutionCenterY = Rigidbody.get(i).getPosY();
                    pixelShiftX = 0.5 * getWidth() - ((Rigidbody.get(i).getPosX() - resolutionCenterX) / resolutionScaling) - resolutionCenterX;
                    pixelShiftY = 0.5 * getHeight() - ((Rigidbody.get(i).getPosY() - resolutionCenterY) / resolutionScaling) - resolutionCenterY;
                }
            } catch (Exception e) {

            }
            if (Rigidbody.get(i).simID != simID || !Rigidbody.get(i).draw) continue;
            g.setColor(Rigidbody.get(i).getColor());
            int[] x = Rigidbody.get(i).getDrawX(shiftX, resolutionScaling, resolutionCenterX, pixelShiftX);
            int[] y = Rigidbody.get(i).getDrawY(shiftY, resolutionScaling, resolutionCenterY, pixelShiftY);
            int length = x.length;
            g.fillPolygon(x, y, length);
            //draw center of mass unless that drawing would be larger than the shape
            if (Rigidbody.get(i).getLargestDistance() / resolutionScaling > 15.0 && Rigidbody.get(i).isMovable()) {
                g.setColor(Color.red);
                g.fillOval(convertX(Rigidbody.get(i).getPosX()) - 5, convertY(Rigidbody.get(i).getPosY()) - 5, 10, 10);
            }
            //draw debug velocity and acceleration vectors
            if (debugVector == 1) {
                g.setColor(Color.red);
                g.drawLine(convertX(Rigidbody.get(i).getPosX()), convertY(Rigidbody.get(i).getPosY()), convertX(Rigidbody.get(i).getPosX() + Rigidbody.get(i).getVX() * debugVectorScaling), convertY(Rigidbody.get(i).getPosY() + Rigidbody.get(i).getVY() * debugVectorScaling));
            }
            else if (debugVector == 2) {
                g.setColor(Color.blue);
                g.drawLine(convertX(Rigidbody.get(i).getPosX()), convertY(Rigidbody.get(i).getPosY()), convertX(Rigidbody.get(i).getPosX() + Rigidbody.get(i).getAX() * debugVectorScaling), convertY(Rigidbody.get(i).getPosY() + Rigidbody.get(i).getAY() * debugVectorScaling));
            }
            //draw face if one-sided face object
            if (Rigidbody.get(i).getTriangles().get(0).partOfFace) {
                g.setColor(Color.red);
                g.drawLine(x[0], y[0], x[1], y[1]);
            }
        }
        //drawing each point
        for (int i = 0; i < Point.num; i = i + 1) {
            try {
                if (Simulation.get(simID).getObject("Point", i).material.name.contains("Player")) {
                    playerLock = true;
                    resolutionCenterX = Point.get(i).getX();
                    resolutionCenterY = Point.get(i).getY();
                    pixelShiftX = 0.5* getWidth() - ((Point.get(i).getX() - resolutionCenterX) / resolutionScaling) - resolutionCenterX;
                    pixelShiftY = 0.5 * getHeight() - ((Point.get(i).getY() - resolutionCenterY) / resolutionScaling) - resolutionCenterY;
                }
            } catch (Exception e) {
                //throw new RuntimeException(e);
            }
            if (Point.get(i).simID != simID || !Point.get(i).draw) continue;
            g.setColor(Point.get(i).getColor());
            double radius = Point.get(i).getRadius();
            g.fillOval(convertX(Point.get(i).getX() - radius), convertY(Point.get(i).getY() - radius), (int)(radius * (2.0 / resolutionScaling)), (int)(radius * (2.0 / resolutionScaling)));
            //draw the center of mass unless the Point is too small (in which case the center of mass circle would be larger than the Point)
            if (radius / resolutionScaling > 15.0 && Point.get(i).isMovable()) {
                g.setColor(Color.red);
                g.fillOval(convertX(Point.get(i).getX()) - 5, convertY(Point.get(i).getY()) - 5, 10, 10);
            }
            //draw debug velocity and acceleration vectors
            if (debugVector == 1) {
                g.setColor(Color.red);
                g.drawLine(convertX(Point.get(i).getX()), convertY(Point.get(i).getY()), convertX(Point.get(i).getX() + Point.get(i).getVX() * debugVectorScaling), convertY(Point.get(i).getY() + Point.get(i).getVY() * debugVectorScaling));
            }
            else if (debugVector == 2) {
                g.setColor(Color.blue);
                g.drawLine(convertX(Point.get(i).getX()), convertY(Point.get(i).getY()), convertX(Point.get(i).getX() + Point.get(i).getAX() * debugVectorScaling), convertY(Point.get(i).getY() + Point.get(i).getAY() * debugVectorScaling));
            }
            //lines to show spring attachments
            if (Point.get(i).isAttached()) {
                for (int j = 0; j < Point.get(i).getAttachmentNum(); j = j + 1) {
                    if (Point.get(i).ID < Point.get(i).getAttachment(j).ID) {
                        g.drawLine(convertX(Point.get(i).getX()), convertY(Point.get(i).getY()), convertX(Point.get(i).getAttachment(j).getX()), convertY(Point.get(i).getAttachment(j).getY()));
                    }
                }
            }
        }
        //drawing the world border
        if (Simulation.get(simID).bounds) {
            g.setColor(Color.black);
            int[] p1 = new int[]{convertX(Simulation.get(simID).worldLeftBound), convertY(Simulation.get(simID).worldTopBound)};
            int[] p2 = new int[]{convertX(Simulation.get(simID).worldRightBound), p1[1]};
            int[] p3 = new int[]{p2[0], convertY(Simulation.get(simID).worldBottomBound)};
            int[] p4 = new int[]{p1[0], p3[1]};
            g.drawLine(p1[0], p1[1], p2[0], p2[1]);
            g.drawLine(p2[0], p2[1], p3[0], p3[1]);
            g.drawLine(p3[0], p3[1], p4[0], p4[1]);
            g.drawLine(p4[0], p4[1], p1[0], p1[1]);
        }

        if (mouse) {
            //draw mouse cursor
            g.setColor(Color.red);
            g.fillOval(convertX(mouseX) - 5, convertY(mouseY) - 5, 10, 10);
        }
    }
    private int convertX(double x) {
        return((int)Math.round(((x - resolutionCenterX) / resolutionScaling) + resolutionCenterX + pixelShiftX));
    }
    private int reverseConvertX(double x) {
        return((int)Math.round((x - resolutionCenterX - pixelShiftX) * resolutionScaling + resolutionCenterX));
    }
    private int convertY(double y) {
        return((int)Math.round(((y - resolutionCenterY) / resolutionScaling) + resolutionCenterY + pixelShiftY));
    }
    private int reverseConvertY(double y) {
        return((int)Math.round((y - resolutionCenterY - pixelShiftY) * resolutionScaling + resolutionCenterY));
    }

    private void updateMouseInfo(double x, double y) {
        double currentTime = System.currentTimeMillis();
        double timeElapsed = (currentTime - lastTime) / 1000.0;
        lastTime = currentTime;
        mouseX = reverseConvertX(x);
        mouseY = reverseConvertY(y);
        if (timeElapsed > 0.0) {
            vmXDisplay = (x - mouseXDisplay) / (timeElapsed);
            vmYDisplay = (y - mouseYDisplay) / (timeElapsed);
        }
        else {
            vmXDisplay = 0.0;
            vmYDisplay = 0.0;
        }
        mouseXDisplay = x;
        mouseYDisplay = y;
    }

    @Override
    public void mouseClicked(MouseEvent e) {

    }

    @Override
    public void mousePressed(MouseEvent e) {
        mousePressed = true;
        updateMouseInfo(e.getX(), e.getY());
    }

    @Override
    public void mouseReleased(MouseEvent e) {
        mousePressed = false;
        updateMouseInfo(e.getX(), e.getY());
    }

    @Override
    public void mouseEntered(MouseEvent e) {

    }

    @Override
    public void mouseExited(MouseEvent e) {

    }

    @Override
    public void mouseDragged(MouseEvent e) {
        updateMouseInfo(e.getX(), e.getY());
    }

    @Override
    public void mouseMoved(MouseEvent e) {
        startedScrolling = true;
        updateMouseInfo(e.getX(), e.getY());
    }

    @Override
    public void mouseWheelMoved(MouseWheelEvent e) {
        lastTime = System.currentTimeMillis();
        double numClicks = e.getWheelRotation();
        if (mouse) {
            double x = e.getX();
            double y = e.getY();
            if (startedScrolling) {
                resolutionCenterX = reverseConvertX(x);
                resolutionCenterY = reverseConvertY(y);
                pixelShiftX = x - resolutionCenterX;
                pixelShiftY = y - resolutionCenterY;
                startedScrolling = false;
            }
        }
        resolutionScaling = resolutionScaling * Math.pow(1.1, numClicks);
    }

    @Override
    public void keyTyped(KeyEvent e) {

    }

    public void pressKey(char input) {
        if (keyReleased) firstPress = true;
        if (keyPressed != input) firstPress = true;
        keyPressed = input;
        if (!keysCache.contains(keyPressed)) keysCache.add(keyPressed);
        keyReleased = false;
        lastKeyTime = System.currentTimeMillis();
    }
    public void releaseKey(char input) {
        if (keysCache.contains((Character)input)) {
            if (keyPressed != input || !keyReleased || !keyReleasedFirstTime) {
                keyReleasedFirstTime = true;
                keyPressed = input;
                if (!keysCache.contains(keyPressed)) keysCache.add(keyPressed);
                keyReleasedForTheFirstTime = keyPressed;
                keyReleased = true;
            }
        }
    }

    @Override
    public void keyPressed(KeyEvent e) {
        pressKey(e.getKeyChar());
    }

    @Override
    public void keyReleased(KeyEvent e) {
        releaseKey(e.getKeyChar());
    }
}
