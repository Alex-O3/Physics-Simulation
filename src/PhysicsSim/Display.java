package PhysicsSim;
import java.awt.event.*;
import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

class Display extends JPanel implements MouseListener, MouseMotionListener, MouseWheelListener, KeyListener {
    public final int simID;
    public int debugVector = 0;
    public double debugVectorScaling = 0.001;
    public boolean debugBounds = false;
    public boolean debugRadius = false;
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
        boolean results = false;
        if (timeElapsed / 1000.0 > dt) {
            results = Rigidbody.moveByMouse(mouseX, mouseY, true, mousePressed, dt, simID);
            vmXDisplay = 0.0;
            vmYDisplay = 0.0;
        }
        else {
            results = Rigidbody.moveByMouse(mouseX, mouseY, false, mousePressed, dt, simID);
        }
        isInside = results;
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
        try {
            if (Simulation.get(simID).physicsObjects.isEmpty()) return;
            super.paintComponent(g);
            for (int i = 0; i < Rigidbody.num; i++) {
                Rigidbody rigidbody = Rigidbody.get(i);
                if (rigidbody == null || !rigidbody.draw) continue;
                Material material = Simulation.get(simID).getObject("Rigidbody", i).material;
                if (material != null && material.name.contains("Player")) {
                    playerLock = true;
                    resolutionCenterX = rigidbody.getPosX();
                    resolutionCenterY = rigidbody.getPosY();
                    pixelShiftX = 0.5 * getWidth() - ((rigidbody.getPosX() - resolutionCenterX) / resolutionScaling) - resolutionCenterX;
                    pixelShiftY = 0.5 * getHeight() - ((rigidbody.getPosY() - resolutionCenterY) / resolutionScaling) - resolutionCenterY;
                }
                Triplet drawInformation = rigidbody.getDraw(shiftX, resolutionCenterX, pixelShiftX,
                        shiftY, resolutionCenterY, pixelShiftY, resolutionScaling);
                if (rigidbody.simID != simID || !rigidbody.draw) continue;
                g.setColor(rigidbody.geometry.getColor());
                switch (drawInformation.getFirstRigidbodyGeometries()) {
                    case Polygon: {
                        int[] x = drawInformation.getSecondIntArray();
                        int[] y = drawInformation.getThirdIntArray();
                        int length = x.length;
                        g.fillPolygon(x, y, length);
                        break;
                    }
                    case Circle: {
                        int radius = (int) (rigidbody.geometry.getLargestDistance() / resolutionScaling);
                        g.fillOval(drawInformation.getSecondIntArray()[0] - radius, drawInformation.getSecondIntArray()[1] - radius, 2 * radius, 2 * radius);
                    }
                }
                for (Joint joint : rigidbody.attachments) {
                    g.drawLine(convertX(rigidbody.getPosX() + joint.offsetFromCMParent[0]), convertY(rigidbody.getPosY() + joint.offsetFromCMParent[1]),
                            convertX(joint.connection.getPosX() + joint.offsetFromCMOther[0]), convertY(joint.connection.getPosY() + joint.offsetFromCMOther[1]));
                }

                //draw center of mass unless that drawing would be larger than the shape
                double largestDistance = rigidbody.geometry.getLargestDistance();
                if (largestDistance / resolutionScaling > 15.0 && rigidbody.isMovable()) {
                    g.setColor(Color.red);
                    g.fillOval(convertX(rigidbody.getPosX()) - 5, convertY(rigidbody.getPosY()) - 5, 10, 10);
                }

                //draw debug velocity and acceleration vectors
                if (debugVector == 1) {
                    g.setColor(Color.red);
                    g.drawLine(convertX(rigidbody.getPosX()), convertY(rigidbody.getPosY()),
                            convertX(rigidbody.getPosX() + rigidbody.getVX() * debugVectorScaling),
                            convertY(rigidbody.getPosY() + rigidbody.getVY() * debugVectorScaling));
                } else if (debugVector == 2) {
                    g.setColor(Color.blue);
                    g.drawLine(convertX(rigidbody.getPosX()), convertY(rigidbody.getPosY()),
                            convertX(rigidbody.getPosX() + rigidbody.getAX() * debugVectorScaling),
                            convertY(rigidbody.getPosY() + rigidbody.getAY() * debugVectorScaling));
                }
                g.setColor(Color.red);
                if (debugBounds) g.drawRect(convertX(rigidbody.geometry.leftBoundBox + rigidbody.getPosX()),
                        convertY(rigidbody.geometry.topBoundBox + rigidbody.getPosY()),
                        (int) ((rigidbody.geometry.rightBoundBox - rigidbody.geometry.leftBoundBox) / resolutionScaling),
                        (int) ((rigidbody.geometry.bottomBoundBox - rigidbody.geometry.topBoundBox) / resolutionScaling));
                g.setColor(Color.red);
                if (debugRadius) g.drawOval(convertX(rigidbody.getPosX() - rigidbody.geometry.getLargestDistance()),
                        convertY(rigidbody.getPosY() - rigidbody.geometry.getLargestDistance()),
                        (int) (2.0 * rigidbody.geometry.getLargestDistance() / resolutionScaling),
                        (int) (2.0 * rigidbody.geometry.getLargestDistance() / resolutionScaling));
            }

            if (debugBounds || debugRadius) for (int i = 0; i < Softbody.num; i++) {
                Softbody softbody = Softbody.get(i);
                g.setColor(Color.red);
                if (debugBounds) g.drawRect(convertX(softbody.minX), convertY(softbody.minY),
                        (int) ((softbody.maxX - softbody.minX) / resolutionScaling),
                        (int) ((softbody.maxY - softbody.minY) / resolutionScaling));
                if (debugRadius) {
                    double repulse_radius_multiplier = Math.sqrt(Simulation.get(simID).REPULSE_RADIUS_MULTIPLIER);
                    g.drawOval(convertX(softbody.cM[0] - softbody.getRadius()),
                            convertY(softbody.cM[1] - softbody.getRadius()),
                            (int) (2.0 * softbody.getRadius() / resolutionScaling),
                            (int) (2.0 * softbody.getRadius() / resolutionScaling));
                    g.setColor(Color.cyan);
                    g.drawOval(convertX(softbody.cM[0] - repulse_radius_multiplier * softbody.getRadius()),
                            convertY(softbody.cM[1] - repulse_radius_multiplier * softbody.getRadius()),
                            (int) (2.0 * repulse_radius_multiplier * softbody.getRadius() / resolutionScaling),
                            (int) (2.0 * repulse_radius_multiplier * softbody.getRadius() / resolutionScaling));

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
        catch (Exception e) {
            System.out.println(e);
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
