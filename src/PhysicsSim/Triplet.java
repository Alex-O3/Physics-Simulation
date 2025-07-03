package PhysicsSim;
class Triplet {
    private boolean firstBoolean;
    private double[] firstDoubleArray;
    private int first = 0;
    private boolean secondBoolean;
    private double[] secondDoubleArray;
    private int second = 0;
    private boolean thirdBoolean;
    private double[] thirdDoubleArray;
    private int third = 0;
    public Triplet(boolean a, double[] b, double[] c) {
        first = 1;
        firstBoolean = a;
        second = 2;
        secondDoubleArray = b;
        third = 2;
        thirdDoubleArray = c;
    }
    public Triplet(boolean a, double[] b) {
        first = 1;
        firstBoolean = a;
        second = 2;
        secondDoubleArray = b;
    }
    public Triplet(boolean a, double[] b, boolean c) {
        first = 1;
        firstBoolean = a;
        second = 2;
        secondDoubleArray = b;
        third = 1;
        thirdBoolean = c;
    }
    public boolean getFirstBoolean() {
        return(firstBoolean);
    }
    public double[] getFirstDoubleArray() {
        return(firstDoubleArray);
    }
    public boolean getSecondBoolean() {
        return(secondBoolean);
    }
    public double[] getSecondDoubleArray() {
        return(secondDoubleArray);
    }
    public Double[] getSecondDoubleArrayReference() {
        return(new Double[]{secondDoubleArray[0], secondDoubleArray[1]});
    }
    public boolean getThirdBoolean() {
        return(thirdBoolean);
    }
    public double[] getThirdDoubleArray() {
        return(thirdDoubleArray);
    }
    public Double[] getThirdDoubleArrayReference() {
        return (new Double[]{thirdDoubleArray[0], thirdDoubleArray[1]});
    }
}
