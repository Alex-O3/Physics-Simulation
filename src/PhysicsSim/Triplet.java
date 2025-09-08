package PhysicsSim;
class Triplet {
    private boolean firstBoolean;
    private double[] firstDoubleArray;
    private String firstString;
    private int firstInt;
    private int first = 0;
    private boolean secondBoolean;
    private double[] secondDoubleArray;
    private String secondString;
    private int secondInt;
    private int second = 0;
    private boolean thirdBoolean;
    private double[] thirdDoubleArray;
    private String thirdString;
    private int thirdInt;
    private int third = 0;
    public Triplet(double[] a, double[] b) {
        first = 2;
        firstDoubleArray = a;
        second = 2;
        secondDoubleArray = b;
    }
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
    public Triplet(String a, int b, int c) {
        first = a.length();
        firstString = a;
        second = 1;
        secondInt = b;
        third = 1;
        thirdInt = c;
    }

    public Triplet(boolean a, double[] b, int c) {
        first = 1;
        firstBoolean = a;
        second = 2;
        secondDoubleArray = b;
        third = 1;
        thirdInt = c;
    }

    public boolean getFirstBoolean() {
        return(firstBoolean);
    }
    public double[] getFirstDoubleArray() {
        return(firstDoubleArray);
    }
    public Double[] getFirstDoubleArrayReference() {
        return(new Double[]{firstDoubleArray[0], firstDoubleArray[1]});
    }
    public String getFirstString() {
        return(firstString);
    }
    public int getFirstInt() {
        return(firstInt);
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
    public String getSecondString() {
        return(secondString);
    }
    public int getSecondInt() {
        return(secondInt);
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
    public int getThirdInt() {
        return(thirdInt);
    }
}
