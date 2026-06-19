package PhysicsSim;
class Triplet {
    private boolean firstBoolean;
    private double[] firstDoubleArray;
    private String firstString;
    private int firstInt;
    private int[] firstIntArray;
    private RigidbodyGeometries firstRigidbodyGeometries;
    private double firstDouble;
    private boolean secondBoolean;
    private double[] secondDoubleArray;
    private String secondString;
    private int secondInt;
    private int[] secondIntArray;
    private RigidbodyGeometries secondRigidbodyGeometries;
    private double secondDouble;
    private boolean thirdBoolean;
    private double[] thirdDoubleArray;
    private String thirdString;
    private int thirdInt;
    private int[] thirdIntArray;
    private RigidbodyGeometries thirdRigidbodyGeometries;
    private double thirdDouble;
    private Rigidbody body1;
    private Rigidbody body2;
    public Triplet(RigidbodyGeometries a, int[] b, int[] c) {
        firstRigidbodyGeometries = a;
        secondIntArray = b;
        thirdIntArray = c;
    }
    public Triplet(RigidbodyGeometries a, double[] b, double[] c) {
        firstRigidbodyGeometries = a;
        secondDoubleArray = b;
        thirdDoubleArray = c;
    }
    public Triplet(RigidbodyGeometries a, int[] b, double c) {
        firstRigidbodyGeometries = a;
        secondIntArray = b;
        thirdDouble = c;
    }
    public Triplet(RigidbodyGeometries a, double[] b, double c) {
        firstRigidbodyGeometries = a;
        secondDoubleArray = b;
        thirdDouble = c;
    }
    public Triplet(double[] a, double[] b) {
        firstDoubleArray = a;
        secondDoubleArray = b;
    }
    public Triplet(double[] a, double[] b, double c) {
        firstDoubleArray = a;
        secondDoubleArray = b;
        thirdDouble = c;
    }
    public Triplet(double[] a, double[] b, double[] c) {
        firstDoubleArray = a;
        secondDoubleArray = b;
        thirdDoubleArray = c;
    }
    public Triplet(double[] a, double[] b, double[] c, Rigidbody joint1, Rigidbody joint2) {
        firstDoubleArray = a;
        secondDoubleArray = b;
        thirdDoubleArray = c;
        body1 = joint1;
        body2 = joint2;
    }
    public Triplet(int a, double b, double c) {
        firstInt = a;
        secondDouble = b;
        thirdDouble = c;
    }
    public Triplet(boolean a, double[] b, double[] c) {
        firstBoolean = a;
        secondDoubleArray = b;
        thirdDoubleArray = c;
    }
    public Triplet(boolean a, double[] b) {
        firstBoolean = a;
        secondDoubleArray = b;
    }
    public Triplet(boolean a, double[] b, boolean c) {
        firstBoolean = a;
        secondDoubleArray = b;
        thirdBoolean = c;
    }
    public Triplet(String a, int b, int c) {
        firstString = a;
        secondInt = b;
        thirdInt = c;
    }

    public Triplet(boolean a, double[] b, int c) {
        firstBoolean = a;
        secondDoubleArray = b;
        thirdInt = c;
    }

    public Rigidbody getJoint1() {
        return body1;
    }
    public Rigidbody getJoint2() {
        return body2;
    }

    public boolean getFirstBoolean() {
        return(firstBoolean);
    }
    public double[] getFirstDoubleArray() {
        return(firstDoubleArray);
    }
    public Double[] getFirstDoubleArrayReference() {
        return toReferenceType(firstDoubleArray);
    }
    public String getFirstString() {
        return(firstString);
    }
    public int getFirstInt() {
        return(firstInt);
    }
    public RigidbodyGeometries getFirstRigidbodyGeometries() {return(firstRigidbodyGeometries);}
    public boolean getSecondBoolean() {
        return(secondBoolean);
    }
    public double[] getSecondDoubleArray() {
        return(secondDoubleArray);
    }
    public Double[] getSecondDoubleArrayReference() {
        return toReferenceType(secondDoubleArray);
    }
    public String getSecondString() {
        return(secondString);
    }
    public int getSecondInt() {
        return(secondInt);
    }
    public int[] getSecondIntArray() {return(secondIntArray);}
    public double getSecondDouble() {return(secondDouble);}
    public boolean getThirdBoolean() {
        return(thirdBoolean);
    }
    public double[] getThirdDoubleArray() {
        return(thirdDoubleArray);
    }
    public Double[] getThirdDoubleArrayReference() {
        return toReferenceType(thirdDoubleArray);
    }
    public int getThirdInt() {
        return(thirdInt);
    }
    public int[] getThirdIntArray() {return(thirdIntArray);}
    public double getThirdDouble() {return(thirdDouble);}

    private Double[] toReferenceType(double[] arr) {
        Double[] result = new Double[arr.length];
        for (int i = 0; i < arr.length; i++) {
            result[i] = arr[i];
        }
        return result;
    }
}
