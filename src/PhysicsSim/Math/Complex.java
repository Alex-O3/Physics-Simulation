package PhysicsSim.Math;

public class Complex {
    private Double real;
    private Double imaginary;
    Complex (double real, double imaginary) {
        this.real = real;
        this.imaginary = imaginary;
    }
    Complex() {
        this.real = 0.0;
        this.imaginary = 0.0;
    }
    public static Complex valueOf (double x) {
        return(new Complex(x, 0.0));
    }
    public static Complex valueOf (double a, double b) {
        return(new Complex(a,b));
    }
    private void setReal(double real) {
        this.real = real;
    }
    private void setImaginary(double imaginary) {
        this.imaginary = imaginary;
    }
    private void set(double real, double imaginary) {
        this.real = real;
        this.imaginary = imaginary;
    }
    private void set(Complex input) {
        real = input.getReal();
        imaginary = input.getImaginary();
    }
    public double getReal() {
        return(real);
    }
    public double getImaginary() {
        return(imaginary);
    }
    public double squareMagnitude() {
        return(real * real + imaginary * imaginary);
    }
    public double magnitude() {
        return(Math.sqrt(squareMagnitude()));
    }
    public Complex conjugate() {
        return(new Complex(real, -imaginary));
    }
    public boolean equals(Object o) {
        if (o instanceof Complex) {
            return(equals((Complex) o));
        }
        else return(false);
    }
    public boolean equals(Complex input) {
        if (real == input.getReal() && imaginary == input.getImaginary()) {
            return(true);
        }
        else {
            return(false);
        }
    }
    public boolean equals(Complex input, double toZeroMargin) {
        return (Complex.subtract(this, input).squareMagnitude() <= toZeroMargin * toZeroMargin);
    }
    public double compareTo(Complex input) {
        return this.magnitude() - input.magnitude();
    }
    public String toString() {
        return(print());
    }
    public static Complex parseComplex (String s) {
        StringBuilder num = new StringBuilder();
        if (!s.endsWith("i")) s = s + ";";
        double realNum = 0.0;
        double imaginaryNum = 0.0;
        boolean negative = false;
        for (char ch : s.toCharArray()) {
            if (ch == '0' || ch == '1' || ch == '2' || ch == '3' || ch == '4' || ch == '5' || ch == '6' || ch == '7' || ch == '8' || ch == '9' || ch == '.') {
                num.append(ch);
            }
            else if (!num.isEmpty()) {
                try {
                    if (ch != 'i') {
                        if (negative) realNum += Double.parseDouble("-" + new String(num));
                        else realNum += Double.parseDouble(new String(num));
                        num.delete(0, num.length());
                    } else {
                        if (negative) imaginaryNum += Double.parseDouble("-" + new String(num));
                        else imaginaryNum += Double.parseDouble(new String(num));
                    }
                }
                catch (Exception e) {
                    //no action needed
                }
                negative = false;
            }
            if (ch == '-') negative = true;
        }
        return new Complex(realNum, imaginaryNum);
    }
    public boolean isEmpty() {
        if (real == 0.0 && imaginary == 0.0) {
            return(true);
        }
        else {
            return(false);
        }
    }
    public String print() {
        if (imaginary == 0.0 && real == 0.0) {
            return("0");
        }
        else if (imaginary == 0.0) {
            return(String.valueOf(real));
        }
        else if (real == 0.0) {
            return(imaginary + "i");
        }
        else {
            return("(" + real + " + " + imaginary + "i)");
        }

    }

    public static Complex add(Complex input1, Complex input2) {
        return(new Complex(input1.getReal() + input2.getReal(), input1.getImaginary() + input2.getImaginary()));
    }
    public static Complex subtract(Complex input1, Complex input2) {
        return(new Complex(input1.getReal() - input2.getReal(), input1.getImaginary() - input2.getImaginary()));
    }
    public static Complex multiply(Complex input1, Complex input2) {
        Complex result = new Complex();
        result.setReal(input1.getReal() * input2.getReal() - input1.getImaginary() * input2.getImaginary());
        result.setImaginary(input1.getReal() * input2.getImaginary() + input2.getReal() * input1.getImaginary());
        return(result);
    }
    public static Complex divide(Complex input1, Complex input2) {
        Complex result = new Complex();
        Double magnitude = input2.squareMagnitude();
        result.set(Complex.multiply(input1, input2.conjugate()));
        result.set(result.getReal() / magnitude, result.getImaginary() / magnitude);
        return(result);
    }
    public static Complex sqrt(Complex input) {
        double c = Math.sqrt(input.getReal() * input.getReal() + input.getImaginary() * input.getImaginary());
        double d = Math.sqrt((-input.getReal() + c) / 2.0);
        if (input.getImaginary() < 0.0) d = -d;
        c = Math.sqrt((input.getReal() + c) / 2.0);
        return new Complex(c, d);
    }
}
