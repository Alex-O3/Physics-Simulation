package PhysicsSim.Math;

import java.util.ArrayList;

public class Matrix {
    //matrix.get(i) gets the ith row and matrix.get(i).get(j) gets the (i, j) position
    private final ArrayList<ArrayList<Complex>> matrix = new ArrayList<>();
    private final int numRows;
    private final int numColumns;

    public Matrix(int numRows, int numColumns) {
        if (numRows <= 0 || numColumns <= 0) System.out.println("Matrix must have > 0 num of rows and columns.");
        this.numRows = numRows;
        this.numColumns = numColumns;
        for (int i = 0; i < numRows; i++) {
            ArrayList<Complex> row = new ArrayList<>();
            for (int j = 0; j < numColumns; j++) {
                row.add(new Complex(0.0, 0.0));
            }
            matrix.add(row);
        }
    }

    public Matrix getIdentity() {
        Matrix identity = new Matrix(numRows, numColumns);
        for (int i = 0; i < numRows && i < numColumns; i++) identity.set(i, i, new Complex(1.0, 0.0));
        return identity;
    }
    public void set(int row, int column, Complex input) {
        matrix.get(row).set(column, input);
    }
    public void setRow(int row, ArrayList<Complex> inputRow) {
        for (int j = 0; j < numColumns; j++) set(row, j, inputRow.get(j));
    }
    public void setColumn(int column, ArrayList<Complex> inputColumn) {
        for (int i = 0; i < numRows; i++) set(i, column, inputColumn.get(i));
    }
    public Complex get(int row, int column) {
        return(matrix.get(row).get(column));
    }
    public ArrayList<Complex> getRow(int row) {
        return matrix.get(row);
    }
    public ArrayList<Complex> getColumn(int column) {
        ArrayList<Complex> columnEntry = new ArrayList<>();
        for (int i = 0; i < numRows; i++) {
            columnEntry.add(get(i, column));
        }
        return columnEntry;
    }
    public int getNumRows() {
        return numRows;
    }
    public int getNumColumns() {
        return numColumns;
    }

    public static Matrix multiply(Matrix A, Matrix B) {
        return(Matrix.multiply(A, B, Math.pow(2, -32)));
    }
    public static Matrix multiply(Matrix A, Matrix B, double toZeroMargin) {
        if (A.numColumns != B.numRows) {
            System.out.println("Matrices do not match in dimensions.");
            return(null);
        }
        Matrix results = new Matrix(A.numRows, B.numColumns);
        for (int i = 0; i < A.numRows; i++) {
            for (int j = 0; j < B.numColumns; j++) {
                Complex dotProduct = new Complex(0.0, 0.0);
                for (int k = 0; k < A.numColumns; k++) {
                    dotProduct = Complex.add(dotProduct, Complex.multiply(A.get(i, k), B.get(k, j)));
                }
                if (dotProduct.squareMagnitude() <= toZeroMargin * toZeroMargin) dotProduct = new Complex(0.0, 0.0);
                results.set(i, j, dotProduct);
            }
        }
        return(results);
    }
    public static Matrix add(Matrix A, Matrix B) {
        if (A.numRows != B.numRows || A.numColumns != B.numColumns) {
            System.out.println("Matrix dimensions must equal to add.");
            return null;
        }
        else {
            Matrix sum = new Matrix(A.numRows, A.numColumns);
            for (int i = 0; i < A.numRows; i++) {
                for (int j = 0; j < B.numColumns; j++) {
                    sum.set(i, j, Complex.add(A.get(i, j), B.get(i, j)));
                }
            }
            return sum;
        }
    }
    public static Matrix subtract(Matrix A, Matrix B) {
        if (A.numRows != B.numRows || A.numColumns != B.numColumns) {
            System.out.println("Matrix dimensions must equal to add.");
            return null;
        }
        else {
            Matrix sum = new Matrix(A.numRows, A.numColumns);
            for (int i = 0; i < A.numRows; i++) {
                for (int j = 0; j < B.numColumns; j++) {
                    sum.set(i, j, Complex.subtract(A.get(i, j), B.get(i, j)));
                }
            }
            return sum;
        }
    }

    public static Matrix multiply(Matrix A, Complex num) {
        Matrix result = new Matrix(A.numRows, A.numColumns);
        for (int i = 0; i < result.numRows; i++) {
            for (int j = 0; j < A.numColumns; j++) {
                result.set(i, j, Complex.multiply(A.get(i, j), num));
            }
        }
        return result;
    }

    public Matrix clone() {
        return(subMatrix(0, numColumns, 0, numRows));
    }
    public Matrix subMatrix(int column, int columns, int row, int rows) {
        boolean valid = columns > 0 && column >= 0 && column < numColumns && column + columns <= numColumns;
        valid = valid && (rows > 0 && row >= 0 && row < numRows && row + rows <= numRows);
        if (valid) {
            Matrix result = new Matrix(rows, columns);
            for (int j = column; j < column + columns; j++) {
                for (int i = row; i < row + rows; i++) {
                    result.set(i - row, j - column, get(i, j));
                }
            }
            return(result);
        }
        else {
            System.out.println("Bounds of submatrix must be within bounds of matrix.");
            return null;
        }
    }
    public Matrix minor(int row, int column) {
        if (row >= numRows || column >= numColumns) {
            System.out.println("Minor must be within bounds of matrix.");
            return null;
        }
        Matrix minor = new Matrix(numRows - 1, numColumns - 1);
        int i = 0;
        int j = 0;
        for (int Ai = 0; Ai < numRows; Ai++) {
            if (Ai != row) {
                for (int Aj = 0; Aj < numColumns; Aj++) {
                    if (Aj != column) {
                        minor.set(i, j, get(Ai, Aj));
                        j++;
                    }
                }
                j = 0;
                i++;
            }
        }
        return minor;
    }
    public Matrix insertRowAt(int row, ArrayList<Complex> inputRow) {
        if (row > numRows || inputRow.size() != numColumns) {
            System.out.println("Row insertion must be within bounds of matrix.");
            return null;
        }
        Matrix result = new Matrix(numRows + 1, numColumns);
        int i = 0;
        for (int Ai = 0; Ai < numRows; Ai++) {
            if (i == row) i++;
            result.setRow(i, getRow(Ai));
            i++;
        }
        result.setRow(row, inputRow);
        return result;
    }
    public Matrix insertColumnAt(int column, ArrayList<Complex> inputColumn) {
        if (column > numColumns || inputColumn.size() != numRows) {
            System.out.println("Column insertion must be within bounds of matrix.");
            return null;
        }
        Matrix result = new Matrix(numRows, numColumns + 1);
        int j = 0;
        for (int Aj = 0; Aj < numColumns; Aj++) {
            if (j == column) j++;
            result.setColumn(j, getColumn(Aj));
            j++;
        }
        result.setColumn(column, inputColumn);
        return result;
    }
    public Matrix expand(int row, int column) {
        ArrayList<Complex> zeroVector = new ArrayList<>();
        for (int j = 0; j < numColumns; j++) zeroVector.add(new Complex(0.0, 0.0));
        Matrix result = insertRowAt(row, zeroVector);
        zeroVector.clear();
        for (int i = 0; i < numRows + 1; i++) zeroVector.add(new Complex(0.0, 0.0));
        result = result.insertColumnAt(column, zeroVector);
        return result;
    }
    public void print() {
        for (int i = 0; i < numRows; i++) {
            System.out.print("[");
            for (int j = 0; j < numColumns; j++) {
                System.out.print(matrix.get(i).get(j).print());
                if (j < numColumns - 1) System.out.print(", ");
            }
            System.out.println("]");
        }
    }
    @Override
    public String toString() {
        String results = "";
        for (int i = 0; i < numRows; i++) {
            results += "[";
            for (int j = 0; j < numColumns; j++) {
                results += matrix.get(i).get(j).print();
                if (j < numColumns - 1) results += ", ";
            }
            results += "]\n";
        }
        return(results);
    }
    private boolean isAugmentedOrSquare() {
        return(numColumns >= numRows);
    }

    public Complex determinant() {
        return(determinant(Math.pow(2, -32)));
    }
    public Complex determinant(double toZeroMargin) {
        Matrix calculator = clone();
        int numSwaps = calculator.getUpperTriangular(toZeroMargin);
        return(Complex.multiply(new Complex(numSwaps % 2 == 0 ? 1.0 : -1.0, 0.0), calculator.multiplyDiagonal()));
    }
    private Complex multiplyDiagonal() {
        Complex result = new Complex(1.0, 0.0);
        for (int j = 0; j < numRows; j++) {
            result = Complex.multiply(get(j, j), result);
        }
        return(result);
    }

    //returns the ratio between the determinant of the new matrix and the old one
    public Complex rref(double toZeroMargin) {
        Complex ZERO = new Complex(0.0, 0.0);
        if (!isAugmentedOrSquare()) {
            System.out.println("Matrix must be augmented or square for 'rref'");
            return(ZERO);
        }
        int numSwaps = getUpperTriangular(toZeroMargin);
        Complex NEGATIVE = new Complex(-1.0, 0.0);
        for (int j = numRows - 1; j >= 0; j--) {
            Complex divider = get(j,j);
            if (!divider.equals(ZERO, toZeroMargin)) for (int i = j - 1; i >= 0; i--) {
                Complex multiplier = Complex.multiply(Complex.divide(get(i, j), divider), NEGATIVE);
                for (int k = 0; k < numColumns; k++) {
                    Complex result = Complex.add(Complex.multiply(get(j, k), multiplier), get(i, k));
                    if (result.squareMagnitude() <= toZeroMargin * toZeroMargin) result = new Complex(0.0, 0.0);
                    set(i, k, result);
                }
            }
        }
        Complex ratio = new Complex(1.0, 0.0);
        for (int j = 0; j < numRows; j++) {
            Complex divider = get(j, j);
            if (!divider.equals(ZERO, toZeroMargin)) {
                for (int k = 0; k < numColumns; k++) {
                    set(j, k, Complex.divide(get(j, k), divider));
                }
                ratio = Complex.divide(ratio, divider);
            }
        }
        ratio = Complex.multiply(new Complex(numSwaps % 2 == 0 ? 1.0 : -1.0, 0.0), ratio);
        return(ratio);
    }

    //returns the number of row swaps made
    public int getUpperTriangular(double toZeroMargin) {
        if (!isAugmentedOrSquare()) {
            System.out.println("Matrix must be augmented or square for 'getUpperTriangular'");
            return(0);
        }
        Complex NEGATIVE = new Complex(-1.0, 0.0);
        Complex ZERO = new Complex(0.0, 0.0);
        int numSwaps = 0;
        for (int j = 0; j < numRows - 1; j++) {
            Complex divider = get(j,j);
            if (divider.equals(ZERO)) {
                for (int h = j; h < numRows; h++) {
                    if (!get(h, j).equals(ZERO)) {
                        ArrayList<Complex> tempRow = matrix.get(h);
                        matrix.set(h, matrix.get(j));
                        matrix.set(j, tempRow);
                        numSwaps++;
                        break;
                    }
                }
            }
            divider = get(j, j);
            if (!divider.equals(ZERO)) for (int i = numRows - 1; i > j; i--) {
                Complex multiplier = ZERO;
                multiplier = Complex.multiply(Complex.divide(get(i, j), divider), NEGATIVE);
                for (int k = 0; k < numColumns; k++) {
                    Complex result = Complex.add(Complex.multiply(get(j, k), multiplier), get(i, k));
                    if (result.squareMagnitude() <= toZeroMargin * toZeroMargin) result = new Complex(0.0, 0.0);
                    set(i, k, result);
                }
            }
            double a = 1;
        }
        return(numSwaps);
    }

    public Matrix transpose() {
        Matrix transpose = new Matrix(numColumns, numRows);
        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numColumns; j++) {
                transpose.set(j, i, get(i, j));
            }
        }
        return(transpose);
    }
    public Matrix conjugateTranspose() {
        Matrix transpose = new Matrix(numColumns, numRows);
        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numColumns; j++) {
                Complex entry = get(i, j);
                transpose.set(j, i, new Complex(entry.getReal(), -1.0 * entry.getImaginary()));
            }
        }
        return(transpose);
    }
    public Matrix inverseMatrix() {
        return(inverseMatrix(Math.pow(2, -32)));
    }
    public Matrix inverseMatrix(double toZeroMargin) {
        if (numColumns != numRows) {
            System.out.println("Not square matrix: not invertible.");
            return(null);
        }
        Matrix augmented = new Matrix(numRows, 2 * numColumns);
        for (int i = 0; i < numRows; i++) {
            for (int j = 0; j < numColumns; j++) {
                augmented.set(i, j, get(i, j));
            }
        }
        for (int i = 0; i < numRows; i++) {
            augmented.set(i, i + numColumns, new Complex(1.0, 0.0));
        }
        augmented.rref(toZeroMargin);
        Matrix inverse = augmented.subMatrix(numRows, numColumns, 0, numRows);
        return(inverse);
    }

    public Matrix householderReflector() {
        if (numRows != numColumns) {
            System.out.println("Must be square to householder reflect.");
            return null;
        }
        if (numRows <= 1) {
            Matrix result = new Matrix(1,1);
            result.set(0,0, new Complex(1.0, 0.0));
            return result;
        }
        Complex magnitude = new Complex();
        for (int i = 0; i < numRows; i++) {
            magnitude = Complex.add(magnitude, Complex.multiply(get(i, 0), get(i,0)));
        }
        magnitude = Complex.sqrt(magnitude);
        Matrix v = new Matrix(numRows, 1);
        v.set(0, 0, magnitude);
        v = Matrix.subtract(subMatrix(0, 1, 0, numRows), v);

        magnitude = new Complex();
        for (int i = 0; i < numRows; i++) {
            magnitude = Complex.add(magnitude, Complex.multiply(v.get(i, 0), v.get(i, 0)));
        }
        magnitude = Complex.sqrt(magnitude);
        v = Matrix.multiply(v, Complex.divide(new Complex(1.0, 0.0), magnitude));

        Matrix Qk = new Matrix(numRows, numRows);
        for (int i = 0; i < numRows; i++) {
            Qk.set(i, i, new Complex(1.0, 0.0));
        }
        Qk = Matrix.subtract(Qk, Matrix.multiply(Matrix.multiply(v, v.transpose()), new Complex(2.0, 0.0)));

        return Qk;
    }
    public Matrix[] householderQRDecomp() {
        if (numRows != numColumns) {
            System.out.println("Must be square to householder reflect.");
            return null;
        }
        Matrix Q = getIdentity();
        Matrix A = clone();
        for (int i = 0; i < numRows - 1; i++) {
            Matrix Ak = A.subMatrix(i, numRows - i, i, numRows - i);
            Matrix Qk = Ak.householderReflector();
            for (int k = 0; k < i; k++) {
                Qk = Qk.expand(0,0);
                Qk.set(0,0, new Complex(1.0, 0.0));
            }
            Q = Matrix.multiply(Qk, Q);
            A = Matrix.multiply(Qk, A);
        }
        Q = Q.inverseMatrix();
        return new Matrix[]{Q, A};
    }
    public Matrix hessenbergForm() {
        if (!isAugmentedOrSquare()) {
            System.out.println("Must be square to convert to hessenberg form.");
            return null;
        }
        Matrix A = clone();
        for (int i = 0; i < numRows - 2; i++) {
            Matrix Ak = A.subMatrix(i, numRows - i - 1, i + 1, numRows - i - 1);
            Matrix Qk = Ak.householderReflector();
            for (int k = 0; k <= i; k++) {
                Qk = Qk.expand(0,0);
                Qk.set(0,0, new Complex(1.0, 0.0));
            }
            A = Matrix.multiply(Matrix.multiply(Qk, A), Qk.inverseMatrix());
        }
        return A;
    }
    private Matrix QRAlgorithmIteration() {
        Matrix Aold = clone();
        Complex shift = Aold.get(numRows - 1, numColumns - 1);
        for (int i = 0; i < Aold.numRows; i++) {
            Aold.set(i, i, Complex.subtract(Aold.get(i,i), shift));
        }
        Matrix Anew = Aold.clone();
        Complex NEGATIVE = new Complex(-1.0, 0.0);
        for (int i = 0; i < Anew.numRows - 1; i++) {
            Complex s = Complex.sqrt(Complex.add(Complex.multiply(Anew.get(i, i), Anew.get(i, i)), Complex.multiply(Anew.get(i + 1, i), Anew.get(i + 1, i))));
            Complex c = Complex.divide(Anew.get(i, i), s);
            s = Complex.multiply(Complex.divide(Anew.get(i + 1, i), s), NEGATIVE);
            for (int j = 0; j < Anew.numColumns; j++) {
                Complex x = Complex.subtract(Complex.multiply(c, Anew.get(i, j)), Complex.multiply(s, Anew.get(i + 1, j)));
                Complex y = new Complex(0.0, 0.0);
                if (j != i) y = Complex.add(Complex.multiply(s, Anew.get(i, j)), Complex.multiply(c, Anew.get(i + 1, j)));
                Anew.set(i, j, x);
                Anew.set(i + 1, j, y);
            }
        }
        Matrix G = Matrix.multiply(Anew, Aold.inverseMatrix());
        Anew = Matrix.multiply(Anew, G.inverseMatrix());
        for (int i = 0; i < Anew.numRows; i++) {
            Anew.set(i, i, Complex.add(Anew.get(i,i), shift));
        }
        return Anew;
    }
    private static ArrayList<Complex> QRAlgorithmUntilDeflated(Matrix A, int maxIterations, double convergenceMargin) {
        ArrayList<Complex> results = new ArrayList<>();
        for (int i = 0; i < A.numRows; i++) {
            for (int j = 0; j < A.numColumns; j++) {
                if (Double.isNaN(A.get(i, j).getReal()) || Double.isNaN(A.get(i, j).getImaginary())) {
                    System.out.println("Collapsed to NaN. Exiting...");
                    return null;
                }
            }
        }
        if (A.numRows == 1 && A.numColumns == 1) {
            results.add(A.get(0,0));
            return results;
        }
        if (A.numRows == 2 && A.numColumns == 2) {
            Complex a = A.get(0,0);
            Complex b = A.get(0, 1);
            Complex c = A.get(1, 0);
            Complex d = A.get(1, 1);
            Complex sqrt = Complex.sqrt(Complex.subtract(Complex.multiply(Complex.add(a, d), Complex.add(a,d)), Complex.multiply(new Complex(4.0, 0.0), Complex.subtract(Complex.multiply(a,d), Complex.multiply(b,c)))));
            Complex x1 = Complex.divide(Complex.add(Complex.add(a,d), sqrt), new Complex(2.0, 0.0));
            Complex x2 = Complex.divide(Complex.subtract(Complex.add(a,d), sqrt), new Complex(2.0, 0.0));
            results.add(x1);
            results.add(x2);
            return results;
        }
        boolean deflated = false;
        //we perform iterations (with a stopping limit) until the matrix is close to being upper. Each iteration maintains similarity to the initial matrix.
        int[] deflationCoords = new int[]{1,0};
        for (int i = 0; i < maxIterations && !deflated; i++) {
            A = A.QRAlgorithmIteration();
            for (int j = 0; j < A.getNumColumns() - 1; j++) {
                if (A.get(j + 1, j).magnitude() <= convergenceMargin) {
                    deflated = true;
                    deflationCoords = new int[]{j + 1, j};
                    System.out.println("Iterations: " + i);
                    break;
                }
            }
        }
        if (!deflated) {
            Complex min = A.get(1, 0);
            for (int j = 0; j < A.getNumColumns() - 1; j++) {
                if (A.get(j + 1, j).compareTo(min) <= 0.0) {
                    min = A.get(j + 1, j);
                    deflationCoords = new int[]{j + 1, j};
                }
            }
            System.out.println("Iterations: " + maxIterations);
        }
        Matrix leftDeflation = null;
        if (deflationCoords[0] > 0) {
            leftDeflation = A.subMatrix(0, deflationCoords[1] + 1, 0, deflationCoords[0]);
            ArrayList<Complex> result1 = QRAlgorithmUntilDeflated(leftDeflation, maxIterations, convergenceMargin);
            if (result1 == null) return null;
            results.addAll(result1);
        }
        Matrix rightDeflation = null;
        if (deflationCoords[1] < A.numColumns - 1) {
            rightDeflation = A.subMatrix(deflationCoords[1] + 1, A.numColumns - deflationCoords[1] - 1, deflationCoords[0], A.numRows - deflationCoords[0]);
            ArrayList<Complex> result2 = QRAlgorithmUntilDeflated(rightDeflation, maxIterations, convergenceMargin);
            if (result2 == null) return null;
            results.addAll(result2);
        }
        return results;
    }

    /**
     *
     * @param convergenceMargin the requirement of proximity to 0 for entries below the diagonal to be considered upper triangular
     * @param maxIterations the stopping point of iterations
     * @return an (n + 1) x n matrix whose first n x n portion, or first n rows, are the normalized eigenvectors, and the bottommost row is the corresponding eigenvalues.
     */
    public ArrayList<Complex> QRAlgorithm(double convergenceMargin, int maxIterations) {
        Matrix A = clone();
        //we convert A to similar upper hessenberg form to make iterations quicker. Each iteration is programmed with this being a requirement as well.
        A = A.hessenbergForm();
        //now an upper triangular similar matrix, the eigenvalues are listed along the diagonal
        ArrayList<Complex> eigenvalues = QRAlgorithmUntilDeflated(A, maxIterations, convergenceMargin);
        return eigenvalues;
    }

}