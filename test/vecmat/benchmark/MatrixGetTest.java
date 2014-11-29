package vecmat.benchmark;

import vecmat.Matrix;

public class MatrixGetTest
{
    public static void main(String[] args)
    {
        int K = 10000000;
        
        int N = 10;
        int M = 10;
        
        double[][] d1 = new double[N][M];
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                d1[i][j] = (i * M + j) / 2.0;
        
        double[][] d2 = new double[N][M];
        for (int i = 0; i < N; ++i)
            for (int j = 0; j < M; ++j)
                d2[i][j] = (i * M + j) / 5.0;
        
        Matrix m1 = Matrix.create(d1);
        Matrix m2 = Matrix.create(d2);
        
        long start = System.currentTimeMillis();
        for (int k = 0; k < K; ++k)
            m1.addL(m2);
        long elapsed = System.currentTimeMillis() - start;
        
        System.out.println("elapsed : " + elapsed);
    }
}
