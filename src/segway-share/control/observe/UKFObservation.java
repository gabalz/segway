package control.observe;

import helper.Ratio;
import helper.CachedODE;
import java.io.IOException;
import vecmat.Matrix;
import vecmat.Vector;
import ode.ODESolver;
import ode.RK4;
import control.RobotController;

/**
 * Observation estimation by the Unscented Kalman Filter (UKF).
 */
public final class UKFObservation extends Observation
{
    public UKFObservation(RobotController controller, int freq,
                          int gyroSign, double gyroOffset,
                          double a1, double a2, double b1,
                          double R, double W, double L, double g, double K,
                          double B, double m, double M, double Jw,
                          double Jpsi, double Jphi, double psi0)
    {
        super (controller, freq, gyroSign, gyroOffset);
        lrOrder = true;
        
        this.a1 = a1; this.a2 = a2; this.b1 = b1;
        this.R = R; this.W = W; this.K = K; this.B = B; this.psi0 = psi0;
        
        MLL = M*L*L;
        MLR = M*L*R;
        MgL = M*g*L;
        H11 = MLL + Jpsi;
        H22 = (2*m + M)*R*R + 2*Jw;
        ht = m*R*R*W*W + W*W*Jw + 2*R*R*Jphi;
        WWB = W*W*B;
        KRW = K*R*W;
        RRMLL2 = 2*R*R*M*L*L;
        
        z = Vector.zero(8);
        ode = new UKFODE(4);
        odeSolver = new RK4(ode, 0.001);
    }
    
    //--------------------------------------------------------------------------
    
    @Override
    public synchronized long time() { return t; }
    
    @Override
    public synchronized double pitch() { return z.get(2); }
    
    @Override
    public synchronized double roll() { return z.get(3); }
    
    @Override
    public synchronized double yaw() { return z.get(4); }
    
    @Override
    public synchronized double dPitch() { return z.get(5); }
    
    @Override
    public synchronized double dRoll() { return z.get(6); }
    
    @Override
    public synchronized double dYaw() { return z.get(7); }
    
    //--------------------------------------------------------------------------
    
    @Override
    public void run()
    {
        int n = 8, m = 3;
        double kappa = 0, alpha = 0.03, beta = 2;
        double lambda = alpha*alpha*(n+kappa)-n;
        double gamma = Math.sqrt(n + lambda);
        
        ZT = Matrix.create(n, 2*n+1);
        YT = Matrix.create(m, 2*n+1);
        // zTh = Vector.create(n);
        y = Vector.create(m);
        
        SigmaZ = Matrix.diag(Vector.create(new double[]{15, 30,
                                                        2, 1, 1,
                                                        15, 15, 15})
                                   .mulL(Ratio.DEG_TO_RAD));
        
        SigmaY = Matrix.diag(Vector.create(new double[]{5, 1, 1})
                                   .mulL(Ratio.DEG_TO_RAD));
        
        wm = Vector.constant(2*n+1, 1.0/(2*(n+lambda)));
        wm.set(0, lambda/(n+lambda));
        
        wc = Vector.constant(2*n+1, 1.0/(2*(n+lambda)));
        wc.set(0, lambda/(n+lambda) + 1-alpha*alpha+beta);
        
        z = Vector.zero(n);
        Pzz = SigmaZ;
        SigmaZ = SigmaZ.mul(delay()/1000.0);
        t = currentTimeMillis();
        
        try { log(); }
        catch (IOException e) { e.printStackTrace(System.err); }
        
        double v1, v2;
        while (true)
        {
            msDelay(delay());
            
            prevT = t;
            synchronized (this)
            {                
                // observe t, y
                t = currentTimeMillis();
                observe(y);

                // dt
                double dt = ((double)(t - prevT)) / 1000.0; // sec
                prevT = t;
                //System.out.println("dt = " + dt);
                
                // observe v1, v2
                synchronized (controller())
                {
                    v1 = controller().leftPower() / 1000.0; // volt
                    v2 = controller().rightPower() / 1000.0; // volt
                }
                
                // ZT, YT
                Matrix C = Pzz.choleskyL().mul(gamma);
                //System.out.println("sum(w) = " + w.iprod(Vector.one(2*n+1)));
                //System.out.println("z!! : " + z);
                ZT.setCol(0, fG(z, dt, v1, v2));
                for (int i = 0; i < n; ++i)
                {
                    ZT.setCol(i+1, fG(z.add(C.getCol(i)), dt, v1, v2));
                    ZT.setCol(n+i+1, fG(z.sub(C.getCol(i)), dt, v1, v2));
                }
                
                // zTh, PTzz
                zTh = ZT.mul(wm);
                for (int i = 0; i < ZT.rows(); ++i)
                    for (int j = 0; j < ZT.cols(); ++j)
                        ZT.set(i, j, ZT.get(i, j) - zTh.get(i));
                //System.out.println("zTh: " + zTh);
                PTzz = ZT.mulD(wc).mul(ZT.T()).add(SigmaZ);
                //Matrix ZTsave = ZT.copy();
                
                C = PTzz.choleskyL().mul(gamma);
                ZT.setCol(0, zTh);
                for (int i = 0; i < n; ++i)
                {
                    ZT.setCol(i+1, zTh.add(C.getCol(i)));
                    ZT.setCol(n+i+1, zTh.sub(C.getCol(i)));
                }
                for (int i = 0; i < ZT.cols(); ++i)
                    YT.setCol(i, h(ZT.getCol(i)));
                
                // yTh, PTyy, PTzy
                yTh = YT.mul(wm);
                for (int i = 0; i < YT.rows(); ++i)
                    for (int j = 0; j < YT.cols(); ++j)
                        YT.set(i, j, YT.get(i, j) - yTh.get(i));
                PTyy = YT.mulD(wc).mul(YT.T()).add(SigmaY);
                
                for (int i = 0; i < ZT.rows(); ++i)
                    for (int j = 0; j < ZT.cols(); ++j)
                        ZT.set(i, j, ZT.get(i, j) - zTh.get(i));
                PTzy = ZT.mulD(wc).mul(YT.T());

                System.out.println("TPzz.normf = " + PTzz.norm1()/PTzz.cols());

                // estimate
                Matrix PTyyInv = PTyy.inv3x3L();
                //System.out.println("PTyyInv.normf = " + PTyyInv.normf());
                z = zTh.add(PTzy.mul(PTyyInv).mul(y.sub(yTh)));
                Pzz = PTzz.sub(PTzy.mul(PTyyInv).mul(PTzy.T()));
            }
            
            System.out.println("y   : " + y);
            //System.out.println("v   : " + v1 + " , " + v2);
            //System.out.println("yTh : " + yTh + " , gyroOffset: " + gyroOffset());
            //System.out.println("z   : " + z);
            //System.out.println("zDeg: " + z.mul(Ratio.RAD_TO_DEG));
            System.out.println("? : " + h(z));
            //System.out.println("s : " + robot().simDynState());
            //robot().simDynState().print(System.out);
            //System.out.println("dYaw : " + robot().simDynState().dYaw() * Ratio.RAD_TO_DEG);
            //System.out.println();
            //if (--ccc <= 0) System.exit(-5);
            
            try { log(); }
            catch (IOException e) { e.printStackTrace(System.err); }
        }
    }
    int ccc = 2;
    
    //--------------------------------------------------------------------------
    
    private void observe(Vector y)
    {
        double lRotCtr, rRotCtr;
        double gyro = robot().readGyro();
        if (lrOrder)
        {
            lRotCtr = robot().leftRotationCounter();
            rRotCtr = robot().rightRotationCounter();
        }
        else
        {
            rRotCtr = robot().rightRotationCounter();
            lRotCtr = robot().leftRotationCounter();
        }
        lrOrder = !lrOrder;
        
        y.set(0, gyroSign()*gyro*Ratio.DEG_TO_RAD - gyroOffset());
        y.set(1, (lRotCtr + rRotCtr)*Ratio.DEG_TO_RAD/2.0);
        y.set(2, (rRotCtr - lRotCtr)*Ratio.DEG_TO_RAD*R/W);
    }
    
    private Vector fG(Vector z, double dt, double vl, double vr)
    {
        ode.vl = vl;
        ode.vr = vr;
        
        for (double t = 0; t < dt; t += odeSolver.dt())
            odeSolver.next(0.0, z.copy(), z);
        //System.out.println("z[1:2]: " + z.get(0) + ", " + z.get(1));
        
        return z;
    }
    
    private Vector h(Vector z)
    {
        Vector result = Vector.create(3);
        result.set(0, z.get(0));
        result.set(1, z.get(3) - z.get(2) + psi0);
        result.set(2, z.get(4));
        return result;
    }
    
    //--------------------------------------------------------------------------
    
    private final class UKFODE extends CachedODE
    {
        public UKFODE(int cacheSize)
        {
            super (cacheSize, 8);
        }
        
        @Override
        public Vector f(double t, Vector x)
        {
            double gyro = z.get(0);
            double dGyro = z.get(1);
            double pitch = z.get(2) - psi0;
            double dPitch = z.get(5);
            double dRoll = z.get(6);
            double dYaw = z.get(7);
            
            double sinPitch = Math.sin(pitch);
            double cosPitch = Math.cos(pitch);
            
            double diffDRollDPitch = dRoll - dPitch;
            double H12 = MLR*cosPitch;
            double h = RRMLL2*sinPitch*sinPitch + ht;
            
            double fh4 = MLL*dYaw*dYaw*sinPitch*cosPitch
                       + MgL*sinPitch
                       + 2*B*diffDRollDPitch
                 , fh5 = MLR*dPitch*dPitch*sinPitch
                       - 2*B*diffDRollDPitch
                 , fh6 = -RRMLL2*dYaw*dPitch*sinPitch*cosPitch
                       - WWB*dYaw;
            
            double Hdiv = H11*H22 - H12*H12;
            double f4 = (H22*fh4 - H12*fh5)/Hdiv;
            double f5 = (H11*fh5 - H12*fh4)/Hdiv;
            double f6 = fh6/h;
            
            double g4 = -K*(H12 + H22)/Hdiv;
            double g5 = K*(H11 + H12)/Hdiv;
            double g6 = KRW/h;
            
            Vector result = nextCachedVector();
            result.set(0, dGyro);
            result.set(1, b1*dPitch-a1*dGyro-a2*gyro);
            result.set(2, dPitch);
            result.set(3, dRoll);
            result.set(4, dYaw);
            result.set(5, f4 + g4*(vl+vr));
            result.set(6, f5 + g5*(vl+vr));
            result.set(7, f6 + g6*(vr-vl));
            return result;
        }
        
        private double vl, vr;
    }
    
    //--------------------------------------------------------------------------

    private boolean lrOrder;
    private long t, prevT;
    
    // dynamics parameters
    private final double R, W, K, B, psi0;
    private final double H11, H22, ht, MLL, MLR, MgL, WWB, KRW, RRMLL2;

    // rate gyroscope parameters
    private final double a1, a2, b1;
    
    private Vector z, zTh, y, yTh, wm, wc;
    private Matrix ZT, YT, Pzz, PTzz, PTzy, PTyy, SigmaZ, SigmaY;
    
    private UKFODE ode;
    private ODESolver odeSolver;
}
