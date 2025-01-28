package CustomTypes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class PID {
    public double p,i,d,kff,iz;
    
    public PID(double p, double i, double d, double kff) {this.p=p; this.i=i; this.d=d; this.kff=kff;}
    public PID(double p, double i, double d, double kff, double iz) {this.p=p; this.i=i; this.d=d; this.kff=kff; this.iz=iz;}
    public PID() {this(0,0,0,0);}

    public PID(double p, double i, double d) {this(p,i,d,0);}

    public PID(PID cpy) {this(cpy.p, cpy.i, cpy.d);}

    public void setSparkMaxPID(SparkMax spm, ResetMode rm, PersistMode pm)
    {
        ClosedLoopConfig clc;
        SparkMaxConfig smc;
        clc = new ClosedLoopConfig();
        smc = new SparkMaxConfig();
        clc.pidf(p,i,d,kff);
        clc.iZone(iz);
        smc.apply(clc);
        spm.configure(smc, rm, pm);
    }
}

