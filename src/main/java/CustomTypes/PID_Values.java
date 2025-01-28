package CustomTypes;

public class PID_Values {

    public PID_Values(double p, double i, double d, double kff) {this.p=p; this.i=i; this.d=d; this.kff=kff;}
    public PID_Values(double p, double i, double d, double kff, double iz) {this.p=p; this.i=i; this.d=d; this.kff=kff; this.iz=iz;}
    public PID_Values() {this(0,0,0,0);}

    public PID_Values(double p, double i, double d) {this(p,i,d,0);}

    public PID_Values(PID cpy) {this(cpy.p, cpy.i, cpy.d);}

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

