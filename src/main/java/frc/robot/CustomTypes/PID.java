package frc.robot.CustomTypes;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class PID {
    public double p,i,d,kff,iz;
    public PID(double p, double i, double d, double kff) {this.p=p; this.i=i; this.d=d; this.kff=kff;this.iz=0;}
    public PID(double p, double i, double d, double kff, double iz) {this.p=p; this.i=i; this.d=d; this.kff=kff; this.iz=iz;}
    public PID() {this(0,0,0,0);}
    public PID(double p, double i, double d) {this(p,i,d,0);}
    public PID(PID cpy) {this(cpy.p, cpy.i, cpy.d);}

    public final IdleMode _imode_default = IdleMode.kBrake;
    public final PersistMode  _pmode_default = PersistMode.kNoPersistParameters;
    public final ResetMode _rmode_default = ResetMode.kResetSafeParameters;
    private IdleMode _imode;
    private PersistMode _pmode;
    private ResetMode _rmode;
    public SparkMax setSparkMaxPID(SparkMax spm, ResetMode rm, PersistMode pm)
    {
      this._rmode=rm;
      this._pmode=pm;
      return setPIDBase(spm);
    }
    public SparkMaxConfig setSparkMaxPID(SparkMax spm) {return setPIDBase(spm);}
    public SparkMaxConfig setSparkMaxPID(SparkMax spm, IdleMode im) {
      this._imode = im;
      return setPIDBase(spm);
    }
    public SparkMaxConfig setSparkMaxPID(SparkMax spm, ResetMode rm, PersistMode pm,IdleMode im)
    {
      this._imode=im;
      this._pmode=pm;
      this._rmode=rm;
      return setPIDBase(spm);
    }
    private SparkMaxConfig setPIDBase(SparkMax sp) {
      ClosedLoopConfig c = new ClosedLoopConfig();
      c.pidf(p,i,d,kff);
      c.iZone(iz);
      SparkMaxConfig sc = new SparkMaxConfig();
      sc.apply(c);
      sc.idleMode( (_imode!= null)?(_imode):(_imode_default) );
      sp.configure(sc, (_rmode != null)?(_rmode):(_rmode_default), (_pmode!= null)?(_pmode):(_pmode_default));
      return sc;
    }
}

