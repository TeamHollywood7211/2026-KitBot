@Override
public void periodic() {
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    
    boolean aligned = tv == 1 && Math.abs(tx) < 1.0;
    SmartDashboard.putBoolean("Turret/Aligned", aligned);
}