@Override
public void execute() {
    turret.alignToTarget();

    // Get distance from Limelight (Assuming you set up 'ty' calculation or 3D pose)
    // For this example, we fetch the 'ty' value from NetworkTables
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    
    // Convert ty to distance (Simple example, replace with your calibrated math)
    double distance = calculateDistance(ty); 
    double dynamicRPM = shooter.getRPMForDistance(distance);

    shooter.setShooterVelocity(dynamicRPM);

    if (turret.isAligned() && shooter.isAtVelocity() && !shooter.isHopperEmpty()) {
        shooter.runFeeder(1.0);
    } else {
        shooter.runFeeder(0.0);
    }
}

// Calibration: Distance = (TargetHeight - CameraHeight) / tan(CameraAngle + ty)
private double calculateDistance(double ty) {
    double h1 = 0.5; // Camera height meters
    double h2 = 2.0; // Target height meters
    double a1 = Math.toRadians(25); // Camera angle degrees
    double a2 = Math.toRadians(ty);
    return (h2 - h1) / Math.tan(a1 + a2);
}