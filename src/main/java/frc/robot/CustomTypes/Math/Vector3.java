package frc.robot.CustomTypes.Math;

public class Vector3 {
    public double x, y, z;

    public static Vector3 zero = new Vector3(0, 0, 0);

    // Vector3's should be declarative and only have the values you set to them
    public Vector3(double x, double y, double z)
    {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public static double distance(Vector3 a, Vector3 b)
    {
        return Math.sqrt(((b.x - a.x) * (b.x - a.x)) + ((b.y - a.y) * (b.y - a.y)) + ((b.z - a.z) * (b.z - a.z)));
    }

    public double magnitude()
    {
        return distance(this, zero);
    }

    public Vector3 normalized()
    {
        return this.divideBy(this.magnitude());
    }

    public static Vector3 lerp(Vector3 a, Vector3 b, double percent)
    {
        Vector3 relativePos = b.substract(a);

        return a.add(relativePos.times(percent));
    }

    public static Vector3 clampMagnitude(Vector3 vector3, double max)
    {
        Vector3 out = vector3;
        if (vector3.magnitude() > max) { out = vector3.normalized().times((max));}

        return out;
    }

    public Vector3 times(double b)
    {
        return new Vector3(x * b, y * b, z * b);
    }

    public Vector3 add(Vector3 b)
    {
        return new Vector3(x + b.x, y + b.y, z + b.z);
    }

    public Vector3 substract(Vector3 a)
    {
        return new Vector3(x - a.x, y - a.y, z - a.z);
    }

    public Vector3 divideBy(double b)
    {
        return new Vector3(x / b, y / b, z / b);
    }

    @Override
    public String toString() {
        return "(" + x + ", " + y + ", " + z + ")";
    }

    public String toString(int decimals) {
        double mult = Math.pow(10, decimals);
        return "(" + (int)(x * mult) / mult + ", " + (int)(y * mult) / mult + ", " + (int)(z * mult) / mult + ")";
    }
}