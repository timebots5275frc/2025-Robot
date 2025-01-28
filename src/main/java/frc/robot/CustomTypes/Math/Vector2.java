package frc.robot.CustomTypes.Math;

public class Vector2{
    public double x, y;

    public static Vector2 zero = new Vector2(0, 0);

    public Vector2(double x, double y)
    {
        this.x = x;
        this.y = y;
    }

    public double magnitude()
    {
        return Vector2.distance(this, zero);
    }

    public static double distance(Vector2 a, Vector2 b)
    {
        return Math.sqrt(Math.pow(b.x - a.x, 2) + Math.pow(b.y - a.y, 2));
    }

    public Vector2 normalized()
    {
        return this.divideBy(this.magnitude());
    }

    public static Vector2 lerp(Vector2 a, Vector2 b, double percent)
    {
        Vector2 relativePos = b.substract(a);

        return a.add(relativePos.times(percent));
    }

    public static Vector2 clampMagnitude(Vector2 vector2, double max)
    {
        Vector2 out = vector2;
        if (vector2.magnitude() > max) { out = vector2.normalized().times((max)); }

        return out;
    }

    public static Vector2 clampMagnitude(Vector2 vector2, double min, double max)
    {
        Vector2 out = vector2;
        double magnitude = vector2.magnitude();

        if (magnitude > max) { out = vector2.normalized().times((max)); }
        else if (magnitude < min) {out = vector2.normalized().times(min); }

        return out;
    }

    public Vector2 times(double b)
    {
        return new Vector2(x * b, y * b);
    }

    public Vector2 add(Vector2 b)
    {
        return new Vector2(x + b.x, y + b.y);
    }

    public Vector2 substract(Vector2 a)
    {
        return new Vector2(x - a.x, y - a.y);
    }

    public Vector2 divideBy(double b)
    {
        return new Vector2(x / b, y / b);
    }
    
    @Override
    public String toString() {
        return "(" + x + ", " + y + ")";
    }

    public String toString(int decimals) {
        double mult = Math.pow(10, decimals);
        return "(" + (int)(x * mult) / mult + ", " + (int)(y * mult) / mult + ")";
    }

    public static Vector2 rotate(Vector2 v, double delta) {
        return new Vector2(
            v.x * Math.cos(delta) - v.y * Math.sin(delta),
            v.x * Math.sin(delta) + v.y * Math.cos(delta)
        );
    }
}