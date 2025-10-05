package org.firstinspires.ftc.teamcode.pedroPathing;

public class TagCoordinate {
    private float tagX;
    private float tagY;
    private float tagZ;
    private float tagYaw;
    public TagCoordinate(float x, float y, float z, float rotation) {
        tagX = x;
        tagY = y;
        tagZ = z;
        tagYaw = rotation;
    }

    public float getTagX() {
        return tagX;
    }
    public float getTagY() {
        return tagY;
    }
    public float getTagZ() {
        return tagZ;
    }
    public float getTagYaw() {
        return tagYaw;
    }


}
