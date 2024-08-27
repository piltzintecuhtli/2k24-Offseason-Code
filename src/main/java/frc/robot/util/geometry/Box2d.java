package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Represents a 2D Box with 4 corners, the corners are numbered clockwise */
public class Box2d {
  private Translation2d corner1;
  private Translation2d corner2;
  private Translation2d corner3;
  private Translation2d corner4;

  /** Constructs a Box2d object using the 4 corners of the box */
  public Box2d(
      double x1, double y1, double x2, double y2, double x3, double y3, double x4, double y4) {
    this.corner1 = new Translation2d(x1, y1);
    this.corner2 = new Translation2d(x2, y2);
    this.corner3 = new Translation2d(x3, y3);
    this.corner4 = new Translation2d(x4, y4);
  }

  /** Constructs a Box2d object using the top left and bottom right corners of the box */
  public Box2d(double x1, double y1, double x3, double y3) {
    this.corner1 = new Translation2d(x1, y1);
    this.corner2 = new Translation2d(x3, y1);
    this.corner3 = new Translation2d(x3, y3);
    this.corner4 = new Translation2d(x1, y3);
  }

  /** Constructs a Box2d object using the 4 corners of the box */
  public Box2d(Pose2d corner1, Pose2d corner2, Pose2d corner3, Pose2d corner4) {
    this.corner1 = corner1.getTranslation();
    this.corner2 = corner2.getTranslation();
    this.corner3 = corner3.getTranslation();
    this.corner4 = corner4.getTranslation();
  }

  /** Constructs a Box2d object using the top left and bottom right corners of the box */
  public Box2d(Pose2d corner1, Pose2d corner3) {
    this.corner1 = corner1.getTranslation();
    this.corner2 = new Translation2d(corner3.getX(), corner1.getY());
    this.corner3 = corner3.getTranslation();
    this.corner4 = new Translation2d(corner1.getX(), corner3.getY());
  }

  /** Constructs a Box2d object using the 4 corners of the box */
  public Box2d(
      Translation2d corner1, Translation2d corner2, Translation2d corner3, Translation2d corner4) {
    this.corner1 = corner1;
    this.corner2 = corner2;
    this.corner3 = corner3;
    this.corner4 = corner4;
  }

  /** Constructs a Box2d object using the top left and bottom right corners of the box */
  public Box2d(Translation2d corner1, Translation2d corner3) {
    this.corner1 = corner1;
    this.corner2 = new Translation2d(corner3.getX(), corner1.getY());
    this.corner3 = corner3;
    this.corner4 = new Translation2d(corner1.getX(), corner3.getY());
  }

  /** Constructs a Box2d object using the center point, width, height, and rotation angle */
  public Box2d(Translation2d center, double width, double height, Rotation2d rotation) {
    double halfWidth = width / 2.0;
    double halfHeight = height / 2.0;

    // Define corners without rotation
    Translation2d corner1 = new Translation2d(-halfWidth, halfHeight);
    Translation2d corner2 = new Translation2d(halfWidth, halfHeight);
    Translation2d corner3 = new Translation2d(halfWidth, -halfHeight);
    Translation2d corner4 = new Translation2d(-halfWidth, -halfHeight);

    // Rotate the entire box shape around the center point
    double cosRotation = rotation.getCos();
    double sinRotation = rotation.getSin();

    this.corner1 =
        new Translation2d(
            center.getX() + (corner1.getX() * cosRotation - corner1.getY() * sinRotation),
            center.getY() + (corner1.getX() * sinRotation + corner1.getY() * cosRotation));

    this.corner2 =
        new Translation2d(
            center.getX() + (corner2.getX() * cosRotation - corner2.getY() * sinRotation),
            center.getY() + (corner2.getX() * sinRotation + corner2.getY() * cosRotation));

    this.corner3 =
        new Translation2d(
            center.getX() + (corner3.getX() * cosRotation - corner3.getY() * sinRotation),
            center.getY() + (corner3.getX() * sinRotation + corner3.getY() * cosRotation));

    this.corner4 =
        new Translation2d(
            center.getX() + (corner4.getX() * cosRotation - corner4.getY() * sinRotation),
            center.getY() + (corner4.getX() * sinRotation + corner4.getY() * cosRotation));
  }

  /** Constructs a Box2d object using the center pose, width, height */
  public Box2d(Pose2d center, double width, double height) {
    double halfWidth = width / 2.0;
    double halfHeight = height / 2.0;

    // Define corners without rotation
    Translation2d corner1 = new Translation2d(-halfWidth, halfHeight);
    Translation2d corner2 = new Translation2d(halfWidth, halfHeight);
    Translation2d corner3 = new Translation2d(halfWidth, -halfHeight);
    Translation2d corner4 = new Translation2d(-halfWidth, -halfHeight);

    // Rotate the entire box shape around the center point
    double cosRotation = center.getRotation().getCos();
    double sinRotation = center.getRotation().getSin();

    this.corner1 =
        new Translation2d(
            center.getX() + (corner1.getX() * cosRotation - corner1.getY() * sinRotation),
            center.getY() + (corner1.getX() * sinRotation + corner1.getY() * cosRotation));

    this.corner2 =
        new Translation2d(
            center.getX() + (corner2.getX() * cosRotation - corner2.getY() * sinRotation),
            center.getY() + (corner2.getX() * sinRotation + corner2.getY() * cosRotation));

    this.corner3 =
        new Translation2d(
            center.getX() + (corner3.getX() * cosRotation - corner3.getY() * sinRotation),
            center.getY() + (corner3.getX() * sinRotation + corner3.getY() * cosRotation));

    this.corner4 =
        new Translation2d(
            center.getX() + (corner4.getX() * cosRotation - corner4.getY() * sinRotation),
            center.getY() + (corner4.getX() * sinRotation + corner4.getY() * cosRotation));
  }

  /** Returns a Translation2d[] of the corners */
  public Translation2d[] getCorners() {
    return new Translation2d[] {corner1, corner2, corner3, corner4};
  }

  /** Calculates the closest point on the perimeter of the box to the given translation */
  public Translation2d closestPoint(Translation2d point) {
    Translation2d[] vertices = {corner1, corner2, corner3, corner4};
    Translation2d closestPoint = null;
    double minDistance = Double.MAX_VALUE;

    for (int i = 0; i < vertices.length; i++) {
      Translation2d edgeStart = vertices[i];
      Translation2d edgeEnd = vertices[(i + 1) % vertices.length];
      Translation2d closestPointOnEdge;
      double x1 = edgeStart.getX();
      double y1 = edgeStart.getY();
      double x2 = edgeEnd.getX();
      double y2 = edgeEnd.getY();
      double px = point.getX();
      double py = point.getY();

      double dx = x2 - x1;
      double dy = y2 - y1;
      double lengthSquared = dx * dx + dy * dy;

      if (lengthSquared == 0) {
        closestPointOnEdge = edgeStart;
      } else {

        double t = ((px - x1) * dx + (py - y1) * dy) / lengthSquared;
        t = Math.max(0, Math.min(1, t));

        closestPointOnEdge = new Translation2d(x1 + t * dx, y1 + t * dy);
      }
      double distance = point.getDistance(closestPointOnEdge);
      if (distance < minDistance) {
        minDistance = distance;
        closestPoint = closestPointOnEdge;
      }
    }

    return closestPoint;
  }

  /** Calculates the closest point on the perimeter of the box to the given pose */
  public Translation2d closestPoint(Pose2d point) {
    Translation2d[] vertices = {corner1, corner2, corner3, corner4};
    Translation2d closestPoint = null;
    double minDistance = Double.MAX_VALUE;

    for (int i = 0; i < vertices.length; i++) {
      Translation2d edgeStart = vertices[i];
      Translation2d edgeEnd = vertices[(i + 1) % vertices.length];
      Translation2d closestPointOnEdge;
      double x1 = edgeStart.getX();
      double y1 = edgeStart.getY();
      double x2 = edgeEnd.getX();
      double y2 = edgeEnd.getY();
      double px = point.getX();
      double py = point.getY();

      double dx = x2 - x1;
      double dy = y2 - y1;
      double lengthSquared = dx * dx + dy * dy;

      if (lengthSquared == 0) {
        closestPointOnEdge = edgeStart;
      } else {

        double t = ((px - x1) * dx + (py - y1) * dy) / lengthSquared;
        t = Math.max(0, Math.min(1, t));

        closestPointOnEdge = new Translation2d(x1 + t * dx, y1 + t * dy);
      }
      double distance = point.getTranslation().getDistance(closestPointOnEdge);
      if (distance < minDistance) {
        minDistance = distance;
        closestPoint = closestPointOnEdge;
      }
    }

    return closestPoint;
  }

  /** Checks if a given Translation2d is inside the Box2d */
  public boolean contains(Translation2d translation) {
    Translation2d[] vertices = {corner1, corner2, corner3, corner4};
    int n = vertices.length;
    boolean inside = false;

    for (int i = 0, j = n - 1; i < n; j = i++) {
      double xi = vertices[i].getX(), yi = vertices[i].getY();
      double xj = vertices[j].getX(), yj = vertices[j].getY();
      double px = translation.getX(), py = translation.getY();

      boolean intersect = ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
      if (intersect) {
        inside = !inside;
      }
    }

    return inside;
  }

  /** Checks if a given Pose2d is inside the Box2d */
  public boolean contains(Pose2d pose) {
    Translation2d[] vertices = {corner1, corner2, corner3, corner4};
    int n = vertices.length;
    boolean inside = false;

    for (int i = 0, j = n - 1; i < n; j = i++) {
      double xi = vertices[i].getX(), yi = vertices[i].getY();
      double xj = vertices[j].getX(), yj = vertices[j].getY();
      double px = pose.getX(), py = pose.getY();

      boolean intersect = ((yi > py) != (yj > py)) && (px < (xj - xi) * (py - yi) / (yj - yi) + xi);
      if (intersect) {
        inside = !inside;
      }
    }

    return inside;
  }
}
