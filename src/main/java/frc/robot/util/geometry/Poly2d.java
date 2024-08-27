package frc.robot.util.geometry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Arrays;

/** Represents a 2D Polygon with n corners which are listed in clockwise order */
public class Poly2d {
  private Translation2d[] vertices;

  public Poly2d(Translation2d... translations) {
    this.vertices = translations;
  }

  public Poly2d(Pose2d... poses) {
    this.vertices =
        Arrays.stream(poses)
            .map((Pose2d pose) -> pose.getTranslation())
            .toArray(Translation2d[]::new);
  }

  /** Returns a Translation2d[] of the vertices */
  public Translation2d[] getVertices() {
    return vertices;
  }

  /** Calculates the closest point on the perimeter of the polygon to the given translation */
  public Translation2d closestPoint(Translation2d point) {
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

  /** Checks if a given Translation2d is inside the polygon */
  public boolean contains(Translation2d translation) {
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

  /** Checks if a given Pose2d is inside the polygon */
  public boolean contains(Pose2d pose) {
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
