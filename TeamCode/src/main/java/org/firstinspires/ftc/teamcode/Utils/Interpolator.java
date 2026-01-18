package org.firstinspires.ftc.teamcode.Utils;

import java.util.ArrayList;
import java.util.List;

/**
 * A 2D interpolator that takes two input values (x, y) and returns an interpolated output value.
 * Uses bilinear interpolation when the query point falls within the grid of known points.
 */
public class Interpolator {
    private List<DataPoint> dataPoints;
    private static class DataPoint {
        final double x;
        final double y;
        final double value;
        DataPoint(double x, double y, double value) {
            this.x = x;
            this.y = y;
            this.value = value;
        }


        double distanceTo(double targetX, double targetY) {
            return Math.hypot(x - targetX, y - targetY);
        }
    }

    public Interpolator() {
        dataPoints = new ArrayList<>();
    }

    public void addPoint(double x, double y, double value) {
        dataPoints.add(new DataPoint(x, y, value));
    }

    public double get(double x, double y) {
        if (dataPoints.isEmpty()) {
            throw new IllegalStateException("No data points added to interpolator");
        }
        // Check for exact match first
        for (DataPoint point : dataPoints) {
            if (Math.abs(point.x - x) < 1e-9 && Math.abs(point.y - y) < 1e-9) {
                return point.value;
            }
        }

        // Try bilinear interpolation
        Double result = tryBilinearInterpolation(x, y);
        if (result != null) {
            return result;
        }

        // Fall back to nearest neighbor if bilinear fails
        return nearestNeighbor(x, y);
    }


    private Double tryBilinearInterpolation(double x, double y) {
        // Find bounding points
        DataPoint bottomLeft = null, bottomRight = null, topLeft = null, topRight = null;
        double minXDiff = Double.MAX_VALUE, minYDiff = Double.MAX_VALUE;

        // Find x boundaries
        double lowerX = Double.NEGATIVE_INFINITY;
        double upperX = Double.POSITIVE_INFINITY;

        for (DataPoint p : dataPoints) {
            if (p.x <= x && p.x > lowerX){
                lowerX = p.x;
                bottomLeft = p;
            }
            if (p.x >= x && p.x < upperX){
                upperX = p.x;
                bottomRight = p;
            }
        }

        // Find y boundaries
        double lowerY = Double.NEGATIVE_INFINITY;
        double upperY = Double.POSITIVE_INFINITY;

        for (DataPoint p : dataPoints) {
            if (p.y <= y && p.y > lowerY) {
                lowerY = p.y;
                bottomLeft = p;
            }
            if (p.y >= y && p.y < upperY){
                upperY = p.y;
                bottomRight = p;
            }
        }

        // Check if we have valid boundaries
        if (Double.isInfinite(lowerX) || Double.isInfinite(upperX) ||
                Double.isInfinite(lowerY) || Double.isInfinite(upperY)) {
            return null; // Can't do bilinear, point is outside grid
        }


        // If we don't have all 4 corners, can't do bilinear
        if (bottomLeft == null || bottomRight == null || topLeft == null || topRight == null) {
            return null;
        }

        // Perform bilinear interpolation
        double tx = (x - lowerX) / (upperX - lowerX);
        double ty = (y - lowerY) / (upperY - lowerY);

        // Interpolate along x at bottom
        double bottom = lerp(tx, bottomLeft.value, bottomRight.value);

        // Interpolate along x at top
        double top = lerp(tx, topLeft.value, topRight.value);

        // Interpolate along y
        return lerp(ty, bottom, top);
    }

    /**
     * Find the nearest neighbor value
     */
    private double nearestNeighbor(double x, double y) {
        DataPoint nearest = dataPoints.get(0);
        double minDist = nearest.distanceTo(x, y);

        for (DataPoint point : dataPoints) {
            double dist = point.distanceTo(x, y);
            if (dist < minDist) {
                minDist = dist;
                nearest = point;
            }
        }

        return nearest.value;
    }
    /**
     * Linear interpolation helper
     */
    private double lerp(double t, double start, double end) {
        return start + t * (end - start);
    }

}