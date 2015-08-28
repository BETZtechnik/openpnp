package org.firepick.driver;

import java.io.File;
import java.io.FileReader;
import java.io.Reader;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Point2d;

import org.openpnp.model.LengthUnit;
import org.openpnp.model.Location;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

/**
 * Simple barycentric interpolation for Locations. Uses a map generated in
 * the style required by gfilter. Performs interpolation only in 2D and does
 * not attempt to interpolate outside of the bounds of the map. Implemented
 * with basic geometry because matrices... whaaaat?
 */
public class BarycentricInterpolation {
    private static final Logger logger = LoggerFactory.getLogger(BarycentricInterpolation.class);
    private Map<Point2d, Point2d> map = new HashMap<>();
    
    /**
     * Create an interpolator by parsing a gfilter style mapping from the
     * reader.
     * @param reader
     */
    public BarycentricInterpolation(Reader reader) {
        JsonObject o = new JsonParser().parse(reader).getAsJsonObject();
        JsonArray a = o.get("map").getAsJsonArray();
        for (JsonElement e : a) {
            JsonObject o1 = e.getAsJsonObject();
            JsonArray domain = o1.get("domain").getAsJsonArray();
            JsonArray range = o1.get("range").getAsJsonArray();
            Point2d pDomain = new Point2d(domain.get(0).getAsDouble(), domain.get(1).getAsDouble());
            Point2d pRange = new Point2d(range.get(0).getAsDouble(), range.get(1).getAsDouble());
            map.put(pDomain, pRange);
        }
    }
    
    public BarycentricInterpolation() {
        
    }
    
    public void addPoint(Point2d domain, Point2d range) {
        map.put(domain, range);
    }
    
    public Map<Point2d, Point2d> getMap() {
        return map;
    }
    
    public Location interpolate(Location location) {
        Point2d p = interpolate(new Point2d(location.getX(), location.getY()));
        Location ret = location.derive(p.x, p.y, null, null);
        logger.debug("{} -> {}", location, ret);
        return ret;
    }
    
    /**
     * Perform barycentric interpolation of the point using the nearest three
     * points found in the mapping.
     * @param point
     * @return
     */
    private Point2d interpolate(final Point2d point) {
        // find the three nearest points to the given point
        List<Point2d> sorted = new ArrayList<>(map.keySet());
        sorted.sort(new Comparator<Point2d>() {
            @Override
            public int compare(Point2d o1, Point2d o2) {
                return Double.compare(point.distance(o1), point.distance(o2));
            }
        });
        Point2d a1 = sorted.get(0), b1 = sorted.get(1), c1 = sorted.get(2);
        int lastPoint = 3;
        while (!isTriangle(a1, b1, c1)) {
            logger.debug("Bad triangle ({}, {}, {}), trying again.", new Object[] { a1, b1, c1 });
            c1 = sorted.get(lastPoint++);
        }
        
        if (!isPointInTriangle(point, a1, b1, c1)) {
            logger.debug("Point is not within closest triangle, giving up.");
            return point;
        }
        
        Point2d a2 = map.get(a1);
        Point2d b2 = map.get(b1);
        Point2d c2 = map.get(c1);
        logger.debug("({}, {}, {}) -> ({}, {}, {})", new Object[] { a1, b1, c1, a2, b2, c2 });
        return barycentric(point, a1, b1, c1, a2, b2, c2);
    }
    
    private boolean isTriangle(Point2d a, Point2d b, Point2d c) {
        double A = a.distance(b);
        double B = b.distance(c);
        double C = c.distance(a);
        return (A + B > C && B + C > A && C + A > B);
    }
    
    private boolean isPointInTriangle(Point2d point, Point2d a, Point2d b, Point2d c) {
        boolean b1, b2, b3;

        b1 = sign(point, a, b) < 0.0f;
        b2 = sign(point, b, c) < 0.0f;
        b3 = sign(point, c, a) < 0.0f;

        return ((b1 == b2) && (b2 == b3));
    }
    
    private double sign(Point2d a, Point2d b, Point2d c) {
        return (a.x - c.x) * (b.y - c.y) - (b.x - c.x) * (a.y - c.y);
    }
    
    /**
     * Calculate the area of the triangle with vertices at a, b, c.
     * @param a
     * @param b
     * @param c
     * @return
     */
    private double triangleArea(Point2d a, Point2d b, Point2d c) {
        return triangleArea(a.distance(b), b.distance(c), c.distance(a));
    }
    
    /**
     * Calculate the area of the triangle with sides of length a, b, c.
     * @param a
     * @param b
     * @param c
     * @return
     */
    private double triangleArea(double a, double b, double c) {
        double s = (a + b + c) / 2;
        return Math.sqrt(s * (s - a) * (s - b) * (s - c));
    }
    
    /**
     * Perform barycentric interpolation on the point p using domain
     * triangle a1, b1, c1 to range triangle a2, b2, c2.
     * @param p
     * @param a1
     * @param b1
     * @param c1
     * @param a2
     * @param b2
     * @param c2
     * @return
     */
    private Point2d barycentric(
            Point2d p,
            Point2d a1, Point2d b1, Point2d c1,
            Point2d a2, Point2d b2, Point2d c2
            ) {
        double A = triangleArea(a1, b1, c1);
        double A1 = triangleArea(p, b1, c1);
        double A2 = triangleArea(p, c1, a1);
        double A3 = triangleArea(p, a1, b1);
        double x = ((A1 * a2.x) + (A2 * b2.x) + (A3 * c2.x)) / A;
        double y = ((A1 * a2.y) + (A2 * b2.y) + (A3 * c2.y)) / A;
        return new Point2d(x, y);
    }
    
    public static void main(String[] args) throws Exception {
        BarycentricInterpolation interp = new BarycentricInterpolation(new FileReader(new File("/Users/jason/.openpnp-fpd-hax/gfilter.json")));
        System.out.println(interp.interpolate(new Point2d(0.2, 0.2)));
        System.out.println(interp.interpolate(new Point2d(-0.3, 0.3)));
        System.out.println(interp.interpolate(new Point2d(-0.1, -0.1)));
        System.out.println(interp.interpolate(new Point2d(0.4, -0.4)));
        System.out.println(interp.interpolate(new Location(LengthUnit.Millimeters, 10, 20, 30, 40)));
        System.out.println(interp.interpolate(new Location(LengthUnit.Millimeters, 0, 0, 0, 0)));
    }
}
