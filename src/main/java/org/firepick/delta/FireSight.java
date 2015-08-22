package org.firepick.delta;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

import org.apache.commons.io.IOUtils;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;
import org.openpnp.model.Configuration;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;


public class FireSight {
    /**
     * Run FireSight against the input image using a pipeline defined in a
     * configuration file called pipelineName. The pipeline file is expected
     * to be found in OpenPnP's configuration directory under a subdirectory
     * called firesight.
     * @param input
     * @param pipelineName
     * @return
     * @throws Exception
     */
    public static FireSightResult fireSight(BufferedImage input, String pipelineName) throws Exception {
        File sourceFile = Configuration.get().createResourceFile(FireSight.class, "fs-i", ".png");
        ImageIO.write(input, "png", sourceFile);
        
        // load the FireSight pipeline
        File pipeline = new File(
                new File(Configuration.get().getConfigurationDirectory(), "firesight"), 
                pipelineName);
        
        // create a place to store the output image
        File outputFile = Configuration.get().createResourceFile(FireSight.class, "fs-o", ".png");
        
        // run FireSight
        ProcessBuilder pb = new ProcessBuilder(
                "firesight", 
                "-i",
                sourceFile.getAbsolutePath(),
                "-p",
                pipeline.getAbsolutePath(),
                "-o",
                outputFile.getAbsolutePath());
        Process process = pb.start();
        String pOut = IOUtils.toString(process.getInputStream());
        String pErr = IOUtils.toString(process.getErrorStream());
        int ret = process.waitFor();
        if (ret != 0) {
            throw new Exception("FireSight execution failed:\n" + pErr);
        }
        JsonObject results = (JsonObject) new JsonParser().parse(pOut);
        BufferedImage output = ImageIO.read(outputFile);
        return new FireSightResult(results, output);
    }
    
    public static List<RotatedRect> parseRotatedRects(JsonArray a) {
        List<RotatedRect> rects = new ArrayList<RotatedRect>();
        for (JsonElement e : a) {
            JsonObject o = e.getAsJsonObject();
            rects.add(parseRotatedRect(o));
        }
        return rects;
    }
    
    public static RotatedRect parseRotatedRect(JsonObject o) {
        Point p = new Point(o.get("x").getAsDouble(), o.get("y").getAsDouble());
        Size s = new Size(o.get("width").getAsDouble(), o.get("height").getAsDouble());
        double a = o.get("width").getAsDouble();
        RotatedRect r = new RotatedRect(p, s, a);
        return r;
    }
    
    public static class FireSightResult {
        public BufferedImage output;
        public JsonObject model;
        
        public FireSightResult(JsonObject model, BufferedImage output) {
            this.model = model;
            this.output = output;
        }
    }
}

