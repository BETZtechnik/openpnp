package org.firepick.vision;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import javax.imageio.ImageIO;

import org.apache.commons.io.FileUtils;
import org.apache.commons.io.IOUtils;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;
import org.openpnp.model.Configuration;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;


public class FireSight {
    private static final Logger logger = LoggerFactory
            .getLogger(FireSight.class);
    
    public static FireSightResult fireSight(BufferedImage input, JsonArray pipeline) throws Exception {
        File pipelineFile = File.createTempFile("fsp", ".json");
        FileUtils.write(pipelineFile, pipeline.toString());
        return fireSight(input, pipelineFile);
    }
    
    public static FireSightResult fireSight(BufferedImage input, String pipelineResourceName) throws Exception {
        return fireSight(input, ClassLoader.getSystemResourceAsStream(pipelineResourceName));
    }

    public static FireSightResult fireSight(BufferedImage input, InputStream pipeline) throws Exception {
        File pipelineFile = File.createTempFile("fsp", ".json");
        FileUtils.copyInputStreamToFile(pipeline, pipelineFile);
        return fireSight(input, pipelineFile);
    }
    
    /**
     * Run FireSight against the input image using a pipeline defined in a
     * classpath resource called pipelineResourceName.
     * @param input
     * @param pipelineResourceName
     * @return
     * @throws Exception
     */
    public static FireSightResult fireSight(BufferedImage input, File pipelineFile) throws Exception {
        File sourceFile = File.createTempFile("fsi", ".png");
        ImageIO.write(input, "png", sourceFile);
        
        // create a place to store the output image
        File outputFile = File.createTempFile("fso", ".png");
        
        // run FireSight
        ProcessBuilder pb = new ProcessBuilder(
                "firesight", 
                "-i",
                sourceFile.getAbsolutePath(),
                "-p",
                pipelineFile.getAbsolutePath(),
                "-o",
                outputFile.getAbsolutePath());
        Process process = pb.start();
        String pOut = IOUtils.toString(process.getInputStream());
        String pErr = IOUtils.toString(process.getErrorStream());
        int ret = process.waitFor();
        if (ret != 0) {
            throw new Exception("FireSight execution failed:\n" + pErr);
        }
        
        if (logger.isDebugEnabled()) {
            // log the firesight results
            copyFileToLogDirectory(sourceFile, "fsi", ".png");
            copyFileToLogDirectory(pipelineFile, "fsp", ".json");
            copyFileToLogDirectory(outputFile, "fso", ".png");
            File output = Configuration.get().createResourceFile(FireSight.class, "fsr", ".json");
            FileUtils.write(output, pOut);
        }
        
        JsonObject results = (JsonObject) new JsonParser().parse(pOut);
        BufferedImage output = ImageIO.read(outputFile);
        return new FireSightResult(results, output);
    }
    
    private static void copyFileToLogDirectory(File file, String prefix, String suffix) throws Exception {
        File output = Configuration.get().createResourceFile(FireSight.class, prefix, suffix);
        FileUtils.copyFile(file, output);
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
        double a = o.get("angle").getAsDouble();
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

