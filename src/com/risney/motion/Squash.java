package com.risney.motion;


import bRigid.BObject;
import bRigid.BPhysics;
import bRigid.BSphere;

import java.nio.ByteBuffer;

import javax.vecmath.Vector3f;

import processing.core.PApplet;
import processing.core.PVector;

import processing.data.JSONObject;

import processing.serial.Serial;

import saito.objloader.BoundingBox;
import saito.objloader.OBJModel;

import shapes3d.utils.Rot;


public class Squash extends PApplet {
    String portName = "COM3";


    BPhysics physics;
    BSphere sphere;

   // JSONObject config;

    // Variable to manage manual mouse-based rotation
    float rotX = 0f;

    // serial port
    Serial port;

    // Rotation object for tteh MPU-9150 sensor to keep track of each orientation (default: no rotation)
    Rot r = new Rot(new PVector(1, 0, 0), 0);

    // PVector - for the sensor to keep track of each rotation axis
    PVector axis = new PVector();
    // angle, float for the sensor to keep track of each rotation angle
    float angle = 0;

    // An OBJModel object for each piece of the model (body, upper arm, forearm)
    OBJModel racquet;

    // Scale parameter to make sure the model(s) are easily visible. Default: 40
    float scaleParam = 8;

    // Flag to determine if we should display the body model or just the arm
    boolean drawBody = false;

    // A BoundingBox object for the upper arm and forearm to help determine the joint positions
    BoundingBox racquetBox;

    // Flag to easily switch between bluetooth mode (connect=true) and static mode (connect=false)
    boolean connect = true;

    // Flag to determine if we should show the COM ports or not
    boolean showPorts = false;

    // Set up the program

    public void setup() {
        // Resolution by default is 1000x800, using P3D which tries to use OpenGL if possible
        size(1500, 1200, P3D);


        frameRate(30);

        // cam = new PeasyCam(this, 600);

        //extends of physics world
        Vector3f min = new Vector3f(-250, -250, -250);
        Vector3f max = new Vector3f(250, 250, 250);
        //create a rigid physics engine with a bounding box
        physics = new BPhysics(min, max);
        //set gravity
        physics.world.setGravity(new Vector3f(0, 500, 0));

        //create the first rigidBody as Box or Sphere
        //BSphere(PApplet p, float mass, float x, float y, float z, float radius)
        sphere = new BSphere(this, 2, 0, 0, 0, 20);

        Vector3f pos = new Vector3f(random(30), -150, random(1));
        //reuse the rigidBody of the sphere for performance resons
        //BObject(PApplet p, float mass, BObject body, Vector3f center, boolean inertia)
        BObject r = new BObject(this, 100, sphere, pos, true);
        //add body to the physics engine
        physics.addBody(r);


        // Disable drawing the line-strokes
        noStroke();
        // Instantiate the racquet OBJModel, with (Blender) .obj files
        racquet = new OBJModel(this, "Squash_Racquet.obj", QUAD);


        racquet.enableTexture();
        racquet.scale(scaleParam);

        racquetBox = new BoundingBox(this, racquet);

        // List all the available serial ports:
        println("Available serial (COM) ports:");
        for (String s : Serial.list())
            println(s);

        // Connect to the two serial ports
        if (connect) {
            // Exit if no serial ports found
            if (Serial.list().length == 0) {
                println("No serial ports found!");
                exit();
            } else {
                // Instantiate the JSON object to read the configuration data for the bluetooth sensors
                //config = loadJSONObject("config.json");
                port = new Serial(this, portName, 115200);
                port.bufferUntil('$');
                println("Connected and buffering!");
            }
        }
    }

    // Runs each frame

    public void draw() {
        // Set the background color to white
        background(255, 255, 255);


        // Use default lighting
        lights();
        // Translate to a good view point so that the racquet  is in sight
        translate(width / 2.0f, height / 2.0f + 125, 0);
        // Allow the mouse to rotate the scene around the vertical axis
        rotateY(rotX);

        // Get the width/height/depth of the racquet bounding box
        PVector whd = racquetBox.getWHD();
        // Get the center of the racquet bounding box
        PVector center = racquetBox.getCenter();
        // Translate the rotation point to the center of the racquet bounding box
        translate(center.x, center.y, center.z);
        // Translate the rotation point to the racquet
        //translate(whd.x / 4.0f, -whd.y / 2.0f, 0);
        // Rotate about the Z-axis so that by default the racquet is hanging down
        //rotateZ(PI / 2.0f);
        // Rotate the racquet  by the amount provided by the sensor
        // See the docs for how the axes convert from the sensor frame of reference to this one
        //rotate(angle, -axis.z, -axis.y, axis.x);
        rotate(angle, axis.x, axis.y, axis.z);

        // Translate back to the center of the scene
        translate(-center.x - whd.x / 4.0f, -center.y + whd.y / 2.0f, 0);
        // Draw the rqacquet


        racquet.enableMaterial();
        racquet.enableTexture();

        racquet.draw();

        //strokeWeight(1);
        //drawAxes();

        pushMatrix();
        popMatrix();


        //update physics engine every frame
        physics.update();
        // default display of every shape
        physics.display();
       // saveFrame("racquet-######.tga");
    }


    void drawAxes() {
        // stroke(255, 0, 0);
        line(-50, 0, 0, 50, 0, 0);

        //stroke(0, 255, 0);
        line(0, -50, 0, 0, 50, 0);

        //stroke(0, 0, 255);
        line(0, 0, -50, 0, 0, 50);
    }

    /** Helper method to process the data stream
     * @param byte[] rs: a byte array read from the data stream which is 23 bytes long
     * @param int id: indicates which sensor this data packet came from (0 or 1)
     **/

    public void processQuat(byte[] rs) {
        // Quaternions come as (w, x, y, z)
        // Each component of the quaternion is 4 bytes long
        // Extract each component into a float
        float q0 = ByteBuffer.wrap(subset(rs, 2, 4)).getInt() * 1.0f / (1 << 30);
        float q1 = ByteBuffer.wrap(subset(rs, 6, 4)).getInt() * 1.0f / (1 << 30);
        float q2 = ByteBuffer.wrap(subset(rs, 10, 4)).getInt() * 1.0f / (1 << 30);
        float q3 = ByteBuffer.wrap(subset(rs, 14, 4)).getInt() * 1.0f / (1 << 30);

        // Create a new Rotation object from this quaternion and save it to the Rotation array
        r = new Rot(q0, q1, q2, q3, true);

        // Extract the angle-axis representation of this rotation for easy use
        axis = r.getAxis();
        angle = r.getAngle();
    }

    /** Helper method to parse the data stream and determine
     * if this is a quaternion or some other kind of data (currently only processes quaternion packets)
     * @param byte[] rs: a byte array of 23 bytes long where the first byte is an indicator of the packet type
     * @param int id: indicates which sensor this packet came from (0 or 1)
     **/

    public void parseResponse(byte[] rs) {
        // If the first byte is equal to 2, this is a quaternion
        if (rs[0] == 2) {
            processQuat(rs);
        }
    }

    /** Callback function for the serial ports called when there is buffered data available
     * @param Serial port: the port that has buffered data available (automatically called)
     **/

    public void serialEvent(Serial port) {

        // Create a byte array buffer to read the sensor data into
        byte[] bytes = new byte[23];
        int bytesread = port.readBytes(bytes);
        // If the last byte is a '$', that means we read an entire packet
        // since the data is being buffered until a '$', so parse the packet
        if (bytes[22] == '$') {
            parseResponse(bytes);
        }
    }

    /* Triggered when a keyboard button is pressed */

    public void keyPressed() {
        if (key == 't')
            drawBody = !drawBody;
        else {
            if (key == ESC)
                exit();
            else {
                if (key != CODED && !showPorts) {
                    println("Sending (to first sensor): " + key);
                    // Send the keystroke out:
                    port.write(key);
                    println("Sent!");
                }
            }
        }
    }


    /** Triggered when a mouse button is held down while moving the mouse **/

    public void mouseDragged() {
        // Compute the difference between the previous mouse location and the current location
        float x = (mouseX - pmouseX);
        if (mouseButton == LEFT) {
            // Uses the amount of movement to rotate the scene
            rotX += x * 0.01f;
        }
    }

    static public void main(String[] passedArgs) {
        String[] appletArgs = new String[] { "com.risney.motion.Squash" };
        if (passedArgs != null) {
            PApplet.main(concat(appletArgs, passedArgs));
        } else {
            PApplet.main(appletArgs);
        }
    }
}
