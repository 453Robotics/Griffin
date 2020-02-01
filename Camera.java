package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;

public class Camera
{
    CameraServer server;

    public void init()
    {
        server = CameraServer.getInstance();
        server.startAutomaticCapture();
    }
}

