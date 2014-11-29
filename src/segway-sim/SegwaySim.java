import helper.Ratio;

import java.awt.event.KeyListener;
import java.awt.event.MouseListener;
import java.io.File;

import model.motion.MotionConfig;
import model.scene.SceneModel;
import model.sensor.DistanceSensorConfig;
import model.sensor.GyroSensorConfig;
import run.PC;
import run.Robot;
import run.RunConfig;
import run.SimulatedPC;
import run.SimulatedRobot;
import simulator.SimConfig;
import simulator.Simulator;
import visual.View;
import visual.ViewConfig;

import comm.SimulatedChannelLink;

import control.PCController;
import control.RobotController;

/**
 * Main entry point of the segway program (simulation case).
 */
public class SegwaySim
{
    public static void main(String[] args)
    {
        int exitCode = 0;
        
        View view = null;
        try
        {
            if (args.length != 1)
            {
                System.err.println("Usage: java SegwaySim <run-config>");
                System.exit(-2);
            }
            
            File runCfgFile = new File(args[0]);
            SegwaySim segway = new SegwaySim(runCfgFile);
            view = segway.view;
            segway.run();
        }
        catch (Exception e)
        {
            e.printStackTrace(System.err);
            exitCode = -1;
        }
        finally { if (view != null) view.dispose(); }
        
        System.exit(exitCode);
    }

    //--------------------------------------------------------------------------
    
    public SegwaySim(File runCfgFile)
    throws Exception
    {
        RunConfig runCfg = new RunConfig(runCfgFile);
        SimConfig simCfg = new SimConfig(runCfg.simCfgFile());
        ViewConfig viewCfg = new ViewConfig(runCfg.viewCfgFile());
        MotionConfig motionCfg = new MotionConfig(runCfg.motionCfgFile());

        GyroSensorConfig gyroCfg =
            new GyroSensorConfig(runCfg.gyroSensorConfig());
        
        File[] distCfgFile = runCfg.distSensorConfigs();
        DistanceSensorConfig[] distCfg =
            new DistanceSensorConfig[distCfgFile.length];
        for (int i = 0; i < distCfg.length; ++i)
            distCfg[i] = (DistanceSensorConfig)
                             Class.forName(runCfg.distanceSensorConfigClass())
                             .getConstructor(File.class)
                             .newInstance(distCfgFile[i]);
        
        sceneModel = new SceneModel(motionCfg, runCfg.mapFile(), distCfg);
        view = View.create(sceneModel, viewCfg);
        simulator = new Simulator(simCfg,
                                  gyroCfg, distCfg,
                                  motionCfg, sceneModel);
        
        channelLink = new SimulatedChannelLink();
        robot = new SimulatedRobot(simulator, channelLink.channelA());
        pc = new SimulatedPC(simulator, channelLink.channelB());
        
        robotController = (RobotController)
            Class.forName(runCfg.robotControllerClassName())
            .getConstructor(Robot.class).newInstance(robot);
        robot.setController(robotController);
        simulator.registerThread(robot);
        
        pcController = (PCController)
            Class.forName(runCfg.pcControllerClassName())
            .getConstructor(PC.class, MotionConfig.class,
                            SceneModel.class, View.class)
            .newInstance(pc, motionCfg, sceneModel, view);
        pc.setController(pcController);
        simulator.registerThread(pc);
        
        KeyListener pcKeyListener = pcController.keyListener();
        if (pcKeyListener != null) view.addKeyListener(pcKeyListener);
        
        MouseListener pcMouseListener = pcController.mouseListener();
        if (pcMouseListener != null) view.addMouseListener(pcMouseListener);
    }
    
    private void run()
    throws Exception
    {
        while (!view.isInitialized()) Thread.sleep(10);
        
        simulator.reset();
        robot.start();
        pc.start();
        Thread.sleep(1000);

        try
        {
            long realT, diffT;
            double simT, leftPwr, rightPwr;
            while (!view.isCloseRequested())
            {
                if (simulator.cfg().isGroundHitExit()
                    && simulator.isStanding()
                    && simulator.isGroundHit()) break;
                
                realT = System.currentTimeMillis();
                simT = simulator.time()*1000;
                
                leftPwr = robot.leftPower() * Ratio.MILLIVOLT_TO_VOLT;
                rightPwr = robot.rightPower() * Ratio.MILLIVOLT_TO_VOLT;
                simulator.step(leftPwr, rightPwr);
                
                realT = System.currentTimeMillis() - realT;
                simT = simulator.time()*1000 - simT;
                
                diffT = (long)(simT*simulator.cfg().timeRatio())-realT;
                if (0 < diffT) Thread.sleep(diffT);
            }
        }
        finally
        {
            robotController.terminate();
            pcController.terminate();
        }
        Thread.sleep(1000);
    }
    
    //--------------------------------------------------------------------------
    
    private final SimulatedRobot robot;
    private final SimulatedPC pc;
    private final SimulatedChannelLink channelLink;
    private final RobotController robotController;
    private final PCController pcController;
    
    private final Simulator simulator;
    private final SceneModel sceneModel;
    private final View view;
}
