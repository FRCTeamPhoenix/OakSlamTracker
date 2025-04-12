import spectacularAI
import depthai
from networktables import NetworkTables
import threading
import math
from pynput import keyboard

break_prog = False

def parseArgs():
    import argparse
    p = argparse.ArgumentParser(__doc__)
    p.add_argument('aprilTagPath', help="Path to .json file with AprilTag ids, sizes and poses")
    p.add_argument("--useRgb", help="Use OAK-D RGB camera", action="store_true")
    p.add_argument('--irDotBrightness', help='OAK-D Pro (W) IR laser projector brightness (mA), 0 - 1200', type=float, default=0)
    p.add_argument('--noFeatureTracker', help='Disable on-device feature tracking and depth map', action="store_true")
    p.add_argument('--load-map', help='Map file to load', default=None)
    p.add_argument('--save-map', help='Map file to save', default=None)
    p.add_argument("--useRectification", help="This parameter must be set if the stereo video inputs are not rectified", action="store_true")
    p.add_argument('--keyFrameCandidateInterval', type=int, help='Sets internal parameter keyframeCandidateEveryNthFrame')
    return p.parse_args()

if __name__ == '__main__':
    args = parseArgs()
    NetworkTables.initialize(server='127.0.0.1')
    table = NetworkTables.getTable('OAKSLAM')
    table.putNumberArray("Pose", [0, 0])

    configInternal = {
        "computeStereoPointCloud": "true",
        "pointCloudNormalsEnabled": "true",
        "computeDenseStereoDepth": "true",
    }

    def onMappingOutput(mapperOutput):
        if mapperOutput.finalMap: print("Final map ready!")

    def onVioOutput(vioOutput):
        table.putNumberArray("Pose", [-vioOutput.pose.position.x, -vioOutput.pose.position.y])

    def on_press(key):
        global break_prog
        if key == keyboard.Key.esc:
            break_prog = True

    def captureLoop():
        print("Starting OAK-D device")
        pipeline = depthai.Pipeline()
        config = spectacularAI.depthai.Configuration()
        config.useFeatureTracker = not args.noFeatureTracker
        config.aprilTagPath = args.aprilTagPath

        config.useSlam = True
        #config.lowLatency = True
        config.useStereo = True
        if args.map is not None:
            config.mapLoadPath = args.map
        if args.save_map is not None:
            config.mapSavePath = args.save_map

        config.useColor = args.useRgb
        config.internalParameters = configInternal
        if args.keyFrameCandidateInterval: config.keyframeCandidateEveryNthFrame = args.keyFrameCandidateInterval
        vioPipeline = spectacularAI.depthai.Pipeline(pipeline, config, onMappingOutput)

        
        with keyboard.Listener(on_press=on_press) as listener:
            with depthai.Device(pipeline) as device, \
                vioPipeline.startSession(device) as vio_session:
                if args.irDotBrightness > 0:
                    device.setIrLaserDotProjectorBrightness(args.irDotBrightness)
                while not break_prog:
                    onVioOutput(vio_session.waitForOutput())
                
    def quaternionHeading(q):
        siny = +2.0 * (q.w * q.z + q.y * q.x);
        cosy = +1.0 - 2.0 * (q.x * q.x + q.z * q.z);
        heading = math.atan2(siny, cosy);
        return heading

    thread = threading.Thread(target=captureLoop)
    thread.start()
    thread.join()