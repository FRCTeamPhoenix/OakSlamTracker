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
    p.add_argument('--aprilTagPath', help="Path to .json file with AprilTag ids, sizes and poses", default=None)
    p.add_argument("--useRgb", help="Use OAK-D RGB camera", action="store_true")
    p.add_argument('--irDotBrightness', help='OAK-D Pro (W) IR laser projector brightness (mA), 0 - 1200', type=float, default=0)
    p.add_argument('--noFeatureTracker', help='Disable on-device feature tracking and depth map', action="store_true")
    p.add_argument('--lowLatency', help='Enable low-latency configuration', action="store_true")
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
        if len(mapperOutput.updatedKeyFrames) < 1:
            return
        frame = mapperOutput.map.keyFrames[max(mapperOutput.updatedKeyFrames)].frameSet.primaryFrame
        camPose = frame.cameraPose.pose
        table.putNumberArray("Pose", [camPose.position.x, camPose.position.y])
        print(f'{camPose.position.x}, {camPose.position.y}')
        if mapperOutput.finalMap: print("Final map ready!")

    def onVioOutput(vioOutput):
        table.putNumberArray("Pose", [vioOutput.pose.position.x, vioOutput.pose.position.y])
        print(f'{vioOutput.pose.position.x}, {vioOutput.pose.position.y}')

    def on_press(key):
        global break_prog
        if key == keyboard.Key.esc:
            break_prog = True

    def captureLoop():
        print("Starting OAK-D device")
        pipeline = depthai.Pipeline()
        config = spectacularAI.depthai.Configuration()
        config.useFeatureTracker = not args.noFeatureTracker

        if args.aprilTagPath is not None:
            config.aprilTagPath = args.aprilTagPath
        if args.load_map is not None:
            config.mapLoadPath = args.load_map
        if args.save_map is not None:
            config.mapSavePath = args.save_map

        config.useSlam = True
        config.lowLatency = args.lowLatency
        config.useStereo = True

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
                
    def quaternion_yaw(x, y, z, w):     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return yaw_z

    thread = threading.Thread(target=captureLoop)
    thread.start()
    thread.join()