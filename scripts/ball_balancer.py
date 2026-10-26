# Main loop

import cv2
import numpy as np
import time
import argparse
import signal

import hardware
import vision
import control
import kinematics
import kalman
import stream

# CONTROL MODE AND GUI ON/OFF
# exactly one control mode must be given: --pid, --lqr or --rl
# --gui displays the image of what the robot sees
# --kalman feeds the controller with the error and velocity estimated by the kalman filter instead of the raw measurements from the camera
# --log saves time, error and control action of every frame to a csv, read by metrics.py
# --camera d435|sonix reads a USB camera (RealSense D435 colour stream, Sonix webcam) instead of the CSI camera module
# --detect background (default) finds the ball as what differs from the empty plate, whatever its color; color uses the HSV range of vision.py
# --stream HOST[:PORT][,HOST[:PORT]...] sends every frame to the dashboard (dashboard.py, the default) and to the digital
#   twin, reinforcement_learning/twin.py, on the Mac: --stream 127.0.0.1:5006,<mac>
# --dry runs camera, detection and controller with the servos off, to watch the detection or the twin by moving the ball by hand
# --latency S with --kalman: the controllers act on the error predicted S seconds ahead with the ball model, to make up for the
#   delay from the frame to the plate moving (default 0.08 s, 0 disables)
parser = argparse.ArgumentParser()
parser.add_argument("--gui", action="store_true", help="show the camera window")
parser.add_argument("--kalman", action="store_true", help="estimate error and velocity with the Kalman filter instead of the finite difference")
parser.add_argument("--log", action="store_true", help="save time, error and control action of every frame to a csv")
parser.add_argument("--camera", choices=["picam", *hardware.UVC_CAMERAS], default="picam", help="CSI camera module (default) or a USB camera")
parser.add_argument("--detect", choices=["background", "color"], default="background", help="how the ball is told from the rest of the frame")
parser.add_argument("--stream", metavar="HOST[:PORT][,...]", default="127.0.0.1:5006", help="send every frame to these viewers (default: the dashboard on the Pi; add ,<mac> for the twin)")
parser.add_argument("--dry", action="store_true", help="leave the servos off")
parser.add_argument("--latency", type=float, default=0.08, help="with --kalman: control the error predicted this many seconds ahead (0 disables)")
parser.add_argument("--gain", type=float, default=1.0, help="multiplies the PID gains KP, KI, KD of control.py (this build's servos have a dead band the original gains barely cross)")
parser.add_argument("--kd", type=float, default=1.0, help="extra multiplier on KD alone, on top of --gain (more damping)")
parser.add_argument("--ki", type=float, default=1.0, help="extra multiplier on KI alone, on top of --gain")
mode = parser.add_mutually_exclusive_group(required=True)
mode.add_argument("--pid", action="store_true", help="PID control")
mode.add_argument("--lqr", action="store_true", help="LQR control")
mode.add_argument("--rl", action="store_true", help="RL control")
args = parser.parse_args()
gui_on = args.gui
control.KP, control.KI, control.KD = control.KP * args.gain, control.KI * args.gain * args.ki, control.KD * args.gain * args.kd

# SIGTERM (kill, start.sh stop) is treated like ctrl+c, so the servos are released on the way out
signal.signal(signal.SIGTERM, lambda *a: (_ for _ in ()).throw(KeyboardInterrupt()))

fps_smooth = float(hardware.TARGET_FPS)    # variable to keep track of the fps
U_MAX = 0.2                 # largest slope commanded, within the servo travel for every direction
LOST_HOME_S = 0.3           # without the ball for this long the plate goes back to level: it is being put back by hand, and
t_lost = None               # a plate left tilted would throw it off again

t_prev = time.monotonic()   # instant of actuation
had_ball = False            # flag to know wether the previous frame had the ball
u_prev = np.zeros(2)        # slope last applied to the plate
log = []                    # rows of the csv, one per frame with the ball

center = vision.center

if args.camera in hardware.UVC_CAMERAS:
    picam2 = hardware.start_uvc(args.camera, vision.W, vision.H)
else:
    picam2 = hardware.start_camera(vision.W, vision.H)

link = stream.Stream(args.stream, vision.W, vision.H, getattr(picam2, "cal", None), vision.PX_PER_MM) if args.stream else None

try:                # from the first servo pulse on, every exit (ctrl+c, SIGTERM, an error) goes through shut_down
    if not args.dry:
        hardware.home()

    # BACKGROUND CALIBRATION: the plate must be empty, and level, as it is when the servos are at home
    if args.detect == "background":
        print("Calibrating the background, keep the plate empty...")
        time.sleep(0.5)                                         # the plate settles at home
        print(hardware.lock_exposure(picam2))
        noise = vision.calibrate([picam2.capture_array() for _ in range(vision.BG_FRAMES)])
        print(f"Background ready (pixel noise {noise:.1f}), place the ball")

    count = 0       # holds the number of frames
    while True:
        count = count + 1
        frame = picam2.capture_array()
        t_frame = time.monotonic()
        q_sent = None       # servo angles commanded at this frame, for the twin
        q_actual = None     # the angles the servos were really sent, after the limits of hardware.set_angle

        ball, mask = vision.detect(frame, args.detect)

        if ball is not None:

            # ERROR
            cx, cy, r = ball
            err = np.array([center[0] - cx, center[1] - cy], dtype=float)

            # REACQUISITION in case the ball is repositioned for example after falling
            reacquired = not had_ball
            if reacquired:     
                control.reset(err)
                kalman.reset(err)
                t_prev = time.monotonic()
            had_ball = True

            # UPDATE OF TIME AND FRAME RATE
            now = time.monotonic()
            dt = max(now - t_prev, 1e-3)
            t_prev = now

            if not reacquired:      # on reacquisition dt is not the interval between two frames
                fps_smooth = 0.9 * fps_smooth + 0.1 * (1.0/dt)

            # ERROR AND ERROR DERIVATIVE ESTIMATION
            if args.kalman:
                err_ctrl, err_der = kalman.update(err, u_prev, dt)
                if args.latency > 0:
                    err_ctrl, err_der = kalman.predict(err_ctrl, err_der, u_prev, args.latency)
            else:
                err_ctrl, err_der = err, None      # the controllers take the finite difference themselves

            # CONTROL ACTION
            if args.pid:
                u = control.pid(err_ctrl, dt, err_der)
            elif args.lqr:
                u = control.lqr(err_ctrl, dt, err_der)
            elif args.rl:
                u = control.rl(err_ctrl)

            # SATURATION: beyond U_MAX the inverse kinematics has no solution and the loop used to keep the previous command;
            # the plate now goes to the largest slope in the wanted direction instead
            n_u = np.linalg.norm(u)
            if n_u > U_MAX:
                u = u / n_u * U_MAX

            if args.log:    # the raw error, so that runs with --kalman are judged on what the camera sees
                log.append([now, err[0], err[1], u[0], u[1]])

            # INVERSE KINEMATICS
            try:
                q_raw = kinematics.solve(u, kinematics.H_NOM)
                if q_raw is not None:
                    q_deg = np.clip(q_raw, hardware.Q_MIN, hardware.Q_MAX)
                    q_cmd = [q_deg[i] + hardware.OFFSETS[i] for i in range(3)]
                    if not args.dry:
                        q_actual = hardware.set_angle(q_cmd) - hardware.OFFSETS    # what the servos got, after the slew limit
                    q_sent = q_deg
                    u_prev = u      # only a slope that reached the servos acts on the ball
                    if count % 5 == 0:
                        print(f"\rfps: {fps_smooth:5.1f} | err: [{err[0]:6.1f} {err[1]:6.1f}] | u: [{u[0]:+.3f} {u[1]:+.3f}] | q: [{q_cmd[0]:5.1f} {q_cmd[1]:5.1f} {q_cmd[2]:5.1f}]", end="", flush=True)
                else:
                    print("IK unreachable, skipping")
            except (ValueError, ZeroDivisionError, FloatingPointError):
                print("IK error, skipping")
        else:
            if had_ball:
                t_lost = time.monotonic()
            had_ball = False
            if t_lost is not None and time.monotonic() - t_lost > LOST_HOME_S:
                q_sent = np.full(3, float(hardware.Q_HOME))
                if not args.dry:
                    q_actual = hardware.set_angle([hardware.Q_HOME + hardware.OFFSETS[i] for i in range(3)]) - hardware.OFFSETS   # through the slew limit
                u_prev = np.zeros(2)

        if link is not None:
            link.send(t_frame, ball, u_prev if q_sent is not None else None, q_sent, q_actual, getattr(picam2, "t_capture", None))

        # DISPLAY FRAME
        if gui_on:
            frame[mask == 0] = [0, 0, 0]

            if ball is not None:
                # display ball
                cv2.circle(frame, (int(cx), int(cy)), int(r), (0, 255, 0), 3)
                cv2.circle(frame, (int(cx), int(cy)), 5, (0, 255, 0), 3)

                # display error
                cv2.arrowedLine(frame, center, (int(cx), int(cy)), (255, 0, 0), 3, tipLength=0.15)

                # display control (scaled version)
                s = 500
                tip = (int(center[0] + u[0]*s), int(center[1] + u[1]*s))
                cv2.arrowedLine(frame, center, tip, (0, 255, 255), 3, tipLength=0.15)

            frame_show = cv2.resize(frame, (648, 486))
            cv2.imshow("Camera", frame_show)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

except KeyboardInterrupt:       # ctrl+c is the normal way to stop the robot, not an error
    pass
finally:
    print('\nShut down')        # on a new line, after the status line that print keeps rewriting with \r
    hardware.shut_down(picam2)
    cv2.destroyAllWindows()
    if args.log:
        if args.pid:
            name = time.strftime("log_pid_%Y%m%d_%H%M%S.csv")
        elif args.lqr:
            name = time.strftime("log_lqr_%Y%m%d_%H%M%S.csv")
        elif args.rl:
            name = time.strftime("log_rl_%Y%m%d_%H%M%S.csv")
        np.savetxt(name, log, delimiter=",", header="t,ex,ey,ux,uy", comments="")
