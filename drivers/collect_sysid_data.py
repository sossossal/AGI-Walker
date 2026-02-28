
import time
import math
import csv
import argparse
import logging
try:
    from drivers.real_robot_driver import RealRobotDriver
except ModuleNotFoundError:
    from real_robot_driver import RealRobotDriver

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("SysIDCollector")

def generate_sine_sweep(duration: float, start_freq: float, end_freq: float, amplitude: float):
    """Generator for sine sweep trajectory"""
    t = 0.0
    dt = 0.01 # 100Hz
    while t < duration:
        # Linear chirp: f(t) = f0 + (f1-f0)*t/T
        freq = start_freq + (end_freq - start_freq) * (t / duration)
        phase = 2 * math.pi * freq * t # Simplified phase accumulation approx for short duration
        # Correct phase accumulation: integral of freq
        # phi(t) = 2*pi * (f0*t + (k/2)*t^2) where k = (f1-f0)/T
        k = (end_freq - start_freq) / duration
        phi = 2 * math.pi * (start_freq * t + (k/2) * t*t)
        
        pos = amplitude * math.sin(phi)
        yield t, pos
        t += dt

def collect_data(port: str, duration: float, out_file: str, mock: bool):
    driver = RealRobotDriver(port=port, mock=mock)
    if not driver.connect():
        logger.error("Failed to connect")
        return

    logger.info("Starting SysID Data Collection (Sine Sweep)...")
    
    # trajectory parameters
    AMPLITUDE = 0.5 # rad
    START_FREQ = 0.1 # Hz
    END_FREQ = 5.0 # Hz
    
    data_log = []
    
    start_time = time.time()
    try:
        for t_sim, target_pos in generate_sine_sweep(duration, START_FREQ, END_FREQ, AMPLITUDE):
            # Send command to ALL motors for simplicity, or specific joint
            # Let's excite "motor_1" (e.g. knee)
            cmd = {'motor_1': target_pos}
            driver.send_motor_commands(cmd)
            
            # Read state
            state = driver.get_state()
            curr_time = time.time() - start_time
            
            # Log: Timestamp, Target, Actual_Pos, Actual_Vel, Torque
            if 'motor_1' in state['motors']:
                m = state['motors']['motor_1']
                data_log.append({
                    'time': curr_time,
                    'target_pos': target_pos,
                    'actual_pos': m.get('pos', 0),
                    'actual_vel': m.get('vel', 0),
                    'actual_torque': m.get('torque', 0),
                    'voltage': m.get('voltage', 0),
                    'current': m.get('current', 0)
                })
            
            time.sleep(0.01) # Maintain ~100Hz loop
            
    except KeyboardInterrupt:
        logger.info("Interrupted")
    finally:
        driver.disconnect()
        
    # Save to CSV
    if data_log:
        keys = data_log[0].keys()
        with open(out_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=keys)
            writer.writeheader()
            writer.writerows(data_log)
        logger.info(f"Saved {len(data_log)} samples to {out_file}")
    else:
        logger.warning("No data collected")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", default="COM3")
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument("--out", default="sysid_data.csv")
    parser.add_argument("--mock", action="store_true")
    args = parser.parse_args()
    
    collect_data(args.port, args.duration, args.out, args.mock)
