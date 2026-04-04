
import numpy as np
import pandas as pd
from scipy.optimize import curve_fit
import argparse
import json
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("SysIDFitter")

def motor_dynamics(data, k_torque, k_damping, k_friction):
    """
    Simplified Motor Model:
    Torque = K_t * Current
    Torque = Inertia * Acc + Damping * Vel + Friction * sign(Vel) + Load_Torque
    
    Here we assume we measure 'actual_torque' (from current) or we want to fit the relationship:
    Voltage = R * Current + K_e * Vel
    Current = (Voltage - K_e * Vel) / R
    Torque_delivered = K_t * Current - (Damping * Vel + Friction * sign(Vel))
    
    Let's assume we want to find effective Damping and Friction given Torque and Velocity.
    Model: T_net = T_motor - T_loss
    T_loss = k_damping * vel + k_friction * sign(vel)
    
    If we are controlling position, the PID output is Torque (or Current).
    Let's fit:
    Measured_Torque = Inertia * Acc + k_damping * Vel + k_friction * sign(Vel)
    
    We need Acc (derivative of Vel).
    """
    vel = data[:, 0]
    acc = data[:, 1]
    return k_damping * vel + k_friction * np.sign(vel) 
    # Inertia is usually handled by the physics engine body mass, but rotor inertia exists.
    # For now, let's just fit friction/damping models.

def fit_data(csv_file: str, out_file: str):
    df = pd.read_csv(csv_file)
    
    # Preprocess
    # Calculate acceleration
    df['dt'] = df['time'].diff().fillna(0.01)
    df['acc'] = df['actual_vel'].diff() / df['dt']
    df = df.fillna(0)
    
    # We want to explain 'actual_torque' (from current) using Vel and Acc?
    # Or matches Simulation parameters?
    # In Godot: Joint Damping, Joint Friction.
    # T_applied = T_motor - Damping * w - Friction
    
    # We essentially want to find Damping and Friction coefficients that match the real data curve.
    # T_motor (Current * Kt) = J * alpha + B * omega + C * sign(omega) + T_external
    # If robot is in air (no contact), T_external ~ Gravity (if vertical).
    
    # For MVP: Simple linear regression on T ~ Vel
    
    X = df[['actual_vel']].values
    y = df['actual_torque'].values # Assuming this is K_t * Current
    
    # Function to optimize
    def func(v, b, c):
        return b * v + c * np.sign(v)
    
    popt, pcov = curve_fit(func, df['actual_vel'], df['actual_torque'])
    
    k_damping, k_friction = popt
    logger.info(f"Fitted Parameters: Damping={k_damping:.4f}, Friction={k_friction:.4f}")
    
    # Save to JSON
    result = {
        "joint_damping": float(k_damping),
        "joint_friction": float(k_friction),
        "r_squared": 0.0 # TODO calculate
    }
    
    with open(out_file, 'w') as f:
        json.dump(result, f, indent=4)
        
    logger.info(f"Saved parameters to {out_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_file")
    parser.add_argument("--out", default="fitted_params.json")
    args = parser.parse_args()
    
    fit_data(args.csv_file, args.out)
