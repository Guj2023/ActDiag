import scipy.io
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from typing import List, Dict, Optional

def list_variables(file_path: str) -> List[str]:
    """List variables in a .mat file."""
    data = scipy.io.loadmat(file_path)
    return [k for k in data.keys() if not k.startswith('__')]

def plot_variables(file_path: str, variables: List[str], output_path: Optional[str] = None):
    """Visualize variables from a .mat file."""
    data = scipy.io.loadmat(file_path)
    time = data['time'].flatten()
    
    plt.figure(figsize=(10, 6))
    for var in variables:
        if var in data:
            val = data[var].flatten()
            if len(val) == len(time):
                plt.plot(time, val, label=var)
            else:
                print(f"Variable {var} has different length ({len(val)}) than time ({len(time)}). Skipping.")
        else:
            print(f"Variable {var} not found in {file_path}")
            
    plt.xlabel('Time [s]')
    plt.ylabel('Value')
    plt.title(f'Variables from {Path(file_path).name}')
    plt.legend()
    plt.grid(True)
    
    if output_path:
        plt.savefig(output_path)
        print(f"Plot saved to {output_path}")
    else:
        plt.show()

def extract_to_csv(
    file_path: str, 
    output_path: str, 
    mapping_overrides: Optional[Dict[str, str]] = None,
    zero_point: Optional[float] = None,
    start_time: Optional[float] = None,
    duration: Optional[float] = None
):
    """
    Extract data to CSV format.
    """
    data = scipy.io.loadmat(file_path)
    
    # Default mapping
    def get_q(d):
        if 'h_act' in d:
            h = d['h_act'].flatten()
            return np.arccos(np.clip((0.616 - h) / 0.37, -1.0, 1.0))
        if 'q1_act' in d: return d['q1_act'].flatten()
        return np.zeros_like(d['time'].flatten())

    def get_q_des(d):
        if 'h_des' in d:
            h = d['h_des'].flatten()
            return np.arccos(np.clip((0.616 - h) / 0.37, -1.0, 1.0))
        if 'q1_des' in d: return d['q1_des'].flatten()
        return np.zeros_like(d['time'].flatten())

    def get_dq(d):
        q = get_q(d)
        time = d['time'].flatten()
        dt = np.diff(time)
        dt = np.where(dt <= 0, 0.001, dt)
        dq = np.diff(q) / dt
        return np.append(dq, dq[-1])

    def get_dq_des(d):
        q_des = get_q_des(d)
        time = d['time'].flatten()
        dt = np.diff(time)
        dt = np.where(dt <= 0, 0.001, dt)
        dq_des = np.diff(q_des) / dt
        return np.append(dq_des, dq_des[-1])

    def get_tau_cmd(d):
        if 'outAfter0' in d: return d['outAfter0'].flatten()
        if 'tau_cmd' in d: return d['tau_cmd'].flatten()
        return np.zeros_like(d['time'].flatten())

    default_mapping = {
        'time': 'time',
        'q': get_q,
        'dq': get_dq,
        'q_des': get_q_des,
        'dq_des': get_dq_des,
        'tau_des': lambda d: d['F1_des'].flatten() if 'F1_des' in d else np.zeros_like(d['time'].flatten()),
        'position_error': lambda d: get_q_des(d) - get_q(d),
        'velocity_error': lambda d: get_dq_des(d) - get_dq(d),
        'tau_cmd': get_tau_cmd,
        'tau_applied': get_tau_cmd, # Often same in these experiments
        'integral_error': lambda d: np.zeros_like(d['time'].flatten()),
        'is_saturated': lambda d: np.zeros_like(d['time'].flatten(), dtype=bool),
    }

    if mapping_overrides:
        default_mapping.update(mapping_overrides)
            
    df_dict = {}
    for csv_col, val in default_mapping.items():
        if callable(val):
            df_dict[csv_col] = val(data)
        elif isinstance(val, str) and val in data:
            df_dict[csv_col] = data[val].flatten()
        else:
            # If val is a string but not in data, try to see if it's a constant or just missing
            df_dict[csv_col] = np.zeros_like(data['time'].flatten())
            
    df = pd.DataFrame(df_dict)
    
    # Filter by time
    if start_time is not None:
        df = df[df['time'] >= start_time]
        
    if duration is not None:
        if not df.empty:
            t0 = df['time'].iloc[0]
            df = df[df['time'] <= t0 + duration]
            
    # Set zero point (shift time)
    if zero_point is not None:
        df['time'] = df['time'] - zero_point
    elif not df.empty:
        df['time'] = df['time'] - df['time'].iloc[0]

    df.to_csv(output_path, index=False)
    print(f"Data extracted to {output_path} ({len(df)} rows)")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="Data processing utilities for .mat files")
    subparsers = parser.add_subparsers(dest="command")
    
    # List command
    list_parser = subparsers.add_parser("list", help="List variables in a .mat file")
    list_parser.add_argument("file", help="Path to .mat file")
    
    # Plot command
    plot_parser = subparsers.add_parser("plot", help="Plot variables from a .mat file")
    plot_parser.add_argument("file", help="Path to .mat file")
    plot_parser.add_argument("vars", nargs="+", help="Variables to plot")
    plot_parser.add_argument("--out", help="Output plot file path")
    
    # Extract command
    extract_parser = subparsers.add_parser("extract", help="Extract data to CSV")
    extract_parser.add_argument("file", help="Path to .mat file")
    extract_parser.add_argument("out", help="Output CSV file path")
    extract_parser.add_argument("--start", type=float, help="Start time in original data")
    extract_parser.add_argument("--duration", type=float, help="Duration to extract")
    extract_parser.add_argument("--zero", type=float, help="Time value in original data to treat as 0")
    
    args = parser.parse_args()
    
    if args.command == "list":
        variables = list_variables(args.file)
        print("Variables in file:")
        for v in variables:
            print(f"  {v}")
            
    elif args.command == "plot":
        plot_variables(args.file, args.vars, args.out)
        
    elif args.command == "extract":
        extract_to_csv(args.file, args.out, None, args.zero, args.start, args.duration)
