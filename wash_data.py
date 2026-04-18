import scipy.io
import pandas as pd
import numpy as np
import os

files = [
    ('adr-step005-kf1.mat', 'adr_step005_kf1.csv'),
    ('adr-sweep-kf1.mat', 'adr_sweep_kf1.csv'),
    ('sinsweep.mat', 'sinsweep.csv'),
    ('step-kf1.mat', 'step_kf1.csv'),
]

base_dir = '/Users/gujun/Developer/ActDiag/real_data/20250627_Jun'
out_dir = '/Users/gujun/Developer/ActDiag/real_data/processed_csv'

if not os.path.exists(out_dir):
    os.makedirs(out_dir)

for mat_file, csv_file in files:
    mat_path = os.path.join(base_dir, mat_file)
    data = scipy.io.loadmat(mat_path)
    
    time = data['time'].flatten()
    h_des = data['h_des'].flatten()
    h_act = data['h_act'].flatten()
    
    # Calculate q from h_act using formula: h = 0.616 - 0.37 * cos(q)
    # => cos(q) = (0.616 - h) / 0.37
    # => q = arccos((0.616 - h) / 0.37)
    cos_q = (0.616 - h_act) / 0.37
    cos_q = np.clip(cos_q, -1.0, 1.0)
    q = np.arccos(cos_q)
    
    # Skip transient garbage at the very start
    mask = np.ones_like(h_act, dtype=bool)
    if mat_file in ['adr-step005-kf1.mat', 'step-kf1.mat']:
        mask[:5] = False
    
    time = time[mask]
    q = q[mask]
    h_des = h_des[mask]
    h_act = h_act[mask]
    
    if len(time) < 2:
        print(f"Skipping {mat_file} due to lack of data after filtering")
        continue

    # Numerical differentiation for dq
    dt = np.diff(time)
    dt[dt <= 0] = 0.002 
    dq = np.diff(q) / dt
    dq = np.append(dq, dq[-1]) 
    
    df = pd.DataFrame({
        'time': time,
        'q': q,
        'dq': dq,
        'h_des': h_des,
        'h_act': h_act
    })
    
    out_path = os.path.join(out_dir, csv_file)
    df.to_csv(out_path, index=False)
    print(f"Processed {mat_file} -> {csv_file} (rows: {len(df)})")
