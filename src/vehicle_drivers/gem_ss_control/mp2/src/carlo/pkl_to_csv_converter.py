import pickle as pkl
import pandas as pd
with open("runs/highway_u1_Tue; Jun 21 2022; 15.15.51.pkl", "rb") as f:
    object = pkl.load(f)
    
df = pd.DataFrame(object)
df.to_csv(r'runs/highway_u1_Tue; Jun 21 2022; 15.15.51.csv')