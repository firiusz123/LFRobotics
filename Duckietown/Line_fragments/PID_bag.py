#!/usr/bin/env python3
import bagpy
from bagpy import bagreader
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
import numpy as np

b = bagreader('2025-02-18-15-00-23.bag')
LASER_MSG = b.message_by_topic('/d3/polyfit_node/image/error')
print(b.topic_table)
df_data = pd.read_csv(LASER_MSG)
for i in df_data:
    print(i)


data = df_data['data']
time = df_data['Time']
print(time[len(time)-1] - time[0] )
dt = (time[len(time)-1] - time[0])/len(data) 
print(dt)
print(dt*79 - dt*36)