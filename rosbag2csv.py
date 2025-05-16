from bagpy import bagreader
import pandas as pd
import numpy as np
from math import pi
#alvinxy.py
from math import *
import numpy as np
import readline
import glob
import os
# Got trouble with the import of alvinxy, so I just copied the function
x=1
def  mdeglat(lat):
    '''
    Provides meters-per-degree latitude at a given latitude
    
    Args:
      lat (float): latitude

    Returns:
      float: meters-per-degree value
    '''
    latrad = lat*2.0*pi/360.0 ;

    dy = 111132.09 - 566.05 * cos(2.0*latrad) \
         + 1.20 * cos(4.0*latrad) \
         - 0.002 * cos(6.0*latrad)
    return dy


def mdeglon(lat):
    '''
    Provides meters-per-degree longitude at a given latitude

    Args:
      lat (float): latitude in decimal degrees

    Returns:
      float: meters per degree longitude
    '''
    latrad = lat*2.0*pi/360.0 
    dx = 111415.13 * cos(latrad) \
         - 94.55 * cos(3.0*latrad) \
	+ 0.12 * cos(5.0*latrad)
    return dx

def complete_path(text, state):
    return (glob.glob(text + '*') + [None])[state]


def ll2xy(lat, lon, orglat, orglon):
    '''
    AlvinXY: Lat/Long to X/Y
    Converts Lat/Lon (WGS84) to Alvin XYs using a Mercator projection.

    Args:
      lat (float): Latitude of location
      lon (float): Longitude of location
      orglat (float): Latitude of origin location
      orglon (float): Longitude of origin location

    Returns:
      tuple: (x,y) where...
        x is Easting in m (Alvin local grid)
        y is Northing in m (Alvin local grid)
    '''
    x = (lon - orglon) * mdeglon(orglat)
    y = (lat - orglat) * mdeglat(orglat)
    return (x,y)


readline.set_completer_delims(' \t\n;')
readline.set_completer(complete_path)
readline.parse_and_bind('tab: complete')
# Prompt for bag file path
bag_path = input("Enter the path to your .bag file: ")
bag_name = bag_path.split('/')[-1].split('.')[0]
# Read the bag file
b = bagreader(bag_path)

# Extract the CSV file for the topic
csv_file = b.message_by_topic('/septentrio_gnss/insnavgeod')

# Read it with pandas
df = pd.read_csv(csv_file)

# Remove the first row if it's a duplicate header
df = df.iloc[1:].reset_index(drop=True)

# Select only the required columns
selected_columns = ['longitude', 'latitude', 'heading']

df_selected = df[selected_columns].copy()

df_selected.loc[:, ['longitude', 'latitude']] = np.degrees(df_selected[['longitude', 'latitude']])

x_list, y_list = ll2xy(
    df_selected['latitude'],
    df_selected['longitude'],
    df_selected['latitude'].iloc[0],
    df_selected['longitude'].iloc[0]
)
# Add new columns to the DataFrame
df_selected['x'] = x_list
df_selected['y'] = y_list
df_final = df_selected[['x', 'y', 'heading']]
df_sampled = df_final[::4].round(4)
output_file = f'{bag_name}.csv'
df_sampled.to_csv(output_file, index=False, header=False)
print(f"Saved selected data to: {output_file}")












































