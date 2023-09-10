import os
from fnmatch import fnmatch

import numpy as np
import pandas as pd
import ruamel.yaml as ry

from pCrunch import LoadsAnalysis, PowerProduction
from pCrunch.io import load_FAST_out
from pCrunch.utility import save_yaml, get_windspeeds, convert_summary_stats

def valid_extension(fp):
    return any([fnmatch(fp, ext) for ext in ["*.outb", "*.out"]])

if __name__ == '__main__':
    
    ## PROJECT DIRECTORY
    
    project_dir = "/Users/aoifework/Documents/Research/ipc_tuning/"
    data_dir = os.path.join(project_dir, 'controller_simulations')
    results_dir = os.path.join(project_dir, "postprocessing_results")
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
        
    save_results = True
    
    outfiles = [
        os.path.join(data_dir, f) for f in os.listdir(data_dir)
        if valid_extension(f)
    ]
    
    print(f"Found {len(outfiles)} files.")
    
    ## INTERACTING WITH OUTPUT FILES
    
    # The new framework provides an object oriented framework to interact with
    # output files. The easiest way to use this is to use the 'load_FAST_out' function.
    
    outputs = load_FAST_out(outfiles[:3])
    outputs
    
    # An instance of 'OpenFASTBinary' (or 'OpenFASTAscii' if applicable) is created.
    # The instance stores the raw data but also provides many useful methods for
    # interacting with the data:
    
    # print(outputs[0].data)
    # print(outputs[0].time)
    # print(outputs[0].channels)
    # print(outputs[0].maxima)
    # print(outputs[0].stddevs)
    
    # Individual channel time series can also be accessed with dict style indexing:
    outputs[0]["Wind1VelX"]
    
    ## PCRUNCH CONFIGURATION
    
    # Channel magnitudes are defined in a dict:
    magnitude_channels = {
        "RootMc1": ["RootMxc1", "RootMyc1", "RootMzc1"],
        "RootMc2": ["RootMxc2", "RootMyc2", "RootMzc2"],
        "RootMc3": ["RootMxc3", "RootMyc3", "RootMzc3"],
    }
    
    # Define channels (and their fatigue slopes) in a dict:
    fatigue_channels = {
        "RootMc1": 10,
        "RootMc2": 10,
        "RootMc3": 10
    }
    
    # Define channels to save extreme data in a list:
    channel_extremes = [
        "RotSpeed",
        "RotThrust",
        "RotTorq",
        "RootMc1",
        "RootMc2",
        "RootMc3",
    ]
    
    ## RUN PCRUNCH
    
    # The API has changed and is in more of an object oriented framework.
    la = LoadsAnalysis(
        outfiles[:5],  # The primary input is a list of output files
        magnitude_channels=magnitude_channels,  # All of the following inputs are optional
        fatigue_channels=fatigue_channels,  #
        extreme_channels=channel_extremes,  #
        trim_data=(0,),  # If 'trim_data' is passed, all input files will
    )  # be trimmed to (tmin, tmax(optional))
    
    la.process_outputs(cores=4)  # Once LoadsAnalysis is configured, process outputs with
    # `process_outputs`. `cores` is optional but will trigger parallel processing if configured
    
    ## OUTPUTS
    # The summary stats per each file are here:
    la.summary_stats